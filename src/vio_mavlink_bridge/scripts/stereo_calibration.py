#!/usr/bin/env python3
"""
Stereo Camera Calibration Script for VIO

Performs P1/P2 stereo calibration for RealSense T265 or OAK-D cameras
to improve VIO accuracy.

Author: Barath Kumar JK
Date: 2025

Usage:
    ros2 run vio_mavlink_bridge stereo_calibration.py \\
        --board-size 9x6 \\
        --square-size 0.025 \\
        --output-file config/stereo_calib.yaml
"""

import argparse
import sys
import os
import numpy as np
import cv2
import yaml
from datetime import datetime
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS 2 not available, using standalone mode")


class StereoCalibrator:
    """Stereo camera calibration using chessboard pattern."""
    
    def __init__(
        self,
        board_size: tuple = (9, 6),
        square_size: float = 0.025,
        min_samples: int = 20,
        max_samples: int = 50,
    ):
        """
        Initialize stereo calibrator.
        
        Args:
            board_size: Chessboard inner corners (width, height)
            square_size: Size of each square in meters
            min_samples: Minimum calibration samples
            max_samples: Maximum calibration samples
        """
        self.board_size = board_size
        self.square_size = square_size
        self.min_samples = min_samples
        self.max_samples = max_samples
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0), ...
        self.objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Storage for calibration points
        self.obj_points = []  # 3D points
        self.img_points_left = []  # 2D points left
        self.img_points_right = []  # 2D points right
        
        self.image_size = None
        self.calibration_flags = (
            cv2.CALIB_FIX_ASPECT_RATIO |
            cv2.CALIB_ZERO_TANGENT_DIST |
            cv2.CALIB_SAME_FOCAL_LENGTH
        )
        
    def find_chessboard(self, image: np.ndarray) -> tuple:
        """
        Find chessboard corners in image.
        
        Args:
            image: Grayscale or BGR image
            
        Returns:
            (success, corners) tuple
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
            
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH |
            cv2.CALIB_CB_NORMALIZE_IMAGE |
            cv2.CALIB_CB_FAST_CHECK
        )
        
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, flags)
        
        if ret:
            # Refine corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
        return ret, corners
    
    def add_sample(
        self,
        img_left: np.ndarray,
        img_right: np.ndarray,
    ) -> bool:
        """
        Add a calibration sample from stereo image pair.
        
        Args:
            img_left: Left camera image
            img_right: Right camera image
            
        Returns:
            True if sample was added successfully
        """
        if self.image_size is None:
            self.image_size = (img_left.shape[1], img_left.shape[0])
            
        ret_l, corners_l = self.find_chessboard(img_left)
        ret_r, corners_r = self.find_chessboard(img_right)
        
        if ret_l and ret_r:
            self.obj_points.append(self.objp)
            self.img_points_left.append(corners_l)
            self.img_points_right.append(corners_r)
            return True
            
        return False
    
    def calibrate(self) -> dict:
        """
        Perform stereo calibration.
        
        Returns:
            Dictionary with calibration results
        """
        if len(self.obj_points) < self.min_samples:
            raise ValueError(
                f"Not enough samples: {len(self.obj_points)} < {self.min_samples}"
            )
            
        print(f"Calibrating with {len(self.obj_points)} samples...")
        
        # Individual camera calibration
        print("Calibrating left camera...")
        ret_l, K_l, D_l, rvecs_l, tvecs_l = cv2.calibrateCamera(
            self.obj_points,
            self.img_points_left,
            self.image_size,
            None,
            None,
            flags=self.calibration_flags,
        )
        
        print("Calibrating right camera...")
        ret_r, K_r, D_r, rvecs_r, tvecs_r = cv2.calibrateCamera(
            self.obj_points,
            self.img_points_right,
            self.image_size,
            None,
            None,
            flags=self.calibration_flags,
        )
        
        # Stereo calibration
        print("Performing stereo calibration...")
        stereo_flags = (
            cv2.CALIB_FIX_INTRINSIC |
            cv2.CALIB_USE_INTRINSIC_GUESS
        )
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        
        ret_stereo, K_l, D_l, K_r, D_r, R, T, E, F = cv2.stereoCalibrate(
            self.obj_points,
            self.img_points_left,
            self.img_points_right,
            K_l, D_l,
            K_r, D_r,
            self.image_size,
            criteria=criteria,
            flags=stereo_flags,
        )
        
        # Stereo rectification
        print("Computing rectification transforms...")
        R_l, R_r, P_l, P_r, Q, roi_l, roi_r = cv2.stereoRectify(
            K_l, D_l,
            K_r, D_r,
            self.image_size,
            R, T,
            alpha=0,
        )
        
        # Compute reprojection error
        total_error = 0
        for i in range(len(self.obj_points)):
            imgpoints2_l, _ = cv2.projectPoints(
                self.obj_points[i], rvecs_l[i], tvecs_l[i], K_l, D_l
            )
            error = cv2.norm(self.img_points_left[i], imgpoints2_l, cv2.NORM_L2)
            total_error += error ** 2
        reproj_error = np.sqrt(total_error / len(self.obj_points))
        
        print(f"Stereo calibration RMS error: {ret_stereo:.4f}")
        print(f"Reprojection error: {reproj_error:.4f}")
        
        # Prepare results
        results = {
            'calibration_date': datetime.now().isoformat(),
            'image_size': list(self.image_size),
            'num_samples': len(self.obj_points),
            'board_size': list(self.board_size),
            'square_size': self.square_size,
            'stereo_rms_error': float(ret_stereo),
            'reprojection_error': float(reproj_error),
            'left_camera': {
                'K': K_l.tolist(),
                'D': D_l.flatten().tolist(),
                'R': R_l.tolist(),
                'P': P_l.tolist(),
            },
            'right_camera': {
                'K': K_r.tolist(),
                'D': D_r.flatten().tolist(),
                'R': R_r.tolist(),
                'P': P_r.tolist(),
            },
            'stereo': {
                'R': R.tolist(),  # Rotation from left to right
                'T': T.flatten().tolist(),  # Translation from left to right
                'E': E.tolist(),  # Essential matrix
                'F': F.tolist(),  # Fundamental matrix
                'Q': Q.tolist(),  # Disparity-to-depth mapping
                'baseline': float(np.linalg.norm(T)),  # Baseline distance
            },
        }
        
        return results
    
    def save_calibration(self, results: dict, output_file: str):
        """Save calibration results to YAML file."""
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w') as f:
            yaml.dump(results, f, default_flow_style=False)
            
        print(f"Calibration saved to: {output_path}")


class CalibrationNode(Node):
    """ROS 2 node for interactive stereo calibration."""
    
    def __init__(self, calibrator: StereoCalibrator, args):
        super().__init__('stereo_calibration_node')
        
        self.calibrator = calibrator
        self.args = args
        self.bridge = CvBridge()
        
        self.img_left = None
        self.img_right = None
        self.collecting = False
        
        # Subscribers
        self.sub_left = self.create_subscription(
            Image,
            args.left_topic,
            self.left_callback,
            10,
        )
        self.sub_right = self.create_subscription(
            Image,
            args.right_topic,
            self.right_callback,
            10,
        )
        
        # Timer for display
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Stereo calibration node started')
        self.get_logger().info(f'Subscribing to: {args.left_topic}, {args.right_topic}')
        self.get_logger().info('Press SPACE to capture, Q to finish')
        
    def left_callback(self, msg):
        self.img_left = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
    def right_callback(self, msg):
        self.img_right = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
    def timer_callback(self):
        if self.img_left is None or self.img_right is None:
            return
            
        # Find chessboards
        vis_left = self.img_left.copy()
        vis_right = self.img_right.copy()
        
        ret_l, corners_l = self.calibrator.find_chessboard(self.img_left)
        ret_r, corners_r = self.calibrator.find_chessboard(self.img_right)
        
        if ret_l:
            cv2.drawChessboardCorners(
                vis_left, self.calibrator.board_size, corners_l, ret_l
            )
        if ret_r:
            cv2.drawChessboardCorners(
                vis_right, self.calibrator.board_size, corners_r, ret_r
            )
            
        # Status text
        status = f"Samples: {len(self.calibrator.obj_points)}/{self.calibrator.min_samples}"
        if ret_l and ret_r:
            status += " [READY - Press SPACE]"
        else:
            status += " [Pattern not found]"
            
        cv2.putText(vis_left, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display
        combined = np.hstack([vis_left, vis_right])
        cv2.imshow('Stereo Calibration', combined)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' '):  # Space
            if ret_l and ret_r:
                if self.calibrator.add_sample(self.img_left, self.img_right):
                    self.get_logger().info(
                        f"Sample added: {len(self.calibrator.obj_points)}"
                    )
                    
        elif key == ord('q') or key == 27:  # Q or ESC
            if len(self.calibrator.obj_points) >= self.calibrator.min_samples:
                self.finish_calibration()
            else:
                self.get_logger().warn(
                    f"Need at least {self.calibrator.min_samples} samples"
                )
                
    def finish_calibration(self):
        cv2.destroyAllWindows()
        
        try:
            results = self.calibrator.calibrate()
            self.calibrator.save_calibration(results, self.args.output_file)
            self.get_logger().info('Calibration completed successfully!')
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
            
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Stereo Camera Calibration')
    
    parser.add_argument(
        '--board-size',
        type=str,
        default='9x6',
        help='Chessboard inner corners (WxH)',
    )
    parser.add_argument(
        '--square-size',
        type=float,
        default=0.025,
        help='Chessboard square size in meters',
    )
    parser.add_argument(
        '--output-file',
        type=str,
        default='config/stereo_calib.yaml',
        help='Output calibration file',
    )
    parser.add_argument(
        '--min-samples',
        type=int,
        default=20,
        help='Minimum calibration samples',
    )
    parser.add_argument(
        '--left-topic',
        type=str,
        default='/camera/fisheye1/image_raw',
        help='Left camera topic',
    )
    parser.add_argument(
        '--right-topic',
        type=str,
        default='/camera/fisheye2/image_raw',
        help='Right camera topic',
    )
    parser.add_argument(
        '--images-dir',
        type=str,
        default=None,
        help='Directory with left/right image pairs (standalone mode)',
    )
    
    args = parser.parse_args()
    
    # Parse board size
    board_size = tuple(map(int, args.board_size.split('x')))
    
    calibrator = StereoCalibrator(
        board_size=board_size,
        square_size=args.square_size,
        min_samples=args.min_samples,
    )
    
    if args.images_dir:
        # Standalone mode with image files
        print(f"Loading images from: {args.images_dir}")
        
        images_dir = Path(args.images_dir)
        left_images = sorted(images_dir.glob('left_*.png'))
        right_images = sorted(images_dir.glob('right_*.png'))
        
        if len(left_images) != len(right_images):
            print("Error: Mismatched number of left/right images")
            sys.exit(1)
            
        for left_path, right_path in zip(left_images, right_images):
            img_left = cv2.imread(str(left_path))
            img_right = cv2.imread(str(right_path))
            
            if calibrator.add_sample(img_left, img_right):
                print(f"Added: {left_path.name}")
                
        if len(calibrator.obj_points) >= args.min_samples:
            results = calibrator.calibrate()
            calibrator.save_calibration(results, args.output_file)
        else:
            print(f"Need at least {args.min_samples} valid samples")
            
    elif ROS_AVAILABLE:
        # ROS 2 interactive mode
        rclpy.init()
        node = CalibrationNode(calibrator, args)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        
    else:
        print("Error: ROS 2 not available and no image directory specified")
        print("Use --images-dir to specify a directory with image pairs")
        sys.exit(1)


if __name__ == '__main__':
    main()
