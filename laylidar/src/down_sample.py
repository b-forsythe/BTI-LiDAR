#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from typing import List

class VoxelGridFilter(Node):
    def __init__(self):
        super().__init__('voxel_grid_filter')
        self.publisher_ = self.create_publisher(PointCloud2, '/downsampled_points', 10)
        qos_profile = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)        
        self.subscription_ = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.listener_callback,
            qos_profile)

    def voxel_filter(self, points:np.ndarray, voxel_size:float) -> np.ndarray:
        # Assign each point to a voxel
        voxels = np.floor(points / voxel_size)
        
        # Remove duplicates -> this leaves one point per occupied voxel
        downsampled = np.unique(voxels, axis=0)
        
        # Scale back to original coordinate system
        downsampled = downsampled * voxel_size + voxel_size / 2  # voxel_size / 2 to shift to voxel center

        return downsampled

    def listener_callback(self, msg):
        pc_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        np_pc = np.array([list(p) for p in pc_gen])

        # Handle potential nan and inf values
        np_pc = np_pc[~np.isnan(np_pc).any(axis=1)]
        np_pc = np_pc[~np.isinf(np_pc).any(axis=1)]

        # Print out the point cloud array and its shape and data type
        print(np_pc)
        print(np_pc.shape)
        print(np_pc.dtype)

        # Perform voxel grid downsampling
        voxel_size = 0.6
        downsampled = self.voxel_filter(np_pc, voxel_size)

        # Convert back to PointCloud2 message
        downsampled_msg = pc2.create_cloud_xyz32(msg.header, downsampled)

        self.publisher_.publish(downsampled_msg)

def main(args=None):
    rclpy.init(args=args)
    voxel_grid_filter = VoxelGridFilter()
    rclpy.spin(voxel_grid_filter)
    voxel_grid_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()