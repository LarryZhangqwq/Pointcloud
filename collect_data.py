import sys
import os


import rospy
import ros_numpy
from sensor_msgs.msg import PointField, PointCloud2
from sensor_msgs import point_cloud2
import open3d as o3d
import time
import ctypes
import struct
from std_msgs.msg import Header
import numpy as np

# This is a callback function
class Reader:
    def __init__(self):
        self.np_points = None
        self.np_colors = None
        self.last_update_time = time.time()
        self.cnt = 0
        self.pointcloud = o3d.geometry.PointCloud()
        self.param = np.load("./extrinsic/extrinsic.npz")
        self.R = self.param['R']
        self.T = self.param['T']
        rospy.init_node('pcd_listener', anonymous=True)
        # rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.listener)
        rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.vis)
        rospy.spin()
        
    def extract_color(self,pcd, remove_nans = True):
        self.pcd_array = ros_numpy.point_cloud2.pointcloud2_to_array(pcd)
        self.pcd_array = ros_numpy.point_cloud2.split_rgb_field(self.pcd_array)
        if remove_nans:
            mask = np.isfinite(self.pcd_array['x']) & np.isfinite(self.pcd_array['y']) & np.isfinite(self.pcd_array['z'])
            cloud_array = self.pcd_array[mask]     
  
        points = np.zeros(cloud_array.shape + (3,), dtype=np.float)
        points[...,0] = cloud_array['r']/255
        points[...,1] = cloud_array['g']/255
        points[...,2] = cloud_array['b']/255
        return points
        
    def listener(self, cloud_msg):
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_msg)
        colors = self.extract_color(cloud_msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(-points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud("pointcloud_raw/"+str(self.cnt)+"_points.ply", pcd)
        o3d.visualization.draw_geometries([pcd])
        self.cnt += 1

    def vis(self, cloud_msg):
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_msg)
        colors = self.extract_color(cloud_msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(-points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        pcd.translate(self.T)
        pcd.rotate(self.R, center=(0,0,0))
        o3d.visualization.draw_geometries([pcd])
        self.cnt += 1

    def time_cnt(self, ros_point_cloud):
        time.time()
        print("Time: ",time.time() - self.last_update_time)
        self.last_update_time = time.time()
        
        
if __name__ == "__main__":
    reader = Reader()