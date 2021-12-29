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
        rospy.init_node('pcd_listener', anonymous=True)
        # rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.listener)
        rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.time_cnt)
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
        
    def callback(self, ros_point_cloud):
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        gen = point_cloud2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)
        pcd = np.concatenate([xyz, rgb], axis=1)
        print(pcd.shape)
        
        np.save(f"{self.cnt}_state.npy", pcd)
        self.cnt += 1
        
    def listener(self, cloud_msg):
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_msg)
        colors = self.extract_color(cloud_msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(f"{self.cnt}_points.ply", pcd)
        time.time()
        print("Time: ",time.time() - self.last_update_time)
        self.last_update_time = time.time()
        self.cnt += 1
        
    def time_cnt(self, ros_point_cloud):
        time.time()
        print("Time: ",time.time() - self.last_update_time)
        self.last_update_time = time.time()
        
        
if __name__ == "__main__":
    reader = Reader()
