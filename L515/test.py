# Originally from Larry Zhang
import sys
import os
import math
import threading
import rospy
import ros_numpy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField, PointCloud2
# from std_msgs.msg import Float32MultiArray
import open3d as o3d
import time
import ctypes
import struct
from std_msgs.msg import Header
import numpy as np

class Reader:

    def __init__(self):
        self.np_points = None
        self.np_colors = None
        self.truncated_pcds = []
        self.cnt = 0
        self.pointcloud = o3d.geometry.PointCloud()
        # self.param = np.load( "./params.npz" )
        # self.R = self.param['R']
        # self.T = self.param['T']
        # self.t = time.time()
        
        rospy.init_node('pcd_listener', anonymous=True)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.process)
        # self.pub = rospy.Publisher('loc_xy', Float32MultiArray, queue_size=10)

        rospy.spin()


    def process(self, cloud_msg):
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_msg)
        # colors = self.extract_color(cloud_msg)

        #pcd = o3d.geometry.PointCloud()
        #pcd.points = o3d.utility.Vector3dVector(-points)
        #pcd.colors = o3d.utility.Vector3dVector(colors)
        #o3d.visualization.draw_geometries([pcd])
        # pcd.translate(self.T)
        # pcd.rotate(self.R, center=( 0, 0, 0 )) 

        # pcd = self.crop(pcd)
        # pcd = self.cluster(pcd)

        # self.cnt += 1
        
        
if __name__ == "__main__":
    reader = Reader()

