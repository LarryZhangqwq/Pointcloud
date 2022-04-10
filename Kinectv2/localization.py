# Originally from Larry Zhang
import sys
import os
import math
import threading
import rospy
import ros_numpy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField, PointCloud2
from std_msgs.msg import Float32MultiArray
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
        self.param = np.load( "./params.npz" )
        self.R = self.param['R']
        self.T = self.param['T']
        self.t = time.time()
        
        rospy.init_node('pcd_listener', anonymous=True)
        rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.process)
        self.pub = rospy.Publisher('loc_xy', Float32MultiArray, queue_size=10)

        rospy.spin()

    def extract_color( self,pcd, remove_nans = True ):
        self.pcd_array = ros_numpy.point_cloud2.pointcloud2_to_array(pcd)
        self.pcd_array = ros_numpy.point_cloud2.split_rgb_field(self.pcd_array)

        if remove_nans:
             mask = np.isfinite(self.pcd_array['x']) & np.isfinite(self.pcd_array['y']) & np.isfinite(self.pcd_array['z'])
             cloud_array = self.pcd_array[mask]     
  
        points = np.zeros( cloud_array.shape + (3,), dtype=np.float )

        points[...,0] = cloud_array['r']/255
        points[...,1] = cloud_array['g']/255
        points[...,2] = cloud_array['b']/255

        return points

    def caluate_regression_plane(self, points, label):

        mean = np.zeros(3,dtype=np.float)
        for i in points:
            mean += i
        mean /= len(points)

        b, x2 = 0.0, 0.0  
        for i in points:
            b  += ( i[0] - mean[0] ) * ( i[1] - mean[1] )
            x2 += ( i[0] - mean[0] ) * ( i[0] - mean[0] )
        b /= x2
        a = mean[1] - b * mean[0] # y = bx + a     bx - y + a = 0

        k = - 1 / b

        maxn = points[np.argmax(points[:,2])]

        judge = b * maxn[0] - maxn[1] + a 

        dir_x = 1 / math.sqrt(k * k + 1)
        dir_y = k / math.sqrt(k * k + 1)
        final1 = np.array([mean[0], mean[1]])+ 0.22 * np.array([dir_x, dir_y])
        
        dir_x = -1 / math.sqrt(k * k + 1)
        dir_y = -k / math.sqrt(k * k + 1)
        final2 = np.array([mean[0], mean[1]])+ 0.22 * np.array([dir_x, dir_y])

        res1 = b * final1[0] - final1[1] + a
        res2 = b * final1[0] - final1[1] + 1

        if res1 * judge > 0: 
            loc_xy = Float32MultiArray()
            loc_xy.data = [label] + final1.tolist()
            self.pub.publish(loc_xy)
        if res2 * judge > 0:
            loc_xy = Float32MultiArray()
            loc_xy.data = [label] + final2.tolist()
            self.pub.publish(loc_xy)

    def crop( self, pcd ):
        xyz = np.asarray(pcd.points)
        color = np.asarray(pcd.colors)
        mask = ( xyz[ :, 2 ] < 0.5 ) * ( xyz[ :, 2 ] > 0.15 ) * ( xyz[ :,1 ] < 2 ) * ( xyz[ :, 0] < 4 )
        xyz = xyz[mask]
        color = color[mask]

        mask = ( color[ :,1 ] > 0.8 ) * ( color[ :,2 ] > 0.8 ) * ( color[ :,0 ] > 0.8 )
        xyz = xyz[mask]
        color = color[mask]

        color[ :, 0 ], color[ :, 1 ], color[ :, 2 ] = 0, 0, 0      
        
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(color)

        pcd,ind = pcd.remove_statistical_outlier( 25, 0.05 )

        # o3d.visualization.draw_geometries([pcd])

        # self.caluate_regression_plane( np.asarray(pcd.points), mean )
        return pcd

    def cluster(self, pcd):
        colors = np.asarray(pcd.colors)
        points = np.asarray(pcd.points)
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array( pcd.cluster_dbscan( 0.1, 30, print_progress = True ) )
        max_label = labels.max() + 1
        for i in range( max_label ):
            # pcdd = o3d.geometry.PointCloud()
            # pcdd.points = o3d.utility.Vector3dVector( points[ labels == i ] )
            # pcdd.colors = o3d.utility.Vector3dVector( colors[ labels == i ] )
            self.caluate_regression_plane(points[labels==i], i)
            # colors[ labels == i ] = 1
            # pcd.colors = o3d.utility.Vector3dVector(colors[:,:3])
            # o3d.visualization.draw_geometries([pcdd])
        # print( max_label)
        # colors[ labels < 0 ] = 1
        # colors[ labels == 1 ] = 0.8 

        # pcd.colors = o3d.utility.Vector3dVector(colors[:,:3])
        # o3d.visualization.draw_geometries([pcd])
        return pcd

    def process(self, cloud_msg):
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_msg)
        colors = self.extract_color(cloud_msg)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(-points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        pcd.translate(self.T)
        pcd.rotate(self.R, center=( 0, 0, 0 )) 

        pcd = self.crop(pcd)
        pcd = self.cluster(pcd)

        self.cnt += 1
        
        
if __name__ == "__main__":
    reader = Reader()

