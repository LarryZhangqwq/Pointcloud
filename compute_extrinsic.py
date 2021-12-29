import numpy as np
import open3d as o3d
from argparse import ArgumentParser
import os

parser = ArgumentParser()
parser.add_argument("--red",type=float, default= 0.5)
parser.add_argument("--blue", type=float, default = 0.4)
parser.add_argument("--green", type=float, default = 0.4)
parser.add_argument("--visualize",action="store_true", default = False)
parser.add_argument("--extrinsic_name", type = str, default = "extrinsic")
args = parser.parse_args()

def computeTF(pointcloud_registers):
    top = np.zeros(3,dtype=np.float)
    bottom = np.zeros(3,dtype=np.float)
    left = np.zeros(3,dtype=np.float)
    right = np.zeros(3,dtype=np.float)
    for pc in pointcloud_registers:
        
        points = np.asarray(pc.points)
        top += points[np.argmax(points[:,1])]
        bottom += points[np.argmin(points[:,1])]
        left += points[np.argmin(points[:,0])]
        right += points[np.argmax(points[:,0])]
    
    top /= len(pointcloud_registers)
    bottom /= len(pointcloud_registers)
    left /= len(pointcloud_registers)
    right /= len(pointcloud_registers)
    
    R = np.zeros((3,3))
    R[:,0] = (right - left)/np.linalg.norm(right-left,2)
    R[:,1] = (top - bottom)/np.linalg.norm(top - bottom,2)
    R[:,2] = np.cross(R[:,0],R[:,1])
    R = R.T

    # set the point with minimum x then minimum y as origin
    T = -(top + bottom+left+right)/4
    return R, T

def process_point_cloud(pcd):
    color = np.asarray(pcd.colors).squeeze()
    pcd.colors = o3d.utility.Vector3dVector(color)
    mask = (color[:,0] > args.red) * (color[:,1] < args.green) * (color[:,2] < args.blue)
    points = np.asarray(pcd.points).squeeze()
    truncated_pcd = o3d.geometry.PointCloud()
    truncated_pcd.points = o3d.utility.Vector3dVector(points[mask])
    truncated_pcd.colors = o3d.utility.Vector3dVector(color[mask])
    truncated_pcd, _ = truncated_pcd.remove_statistical_outlier(nb_neighbors=4, std_ratio=0.10)
    return truncated_pcd

filelist = os.listdir("./pointcloud_raw/")
truncated_pcds = []
for file in filelist:
    filename = "./pointcloud_raw/"+file
    pcd = o3d.io.read_point_cloud(filename)
    truncated_pcd = process_point_cloud(pcd)
    truncated_pcds.append(truncated_pcd)
if args.visualize:
    o3d.visualization.draw_geometries([truncated_pcds[-1]])

R, T = computeTF(truncated_pcds)
np.savez("./extrinsic/"+args.extrinsic_name+".npz", R = R, T = T)




