import pyrealsense2 as rs
import numpy as np
import rospy
import cv2
import pandas as pd
from matplotlib import pyplot as plt
from sklearn.cluster import AgglomerativeClustering
from sklearn.neighbors import kneighbors_graph


# Configure depth and color streams of the camera
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)


aligned_stream = rs.align(rs.stream.color) # alignment between color and depth
point_cloud = rs.pointcloud()
#blob_pub  = rospy.Publisher("/connector/position_connector",Point,queue_size=1)

# Start streaming
profile = pipeline.start(config)

def take_picture():
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.resize(color_image, dsize=(depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_AREA)

    return depth_image, color_image


def get_points(color, depth=None):
    _,thresh2 = cv2.threshold(color,40,255,cv2.THRESH_BINARY_INV)
    
    points = np.transpose(np.nonzero(thresh2))
    
    knn_graph = kneighbors_graph(points, 30, include_self=False)
    clustering = AgglomerativeClustering(n_clusters=30, linkage="ward", connectivity=knn_graph)
    clustering.fit(points)

    df = pd.DataFrame(points)
    #print(df)
    df['cluster'] = clustering.labels_
    #print(df)
    df.columns = ["X",'Y', 'f', 'cluster']
    df['Y'] = df['Y'] * (-1)

    dfg = df.groupby('cluster').agg({'X':[np.mean], 'Y':[np.mean]}).reset_index()
    dfg.columns = ['cluster', 'X', 'Y']
    dfg = dfg.apply(round)
    dfg['X'] = dfg['X'] / 400.0
    dfg['Y'] = (dfg['Y'] + 400.0) / 400.0
    dfg.drop(columns=['cluster'])
    dfg['Z'] = 0.0
    return dfg


def main():
    depth_img, color_img = take_picture()
    
    #wire_points = get_points(color_img)
    print("inicio")
    plt.imshow(color_img)
    plt.show()
    #print(wire_points)
    #wire_points.to_csv('./points_xyz.csv')


if __name__ == '__main__':
    main()