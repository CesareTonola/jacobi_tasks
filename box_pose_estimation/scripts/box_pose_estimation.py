import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt

print_figs = True

def segment_box(data):

    # crop the point cloud using predefined bounds to isolate the pallet area,
    # keeping a margin to account for variations in pallet positioning.
    # Filter out points beyond a certain distance to remove the background.
    min_bound_crop = np.array([-1.5, -2.0, -0.75], dtype=np.float64)
    max_bound_crop = np.array([ 1.0, -0.5, -0.10], dtype=np.float64)
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound_crop, max_bound_crop)
    data_cropped = data.crop(bbox)

    # some noise from other obejcts in the scene may remain,
    # use a clustering algorithm to keep the biggest object (the box)
    labels = np.array(data_cropped.cluster_dbscan(eps=0.02, min_points=20))
    valid_labels = labels[labels >= 0]

    if len(valid_labels) == 0:
        raise RuntimeError("No cluster available")

    largest_cluster_id = np.argmax(np.bincount(valid_labels))
    return data_cropped.select_by_index(np.where(labels == largest_cluster_id)[0])

if __name__ == "__main__":

    # load data
    color_array = np.load('../data/one-box.color.npdata.npy')
    depth_array = np.load('../data/one-box.depth.npdata.npy')
    intrinsics  = np.load('../data/intrinsics.npy')
    extrinsics  = np.load('../data/extrinsics.npy')

    # original color image
    if print_figs:
        plt.figure("Color: Original", figsize=(10, 8))
        plt.imshow(color_array, cmap='gray')
        plt.show(block=False)

    # filter the color image
    color_array_clean = np.clip(color_array, 0, 255)

    print(f"Depth ranges from {depth_array.min()} to {depth_array.max()}, with data type {depth_array.dtype}")

    # create rgbd of the image
    grey = color_array_clean.astype(np.uint8)
    color_rgb = np.stack((grey, grey, grey), axis=-1)
    rgb_image = o3d.geometry.Image(color_rgb)

    depth_scaled = depth_array.astype(np.float32)
    depth_image = o3d.geometry.Image(depth_scaled)

    # depth_scale = 1.0 beacuse depth is in meters
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_image, depth_image, depth_scale = 1.0, convert_rgb_to_intensity = False)

    if print_figs:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10), num="RGBD", gridspec_kw={'width_ratios': [1, 1]})
        ax1.imshow(rgbd.color)
        ax1.set_title("Color")
        depth_im = ax2.imshow(rgbd.depth)
        ax2.set_title("Depth")

        cbar_ax = fig.add_axes([0.25, 0.1, 0.5, 0.03]) 
        cbar = fig.colorbar(depth_im, cax=cbar_ax, orientation='horizontal')
        cbar.set_label("Depth (m)")

        plt.show(block=False)

        plt.pause(2.0) # time for the plot windows to open

    # convert rgbd to pointcloud
    print(f"Intrinsics:\n {intrinsics}")
    print(f"Extrinsics:\n {extrinsics}")

    extrinsics[0:3, 3] /= 1000.0 # convert transaltion part from mm to meters

    print(f"Extrinsics (translation in meters):\n {extrinsics}")

    height = color_array.shape[0]
    width  = color_array.shape[1]

    fx = intrinsics[0,0]
    fy = intrinsics[1,1]
    cx = intrinsics[0,2]
    cy = intrinsics[1,2]

    pinhole_intrinsics = o3d.camera.PinholeCameraIntrinsic(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_intrinsics, extrinsics) #pcd expressed in the 'world' frame

    points = np.asarray(pcd.points)
    print("PointCloud depth range:", points[:, 2].min(), "to", points[:, 2].max())
    print("PointCloud X range:", points[:, 0].min(), "to", points[:, 0].max())
    print("PointCloud Y range:", points[:, 1].min(), "to", points[:, 1].max())

    # crop the pcd to isolate the box
    pcd_filtered = segment_box(pcd)

    # find the minimal boundig box to the upper surface of the box and then get its coordinate
    obb = pcd_filtered.get_minimal_oriented_bounding_box()
    obb.color = (0, 0, 1)

    pose_world = np.eye(4)
    pose_world[0:3,0:3] = obb.R
    pose_world[0:3,3] = obb.center
    pose_camera =  np.linalg.inv(extrinsics) @ pose_world

    print("Pose in 'world' frame:\n", pose_world)
    print("Pose in 'camera' frame:\n", pose_camera)

    # add a frame corresponding to the pose of the box
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25) 
    frame.transform(pose_world)

    # pcd.paint_uniform_color([0.6, 0.6, 0.6])   
    # pcd_filtered.paint_uniform_color([1.0, 0.0, 0.0]) 
    # o3d.visualization.draw_geometries([pcd, pcd_filtered, obb, frame])

    o3d.visualization.draw_geometries([pcd, obb, frame])

    if print_figs:
        plt.pause(10000)
