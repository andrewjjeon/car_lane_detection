import os
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import yaml

def loadPC(data_path):
  points = np.fromfile(data_path, dtype=np.float32).reshape(-1, 5)
  return points

def load_config(config_path="config.yaml"):
  with open(config_path, "r") as file:
      config = yaml.safe_load(file)
  return config

def visualizePC(pc):
  points = pc[:, :3] #only x y z
  pcd = o3d.geometry.PointCloud()
  pcd.points = o3d.utility.Vector3dVector(points)

  axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])
  o3d.visualization.draw_geometries([pcd, axes])

def ROI_filter(PC, config):
  '''
  Filter the point cloud to only the immediate area around the ego car.

  Args:
      PC (Numpy Array): The point cloud we are filtering (N, 5).
      config: config file determines the Region of Interest (ROI) we want to focus on. (xbound and ybound).

  Returns:
      ROI_PC (Numpy Array): The point cloud for only the ROI (N, 5).
  '''
  # Potential Improvements
  # sort PC[:, 0] so its in order by x coordinate, then
  # only consider PC[:n/4, 0] 4 windows and fit it for each window, windows need to rotate in the lane direction
  # then do polyfit again on the points from each of the 4 window polynomials
  xbound = config["xROI"]
  ybound = config["yROI"]
  roi_wleft = config["roi_wleft"]
  
  x_filter = (PC[:, 0] < xbound) & (PC[:, 0] > -xbound)
  y_filter = (PC[:, 1] < ybound + roi_wleft) & (PC[:, 1] > -ybound)
  ROI_PC = PC[x_filter & y_filter]

  print(f"Before ROI filter point count: {len(PC)} \n After ROI filter point count: {len(ROI_PC)}")

  return ROI_PC


def intensity_filter(PC, config):
  '''
  Filter the point cloud by intensity, generally the lowest, most common intensities are going to be asphalt.
  Lane marking should be higher intensity values. Extremely high values are probably cars, mirrors, or outlier objects.

  Args:
    PC (Numpy Array): ROI filtered PC, now being filtered by intensity value.
    config: config file determines intensity interval we are considering to be potential lane markings. (intensity_min, intensity_max).

  Returns:
    lane_PC (Numpy Array): intensity filtered PC, now only points with intensities we suspect are lane markings are considered.
  '''
  intensity_min = config["intensity_min"]
  intensity_max = config["intensity_max"]
  lane_PC = PC[(PC[:, 3] >= intensity_min) & (PC[:, 3] <= intensity_max)]

  print(f"Before intensity filter point count: {len(PC)} \n After intensity filter point count: {len(lane_PC)}")
  return lane_PC

def leftright_split(PC):
  '''
  Split the ROI and intensity filtered point cloud into left lane points and right lane points.
  The ego vehicle is located at (0, 0). 
  Upon inspection, the coordinate frame of the pointcloud data seems to be defined as 
    - x (forward-backward) 
    - y (right-left)
  Because it is an ego vehicle, left lane line can be considered negative y and right lane line can be considered positive y coordinate.

  Args:
    PC (Numpy Array): ROI and intensity filtered PC, now being split into left lane PC and right lane PC.

  Returns:
    left_PC (Numpy Array): ROI, intensity filtered, left lane PC.
    right_PC (Numpy Array): ROI, intensity filtered, right lane PC.
  '''
  # if y is neg, right lane. positive, left lane
  left_pc = PC[PC[:, 1] > 0]
  right_pc = PC[PC[:, 1] < 0]

  print(f"Left lane point count: {len(left_pc)}")
  print(f"Right lane point count: {len(right_pc)}")
  return left_pc, right_pc

if __name__ == "__main__":
  config = load_config("config.yaml")
  pc_id = config["pc_id"]
  
  bin_file = f"./pointclouds/{pc_id}.bin"
  PC = loadPC(bin_file)
  # visualizePC(PC)

  # x y z
  # intensity: 0-255 point intensity value, important to distinguish between lane point and non-lane point
  # lidar_beam: 0-31 which lidar beam is point from
  print(f"A points contents are: \n [x            y            z           intensity  lidar )] \n {PC[0]}")

  min_x = np.min(PC[:, 0])
  max_x = np.max(PC[:, 0])
  print(f"x range: {min_x} to {max_x}")

  unique_intensities, counts = np.unique(PC[:, 3], return_counts=True)
  for intensity, count in zip(unique_intensities, counts):
      print(f"Intensity: {intensity}, Count: {count}")

  lane_PC = intensity_filter(PC, config)
  ROI_PC = ROI_filter(lane_PC, config)
  left_pc, right_pc = leftright_split(ROI_PC)

  x_left = left_pc[:, 0]
  y_left = left_pc[:, 1]
  x_right = right_pc[:, 0]
  y_right = right_pc[:, 1]

  poly_left = np.polyfit(x_left, y_left, 3)
  poly_right = np.polyfit(x_right, y_right, 3)

  print(f"left polynomials: {poly_left}")
  print(f"right polynomials: {poly_right}")

  output_file = f"./outputs/{pc_id}.txt"

  with open(output_file, "w") as f:
      f.write(";".join(map(str, poly_left)) + "\n")
      f.write(";".join(map(str, poly_right)) + "\n")

  print(f"Polynomial coefficients saved to {output_file}")