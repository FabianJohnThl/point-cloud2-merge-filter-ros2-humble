# point-cloud2-merge-filter-ros2-humble
This project merges two PointCloud2 messages and apply a filter to downscale the resulting PointCloud2. Developed and tested with Ubuntu 22.04 and ROS2 (humble).

# Contributors

[jpschreiter](https://github.com/jpschreiter)

[FabianJohnThl](https://github.com/FabianJohnThl)

# Installation

```bash
sudo apt install -y ros-humble-pcl-ros python3-pcl pcl-tools
cd ~/ros2_ws/src
sudo rm -r point-cloud2-merge-filter-ros2-humble
git clone https://github.com/FabianJohnThl/point-cloud2-merge-filter-ros2-humble.git
cd ..

colcon build --packages-select pc_mrg_flt
source ./install/setup.bash
ros2 launch pc_mrg_flt pc_mrg_flt_launch.py
```

# Configuration

- The script merges the PointCloud2 from the topics: `cloud_in1` and `cloud_in2`
- The merged cloud (unfiltered) is published under: `cloud_out_merged`
- The merged and filtered/downsampled cloud (if activated) is published under: `cloud_out_merged_filtered`
- refer the remappings in the launch file to map your own topics to be used with the script
- the script is parametrized via the parameters in the launch file

```
'activate_filter': True,        # activation of the filter functionality
'voxel_size': 0.1,          # size of the voxels to apply the voxel-filter
'outlier_radius': 1.0,          # radius for the outliers in the filter
'min_neighbors': 20,            # minimal number of points inside the voxel                     
'sensor1_x': 0.0,               # Transformation-Matrix for Sensor 1
'sensor1_y': 0.0,
'sensor1_z': 0.0,
'sensor1_roll': 0.0,
'sensor1_pitch': 0.0,
'sensor1_yaw': 0.0,
'sensor2_x': 0.0,               # Transformation-Matrix for Sensor 2
'sensor2_y': 0.0,
'sensor2_z': 0.0,
'sensor2_roll': 0.0,
'sensor2_pitch': 0.0,
'sensor2_yaw': 0.0
```

# Run

```bash
ros2 launch pc_mrg_flt pc_mrg_flt_launch.py
```