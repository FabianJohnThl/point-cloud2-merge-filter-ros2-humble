# point-cloud2-merge-filter-ros2-humble
This project merges two PointCloud2 messages and apply a filter to downscale the resulting PointCloud2. Developed and tested with Ubuntu 22.04 and ROS2 (humble).

# Contributors

[jpschreiter](https://github.com/jpschreiter)

[FabianJohnThl](https://github.com/FabianJohnThl)

# Installation

```bash
cd ~/ros2_ws/src
sudo rm -r point-cloud2-merge-filter-ros2-humble
git clone https://github.com/FabianJohnThl/point-cloud2-merge-filter-ros2-humble.git
cd ..

colcon build --packages-select pc_mrg_flt
source ./install/setup.bash
ros2 launch pc_mrg_flt pc_mrg_flt_launch.py
```