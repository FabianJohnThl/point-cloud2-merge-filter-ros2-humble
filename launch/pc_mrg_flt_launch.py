from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pc_mrg_flt', executable='pc_mrg_flt',
            remappings=[('cloud_in1', '/livox/lidar_192_168_1_106'), # Map your first input PointCloud to cloud_in1
                        ('cloud_in2', '/livox/lidar_192_168_1_107'), # Map your second input PointCloud to cloud_in1
                        ('/livox/lidar', 'cloud_out_merged'),        # Map the merged cloud output to the desired output topic
                        ('/livox/lidar_f', 'cloud_out_merged_filtered')], # Map the merged and filtered cloud output to the desired output topic
            parameters=[{'activate_filter': True,
                        'voxel_size_lsc': 0.1,                       # size of the voxels to apply the voxel-filter
                        'outlier_radius': 1.0,                       # radius for the outliers in the filter
                        'min_neighbors': 20,                         # minimal number of points inside the voxel                     
                        'sensor1_x': 0.0,                            # Transformation-Matrix for Sensor 1
                        'sensor1_y': 0.0,
                        'sensor1_z': 0.0,
                        'sensor1_roll': 0.0,
                        'sensor1_pitch': 0.0,
                        'sensor1_yaw': 0.0,
                        'sensor2_x': 0.0,                            # Transformation-Matrix for Sensor 2
                        'sensor2_y': 0.0,
                        'sensor2_z': 0.0,
                        'sensor2_roll': 0.0,
                        'sensor2_pitch': 0.0,
                        'sensor2_yaw': 0.0}],
            name='pc_mrg_flt'
        )
    ])
