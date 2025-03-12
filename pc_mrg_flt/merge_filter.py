import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import message_filters
from scipy.spatial.transform import Rotation
import pcl
import struct

class PointFusionNodePCL(Node):
    def __init__(self):
        super().__init__('point_fusion_node_pcl')

        # Parameters
        self.declare_parameter('activate_filter', True)                     # Flag to activate/deactivate the filter
        self.declare_parameter('voxel_size_lsc', 0.1)                       # size of the voxels to apply the voxel-filter
        self.declare_parameter('outlier_radius', 1.0)                       # radius for the outliers in the filter
        self.declare_parameter('min_neighbors', 20)                         # minimal number of points inside the voxel 
        
        self.declare_parameter('cin_1', 'cloud_in1')                        # Name of first cloud to be mergerd and filtered
        self.declare_parameter('cin_2', 'cloud_in2')                        # Name of second cloud to be mergerd and filtered
        self.declare_parameter('cout_m', 'cloud_out_merged')                # Name of first cloud to be mergerd and filtered
        self.declare_parameter('cout_mf', 'cloud_out_merged_filtered')      # Name of second cloud to be mergerd and filtered

        # Sensor transform parameters
        self.declare_parameter('sensor1_x', 0.0)
        self.declare_parameter('sensor1_y', 0.0)
        self.declare_parameter('sensor1_z', 0.0)
        self.declare_parameter('sensor1_roll', 0.0)
        self.declare_parameter('sensor1_pitch', 0.0)
        self.declare_parameter('sensor1_yaw', 0.0)

        self.declare_parameter('sensor2_x', 0.0)
        self.declare_parameter('sensor2_y', 0.0)
        self.declare_parameter('sensor2_z', 0.0)
        self.declare_parameter('sensor2_roll', 0.0)
        self.declare_parameter('sensor2_pitch', 0.0)
        self.declare_parameter('sensor2_yaw', 0.0)

        # store paremeters in object
        self.filter_active = self.get_parameter('activate_filter').value
        self.cin1 = self.get_parameter('cin_1').value
        self.cin2 = self.get_parameter('cin_2').value
        self.cout_m = self.get_parameter('cout_m').value
        self.cout_mf = self.get_parameter('cout_mf').value

        # Subscribers and Publishers
        self.pc1_sub = message_filters.Subscriber(self, PointCloud2, self.cin1)
        self.pc2_sub = message_filters.Subscriber(self, PointCloud2, self.cin1)
        self.get_logger().info(f'SUBSCRIBED to:\n{self.cin1}\n{self.cin2}')

        self.merged_pc_pub = self.create_publisher(PointCloud2, self.cout_m, 10)
        self.get_logger().info(f'Publish on:\n{self.cout_m}')
        if self.filter_active:
            self.filtered_pc_pub = self.create_publisher(PointCloud2, self.cout_mf, 10)
            self.get_logger().info(f'Filtering activated.\nPublish on:\n{self.cout_mf}')
        else:
            self.get_logger().info(f'Filtering deactivated.')

        # Time synchronizers
        self.ts_pc = message_filters.ApproximateTimeSynchronizer([self.pc1_sub, self.pc2_sub], 10, 0.1)
        self.ts_pc.registerCallback(self.pointcloud_callback)

        # Get transform matrices
        self.transform1 = self.get_transform_matrix('sensor1')
        self.transform2 = self.get_transform_matrix('sensor2')

    def get_transform_matrix(self, sensor_name):
        x = self.get_parameter(f'{sensor_name}_x').value
        y = self.get_parameter(f'{sensor_name}_y').value
        z = self.get_parameter(f'{sensor_name}_z').value
        roll = self.get_parameter(f'{sensor_name}_roll').value
        pitch = self.get_parameter(f'{sensor_name}_pitch').value
        yaw = self.get_parameter(f'{sensor_name}_yaw').value

        r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]
        return transform

    def pointcloud_to_array(self, cloud_msg):
        points_list = []
        for i in range(0, len(cloud_msg.data), cloud_msg.point_step):
            x, y, z = struct.unpack_from('fff', cloud_msg.data, offset=i)
            points_list.append([x, y, z])
        return np.array(points_list)

    def array_to_pointcloud(self, points, header, msg1):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = msg1.fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def filter_points(self, points, voxel_size):
        cloud = pcl.PointCloud()
        cloud.from_array(points)
        vox = cloud.make_voxel_grid_filter()
        vox.set_leaf_size(voxel_size, voxel_size, voxel_size)
        cloud_filtered = vox.filter()
        # Statistical Outlier Removal
        outlier_radius = self.get_parameter('outlier_radius').value
        min_neighbors = self.get_parameter('min_neighbors').value
        
        sor = cloud_filtered.make_statistical_outlier_filter()
        sor.set_mean_k(min_neighbors)
        sor.set_std_dev_mul_thresh(outlier_radius)
        cloud_filtered = sor.filter()
        return cloud_filtered

    def filter_and_merge_pointclouds(self, pc1_array, pc2_array):
        # Transform point clouds
        pc1_homogeneous = np.hstack([pc1_array, np.ones((pc1_array.shape[0], 1))])
        pc2_homogeneous = np.hstack([pc2_array, np.ones((pc2_array.shape[0], 1))])
        
        pc1_transformed = (self.transform1 @ pc1_homogeneous.T).T[:, :3]
        pc2_transformed = (self.transform2 @ pc2_homogeneous.T).T[:, :3]

        # Convert to PCL point clouds
        cloud1 = pcl.PointCloud()
        cloud1.from_array(pc1_transformed.astype(np.float32))
        cloud2 = pcl.PointCloud()
        cloud2.from_array(pc2_transformed.astype(np.float32))
        
        merged_points = np.vstack([cloud1.to_array(), cloud2.to_array()])
        if self.filter_active:
            voxel_size = self.get_parameter('voxel_size_lsc').value
            cloud_m_filtered = self.filter_points(merged_points, voxel_size)
            filtered_points = np.vstack([cloud_m_filtered.to_array()])
        else:
            filtered_points = None
        
        return (merged_points, filtered_points)

    def pointcloud_callback(self, pc1_msg, pc2_msg):
        pc1_array = self.pointcloud_to_array(pc1_msg)
        pc2_array = self.pointcloud_to_array(pc2_msg)
        (merged_points, filtered_points) = self.filter_and_merge_pointclouds(pc1_array, pc2_array)
        
        # Generate and publish Pointclouds as Msg
        merged_msg = self.array_to_pointcloud(merged_points, pc1_msg.header, pc1_msg)
        self.merged_pc_pub.publish(merged_msg)

        if self.filter_active:
            filtered_msg = self.array_to_pointcloud(filtered_points, pc1_msg.header, pc1_msg)
            self.filtered_pc_pub.publish(filtered_msg)


def main(args=None):
    self.get_logger().info(f'Starting point-cloud2-merge-filter-ros2-humble')
    self.get_logger().info(f'++++++++++++++++++++++++++++++++++++++++++++++')
    self.get_logger().info(f'https://github.com/FabianJohnThl/point-cloud2-merge-filter-ros2-humble')
    self.get_logger().info(f'Developed: jpschreiter, FabianJohnTHL')
    self.get_logger().info(f'License: https://github.com/FabianJohnThl/point-cloud2-merge-filter-ros2-humble/blob/main/LICENSE')
    self.get_logger().info(f'Cite as: Publicatin pending')
    self.get_logger().info(f'++++++++++++++++++++++++++++++++++++++++++++++')
    rclpy.init(args=args)
    node = PointFusionNodePCL()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
