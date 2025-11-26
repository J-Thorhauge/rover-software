import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray
import numpy as np

class ElevationMapNode(Node):
    def __init__(self):
        super().__init__('elevation_map_node')
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.pointcloud_callback,
            10)
        self.map_sub = self.create_subscription(
            GridMap,
            '/rtabmap/map',
            self.map_callback,
            10)
        self.publisher = self.create_publisher(GridMap, '/elevation_map', 10)

        # Grid map parameters
        self.resolution = 0.1  # meters per cell
        self.size_x = 100  # number of cells
        self.size_y = 100
        self.origin_x = -5.0  # meters
        self.origin_y = -5.0

    def pointcloud_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        elevation_grid = np.full((self.size_x, self.size_y), np.nan)

        for x, y, z in points:
            i = int((x - self.origin_x) / self.resolution)
            j = int((y - self.origin_y) / self.resolution)
            if 0 <= i < self.size_x and 0 <= j < self.size_y:
                if np.isnan(elevation_grid[i, j]):
                    elevation_grid[i, j] = z
                else:
                    elevation_grid[i, j] = max(elevation_grid[i, j], z)

        # Create GridMap message
        grid_map_msg = GridMap()
        grid_map_msg.layers.append("elevation")
        grid_map_msg.data.append(Float32MultiArray(data=elevation_grid.flatten().tolist()))
        grid_map_msg.info.resolution = self.resolution
        grid_map_msg.info.length_x = self.size_x * self.resolution
        grid_map_msg.info.length_y = self.size_y * self.resolution
        grid_map_msg.info.pose.position.x = self.origin_x
        grid_map_msg.info.pose.position.y = self.origin_y

        self.publisher.publish(grid_map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ElevationMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs_py.point_cloud2 as pc2
# from grid_map_msgs.msg import GridMap
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Pose

# class ElevationMapNode(Node):
#     def __init__(self):
#         super().__init__('elevation_map_node')
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/rtabmap/cloud_map',
#             self.pointcloud_callback,
#             10)
#         self.publisher = self.create_publisher(GridMap, '/elevation_map', 10)

#         # Grid map parameters
#         self.resolution = 0.1  # meters per cell
#         self.size_x = 100  # number of cells
#         self.size_y = 100
#         self.origin_x = -5.0  # meters
#         self.origin_y = -5.0

#     def pointcloud_callback(self, msg):
#         points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
#         elevation_grid = np.full((self.size_x, self.size_y), np.nan)

#         for x, y, z in points:
#             i = int((x - self.origin_x) / self.resolution)
#             j = int((y - self.origin_y) / self.resolution)
#             if 0 <= i < self.size_x and 0 <= j < self.size_y:
#                 if np.isnan(elevation_grid[i, j]):
#                     elevation_grid[i, j] = z
#                 else:
#                     elevation_grid[i, j] = max(elevation_grid[i, j], z)

#         # Create GridMap message
#         grid_map_msg = GridMap()
#         grid_map_msg.layers.append("elevation")
#         grid_map_msg.data.append(Float32MultiArray(data=elevation_grid.flatten().tolist()))
#         grid_map_msg.info.resolution = self.resolution
#         grid_map_msg.info.length_x = self.size_x * self.resolution
#         grid_map_msg.info.length_y = self.size_y * self.resolution

#         pose = Pose()
#         pose.position.x = self.origin_x
#         pose.position.y = self.origin_y
#         pose.position.z = 0.0
#         grid_map_msg.info.pose = pose

#         self.publisher.publish(grid_map_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ElevationMapNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #####

# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# from rtabmap_msgs.msg import MapData
# import sensor_msgs_py.point_cloud2 as pc2
# from grid_map_msgs.msg import GridMap
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Pose

# class ElevationMapNode(Node):
#     def __init__(self):
#         super().__init__('elevation_map_node')
#         self.subscription_cloud = self.create_subscription(
#             PointCloud2,
#             '/rtabmap/cloud_map',
#             self.pointcloud_callback,
#             10)
#         self.subscription_mapdata = self.create_subscription(
#             MapData,
#             '/rtabmap/mapData',
#             self.mapdata_callback,
#             10)
#         self.publisher = self.create_publisher(GridMap, '/elevation_map', 10)

#         self.resolution = 0.1  # meters per cell
#         self.origin_x = 0.0
#         self.origin_y = 0.0
#         self.size_x = 100
#         self.size_y = 100
#         self.map_bounds_initialized = False

#     def mapdata_callback(self, msg):
#         # Determine map bounds from mapData poses
#         min_x = min_y = float('inf')
#         max_x = max_y = float('-inf')

#         for pose in msg.graph.poses:
#             x = pose.position.x
#             y = pose.position.y
#             min_x = min(min_x, x)
#             max_x = max(max_x, x)
#             min_y = min(min_y, y)
#             max_y = max(max_y, y)

#         # Add padding and update grid size
#         self.origin_x = min_x - 5.0
#         self.origin_y = min_y - 5.0
#         self.size_x = int((max_x - min_x) / self.resolution) + 100
#         self.size_y = int((max_y - min_y) / self.resolution) + 100
#         self.map_bounds_initialized = True

#     def pointcloud_callback(self, msg):
#         if not self.map_bounds_initialized:
#             self.get_logger().info("Waiting for mapData to initialize bounds...")
#             return

#         points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
#         elevation_grid = np.full((self.size_x, self.size_y), np.nan)

#         for x, y, z in points:
#             i = int((x - self.origin_x) / self.resolution)
#             j = int((y - self.origin_y) / self.resolution)
#             if 0 <= i < self.size_x and 0 <= j < self.size_y:
#                 if np.isnan(elevation_grid[i, j]):
#                     elevation_grid[i, j] = z
#                 else:
#                     elevation_grid[i, j] = max(elevation_grid[i, j], z)

#         grid_map_msg = GridMap()
#         grid_map_msg.layers.append("elevation")
#         grid_map_msg.data.append(Float32MultiArray(data=elevation_grid.flatten().tolist()))
#         grid_map_msg.info.resolution = self.resolution
#         grid_map_msg.info.length_x = self.size_x * self.resolution
#         grid_map_msg.info.length_y = self.size_y * self.resolution

#         pose = Pose()
#         pose.position.x = self.origin_x
#         pose.position.y = self.origin_y
#         pose.position.z = 0.0
#         grid_map_msg.info.pose = pose

#         self.publisher.publish(grid_map_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ElevationMapNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
