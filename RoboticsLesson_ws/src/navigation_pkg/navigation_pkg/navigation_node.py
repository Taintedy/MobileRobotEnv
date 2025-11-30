import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
import numpy as np
import time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException



class NavigationNode(Node):
    """
    NavigationNode:
    ----------------
    This node receives obstacle data as a PointCloud2 and a goal position as a PoseStamped.
    Students must implement a path planner that computes a path from the robot's
    current pose (obtained via TF) to the goal pose.

    The node:
      • Subscribes to /obstacle_pointcloud  → receives obstacles
      • Subscribes to /robot_goal           → receives a goal PoseStamped
      • Publishes to /path                  → publishes a nav_msgs/Path
      • Uses TF2 to get robot pose in the 'map' frame
    """
    def __init__(self):
        super().__init__('navigation_node')

        # TODO: Placeholder for the path planner (must be assigned)
        self.planner = None

        # Subscriber: obstacle point cloud
        self.pointcloud_sub = self.create_subscription(PointCloud2,'/obstacle_pointcloud', self.pointcloud_callback, 1)
        # Subscriber: goal pose
        self.goal_sub = self.create_subscription(PoseStamped,'/robot_goal', self.planner_callback, 1)

        # Publisher: path result
        self.path_publisher = self.create_publisher(Path, '/path', 1) 

        # TF buffer + listener to read robot pose from TF tree
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pc2_to_xyz_array(self, msg:PointCloud2):
        """
        Convert a PointCloud2 message into an (N, 3) NumPy array of XYZ points.

        Parameters:
            msg (PointCloud2): Incoming point cloud message.

        Returns:
            np.ndarray: Nx3 array of float32 values containing the (x, y, z) coordinates.
        """
        
        N = msg.width * msg.height # Number of points in the cloud

        # Extract raw bytes of the point cloud into a NumPy array
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(N, msg.point_step)

        # The first 12 bytes of each point correspond to x, y, z (float32)
        # View as float32 and reshape to (N, 3)
        xyz = buf[:, 0:12].view(np.float32).reshape(N, 3)
  
        return xyz.copy()

    def pointcloud_callback(self, msg):
        """
        Callback for /obstacle_pointcloud topic.
        Converts PointCloud2 → NumPy array and stores it.
        """
        self.pointcloud =  self.pc2_to_xyz_array(msg)

                
        
    
    def planner_callback(self, msg:PoseStamped):
        """
        Callback for /robot_goal topic.
        Triggered whenever a new goal is published.

        Steps:
          1. Get robot's current position from TF ("Robot" → "map")
          2. Extract goal position from msg
          3. Run student planner to compute a path
          4. Convert path to nav_msgs/Path and publish it
        """

        # Ensure that a planner has been assigned
        if self.planner is not None:

            try:
                # Lookup the robot pose in the "map" frame
                tf = self.tf_buffer.lookup_transform(
                    "map",
                    "Robot",
                    rclpy.time.Time()
                )
                # Robot start position (x, y)
                start = np.array([tf.transform.translation.x, tf.transform.translation.y])
                # Goal position (x, y)
                goal = np.array([msg.pose.position.x, msg.pose.position.y])

                #####################################################################
                ## TODO: implement your planner here
                ## Example:

                ## planned_path = self.planner.planning(start, goal)
                

                #####################################################################
                self.get_logger().info(f"---- Planning successful ----")
                self.get_logger().info(f"Start Pose {start}")
                self.get_logger().info(f"Goal Pose {goal}")
                self.get_logger().info(f"Path length {len(planned_path)} points")
                
                # Create a Path message
                path = Path()
                path.header.frame_id = 'map'
                path.header.stamp = self.get_clock().now().to_msg()
                # Convert each path point to a PoseStamped and append
                for point in planned_path:
                    pose = PoseStamped()
                    pose.pose.position.x = (float)(point)
                    pose.pose.position.y = (float)(point)
                    path.poses.append(pose)
                # Publish the computed path
                self.path_publisher.publish(path)

            except TransformException as ex:
                self.get_logger().info(f"Could not get transform for point cloud: {ex}")
                return
        else:
            self.get_logger().info("Pointcloud is empty")
            



def main(args=None):
    """
    Entry point for the ROS2 node.
    Initializes rclpy, creates the node, and spins it.
    """
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()