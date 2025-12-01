
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
import numpy as np
import time

from tf2_geometry_msgs import do_transform_pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException



class TestingNode(Node):

    def __init__(self):
        super().__init__('testing_node')


        self.goal_poses = np.array([[-3, 3],
                                   [7, -10],
                                   [6, -6]])
        self.current_goal = 0
        self.get_logger().info(f"Sending setpoint command {self.goal_poses[self.current_goal]}")
        self.goal_publisher = self.create_publisher(PoseStamped, '/robot_goal', 1)
        
        self.publish_point(self.goal_poses[self.current_goal])
        
        self.timer = self.create_timer(1/5, self.testing_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    
    def testing_callback(self):
    
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "Robot",
                rclpy.time.Time()
            )
            robot_pose = np.array([tf.transform.translation.x, tf.transform.translation.y])
            goal = self.goal_poses[self.current_goal]
            if np.linalg.norm(goal - robot_pose) <= 0.25:
                self.get_logger().info(f"Goal reached!")
                self.current_goal += 1
                if self.current_goal > len(self.goal_poses) - 1:
                    self.current_goal = len(self.goal_poses) - 1
                    rclpy.shutdown() 
                goal = self.goal_poses[self.current_goal]
                self.publish_point(self.goal_poses[self.current_goal])
                self.get_logger().info(f"Sending setpoint command {self.goal_poses[self.current_goal]}")
            

        except TransformException as ex:
            self.get_logger().info(f"Could not get transform for point cloud: {ex}")
            return

            
    def publish_point(self, point):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = (float)(point[0])
        pose.pose.position.y = (float)(point[1])

        self.goal_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = TestingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
