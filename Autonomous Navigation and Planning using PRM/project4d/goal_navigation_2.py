#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math
from nav_msgs.msg import Path, OccupancyGrid
import tf2_ros
from A_Star import AStar
import numpy as np
from PRM import PRMPlanner
from RRT import RRT
from rclpy.duration import Duration
from disc_robot import load_disc_robot
# from RRT2 import RRTStar




class GoalNavigationNode(Node):
    def __init__(self,robot_name):
        super().__init__('goal_navigation')
        self.subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF2 Buffer and Listener for current robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Current goal
        self.current_goal = None
        self.path = None
        self.idx = 0

        # Subscribe to map data
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map_data = None
        self.navigation_rate = self.create_rate(10)  # 10 Hz rate

        # Publisher for the path
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.robot = load_disc_robot(robot_name)
        # Robot parameters
        self.radius = self.robot['body']['radius']


    def map_callback(self, msg):
        self.map_data = msg

    def goal_pose_callback(self, msg):
        # Check if goal is reachable (not within an obstacle and in reachable space)
        if self.is_goal_reachable(msg):
            self.current_goal = msg
            self.get_logger().info('New goal received, planning path')
            self.plan_and_execute_path()
        else:
            self.get_logger().info('Goal is unreachable, ignoring')

    def is_goal_reachable(self, goal_pose):

        if self.map_data is None:
            self.get_logger().info('Map data not received yet')
            return False

        # Convert goal pose to the map frame
        transformed_pose = self.transform_pose_to_map_frame(goal_pose)

        if transformed_pose is None:
            return False

        # Check if the goal is within an obstacle
        return not self.is_pose_in_obstacle(transformed_pose)

    def transform_pose_to_map_frame(self, pose_stamped):
        try:
            transform = self.tf_buffer.lookup_transform(self.map_data.header.frame_id, 
                                                        pose_stamped.header.frame_id, 
                                                        rclpy.time.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error('Error transforming pose to map frame')
            return None

        
    def is_pose_in_obstacle(self, pose_stamped):
        x = pose_stamped.position.x
        y = pose_stamped.position.y

        # Convert (x, y) to map coordinates
        mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Check if the coordinates are within map bounds
        if mx < 0 or my < 0 or mx >= self.map_data.info.width or my >= self.map_data.info.height:
            return True

        # Index in the map data array
        index = my * self.map_data.info.width + mx

        # Check if the index is valid and if the cell is occupied (100 means occupied, 0 means free)
        return index < len(self.map_data.data) and self.map_data.data[index] == 100
        

    def plan_and_execute_path(self):

        if self.current_goal is None or self.map_data is None:
            self.get_logger().info('No goal or map data available')
            return

        # Transform start and goal poses to map frame
        start_pose = self.get_current_pose()
        goal_pose = self.transform_pose_to_map_frame(self.current_goal)

        if start_pose is None or goal_pose is None:
            self.get_logger().info('Error in pose transformation')
            return

        # Prepare the grid matrix for A*
        grid_matrix = self.prepare_grid_matrix()

        # Instantiate and call A* algorithm
        # astar = AStar(grid_matrix, self.map_data.info.resolution, 
        #               (start_pose.position.x, start_pose.position.y), 
        #               (goal_pose.position.x, goal_pose.position.y))
        # astar.find_path()
        # path = astar.get_shortest_path()

        # prm = PRM(grid_matrix, self.map_data.info.resolution, (start_pose.position.x, start_pose.position.y), (goal_pose.position.x, goal_pose.position.y))
        # path = prm.find_path()
        obstacle_grid_dilated = self.dilate_obstacle_grid(grid_matrix, self.radius, self.map_data.info.resolution)
        obstacle_grid_dilated_int = obstacle_grid_dilated.astype(int)

        # rrt = RRTStar((start_pose.position.x, start_pose.position.y), (goal_pose.position.x, goal_pose.position.y),obstacle_grid_dilated_int, self.map_data.info.resolution, robot_radius = self.radius)
        # self.path = rrt.generate_path()

        # rrt = RRT(start, goal, obstacle_grid, grid_resolution, robot_radius)
        # path = rrt.generate_path()

        rrt = RRT(obstacle_grid_dilated_int,(start_pose.position.x, start_pose.position.y), (goal_pose.position.x, goal_pose.position.y), self.map_data.info.resolution, radius = self.radius)
       
        self.path =rrt.plan_path()

        # prm_planner = PRMPlanner(obstacle_grid_dilated_int, self.map_data.info.resolution)
        # self.path = prm_planner.plan_path((start_pose.position.x, start_pose.position.y), (goal_pose.position.x, goal_pose.position.y))


        print(self.path)

        if self.path is not None:
            self.publish_path(self.path)
            self.timer = self.create_timer(0.1, self.move_robot)
        else:
            self.get_logger().info("No path found")



    def move_robot(self):

        if self.path is None or self.idx >= len(self.path):
            self.get_logger().info("All goals reached")
            new_vel = Twist()
            new_vel.linear.x = 0.0
            new_vel.angular.z = 0.0
            self.velocity_publisher.publish(new_vel)
            self.timer.cancel()
            self.current_goal = None  # Allow accepting new goals
            self.path = None
            self.idx = 0
            return

        goal = self.path[self.idx]

        new_vel = Twist()
        current_pose = self.get_current_pose()
        distance_to_goal = math.sqrt((goal[0] - current_pose.position.x)**2 + (goal[1] - current_pose.position.y)**2)
        angle_to_goal = math.atan2(goal[1] - current_pose.position.y, goal[0] - current_pose.position.x)

        distance_tolerance = 0.05
        angle_tolerance = 0.001

        quater = (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
        _,_,yaw = self.get_yaw_from_pose(quater)

        angle_error = angle_to_goal - yaw

        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi

        kp_linear = 3.0# Proportional gain for linear velocity

        linear_control_effort = kp_linear * distance_to_goal
        
        
        
        kp_angular = 3.0
        angular_control_effort = kp_angular * angle_error

        if abs(angle_error) > angle_tolerance:
            new_vel.angular.z = angular_control_effort # Cap the angular velocity to avoid rapid turns
        else:
            if distance_to_goal > distance_tolerance:
                new_vel.linear.x = linear_control_effort
                new_vel.angular.z = 0.0
            else:
                new_vel.linear.x = 0.0
                self.get_logger().info("Goal reached")
                self.idx += 1

        # if abs(angle_error) > angle_tolerance and distance_to_goal < distance_tolerance:
        #     new_vel.linear.x = 0.0
        #     self.get_logger().info(f"Goal reached")
        #     self.idx += 1

        self.velocity_publisher.publish(new_vel)




    def prepare_grid_matrix(self):
        # Convert occupancy grid data to a numpy array
        grid_array = np.array(self.map_data.data).reshape(self.map_data.info.height, self.map_data.info.width)

        # Flip the grid along the vertical axis
        grid_array = np.flipud(grid_array)
        # print(grid_array)
        # Convert to binary (0: free space, 1: obstacle)
        grid_matrix = np.where(grid_array == 100, 1, 0)
        # print(grid_matrix)
        return grid_matrix
    
    def publish_path(self, path_points):

        if path_points is None:
            self.get_logger().info("No path to publish")
            return

        path_msg = Path()
        path_msg.header.frame_id = self.map_data.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.w = 1.0  # Only position matters for the path
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def get_current_pose(self):
        # Ensure that the transform is available
        try:
            # Lookup the transform from base_link to map
            # This assumes your robot's pose is represented by the base_link frame
            transform = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))

            # Create a PoseStamped to represent the current pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = 0.0
            pose_stamped.pose.position.y = 0.0
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            # Transform the pose into the map frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)

            return transformed_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Failed to get current robot pose: {e}')
            return None


    
    def get_yaw_from_pose(self,quaternion):
        # Convert quaternion to Euler angles
        x, y, z, w = quaternion
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

        
    def dilate_obstacle_grid(self,obstacle_grid, robot_radius, grid_resolution):
        from scipy.ndimage import binary_dilation
        struct = np.ones((int(np.ceil(robot_radius / grid_resolution)), int(np.ceil(robot_radius / grid_resolution))))
        return binary_dilation(obstacle_grid, structure=struct)
        

def main(args=None):
    rclpy.init(args=args)

    ## to take the robot name and world name as parameters 
    node = rclpy.create_node('temp_node_for_robot_name_retrieval')
    node.declare_parameter("robot_name",'/home/lab2004/project_ws/src/project4a/project4a/normal.robot')
    robot_name = node.get_parameter("robot_name").value
    # print(f'in diff drive node the robot name is {robot_name}')
    node.destroy_node()

    goal_navigation_node = GoalNavigationNode(robot_name)
    rclpy.spin(goal_navigation_node)
    goal_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
