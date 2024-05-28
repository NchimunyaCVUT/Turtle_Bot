#!/usr/bin/env python3

import numpy as np
import rospy
from scipy.ndimage import morphology
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D
import tf2_ros
from aro_msgs.srv import GenerateFrontier, GenerateFrontierRequest, GenerateFrontierResponse
from aro_msgs.srv import PlanPath, PlanPathRequest, PlanPathResponse
from typing import Optional
from rospy.exceptions import ROSInterruptException

invalid_pose = Pose2D(np.nan, np.nan, np.nan)

def get_grid_position(pose2d: Pose2D, grid_info: MapMetaData) -> np.ndarray:
    #pos = np.array([0, 0, 0, 1])
    # TODO convert the pose to grid position
    pos = np.array([
        int(pose2d.x / grid_info.resolution),
        int(pose2d.y / grid_info.resolution)
    ])
    
    return pos



def grid_to_map_coordinates(position: np.ndarray, grid_info: MapMetaData) -> np.ndarray:
    # position is (2,) numpy array
    # TODO convert the grid position to map pose
    pos = np.array([
    position[0] * grid_info.resolution,
    position[1] * grid_info.resolution
    ])

    return pos


class FrontierExplorer:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "icp_map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_diameter = float(rospy.get_param("~robot_diameter", 0.8))
        self.min_frontier_size = rospy.get_param("~min_frontier_size", 3)
        self.occupancy_threshold = int(rospy.get_param("~occupancy_threshold", 90))

        # Helper variable to determine if grid was received at least once
        self.grid_ready = False
        self.stale_wfd = True  # True if WFD was not recomputed for the newest grid

        # You may wish to listen to the transformations of the robot
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Path planner can be utilized for testing reachability

        try:
            rospy.wait_for_service('plan_path', timeout=30)  # Wait for 30 seconds
            self.plan_path = rospy.ServiceProxy('plan_path', PlanPath)
        except ROSInterruptException:
            rospy.logerr("ROS was shutdown while waiting for the plan_path service.")
            return
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout or other error while waiting for plan_path service: {str(e)}")
            return

        #rospy.wait_for_service('plan_path')
        #self.plan_path = rospy.ServiceProxy('plan_path', PlanPath)

        self.vis_pub = rospy.Publisher('frontier_vis', MarkerArray, queue_size=2)
        self.vis_map_pub = rospy.Publisher('frontier_map', OccupancyGrid, queue_size=2)

        # The services and other variables are initialized once an occupancy grid has been received
        self.grf_service: Optional[rospy.ServiceProxy] = None
        self.gcf_service: Optional[rospy.ServiceProxy] = None
        self.gbv_service: Optional[rospy.ServiceProxy] = None
        self.grg_service: Optional[rospy.ServiceProxy] = None

        # TODO: you may wish to do initialization of other variables
        self.wfd = None  # TODO: implement WFD computation helper
        self.robot_position: Optional[np.ndarray] = None
        self.robot_vis_position: Optional[np.ndarray] = None
        self.resolution: Optional[float] = None
        self.occupancy_grid: Optional[np.ndarray] = None
        self.visibility_grid: Optional[np.ndarray] = None
        self.origin_pos: Optional[np.ndarray] = None
        self.vis_origin_pos: Optional[np.ndarray] = None
        self.grid_info: Optional[MapMetaData] = None
        self.vis_grid_info: Optional[MapMetaData] = None

        # Subscribe to grids
        self.grid_subscriber = rospy.Subscriber('occupancy', OccupancyGrid, self.grid_cb)
        self.vis_grid_subscriber = rospy.Subscriber('visible_occupancy', OccupancyGrid, self.vis_grid_cb)

    def compute_wfd(self):
        """ Run the Wavefront detector """
        rospy.loginfo('Finding frontiers')
        frontiers = []
        
        # TODO:
        # TODO: First, you should try to obtain the robots coordinates

        try:
            # Listen for the transform from the map frame to the robot's base frame
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))

            # Convert the transform to a Pose2D
            robot_pose = Pose2D()
            robot_pose.x = trans[0]
            robot_pose.y = trans[1]
            robot_pose.theta = self.tf_listener.transformations.euler_from_quaternion(rot)[2]
        except (self.tf_listener.LookupException, self.tf_listener.ConnectivityException, self.tf_listener.ExtrapolationException):
            rospy.logerr("TF error when trying to get robot pose")
            return
        # try:
        #     # Listen for the transform from the map frame to the robot's base frame using tf2_ros buffer
        #     trans = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time(0))
        #     robot_pose = Pose2D()
        #     robot_pose.x = trans.transform.translation.x
        #     robot_pose.y = trans.transform.translation.y
        #     robot_pose.theta = self.quaternion_to_euler(
        #         trans.transform.rotation.x,
        #         trans.transform.rotation.y,
        #         trans.transform.rotation.z,
        #         trans.transform.rotation.w
        #     )

        # except tf2_ros.LookupException:
        #     rospy.logerr("No transform available.")
        #     return
        # except tf2_ros.ConnectivityException:
        #     rospy.logerr("Connectivity Issue.")
        #     return
        # except tf2_ros.ExtrapolationException:
        #     rospy.logerr("Extrapolation Issue.")
        #     return

        # TODO: Then, copy the occupancy grid into some temporary variable and inflate the obstacles
        # Copy the occupancy grid
        temp_grid = np.copy(OccupancyGrid)
        inflated_grid = morphology.binary_dilation(temp_grid, iterations=self.inflation_radius)

        # Inflate the obstacles
        for i in range(temp_grid.shape[0]):
            for j in range(temp_grid.shape[1]):
                # If this cell is an obstacle
                if temp_grid[i, j] == 100:
                    # Inflate it
                    for di in range(-self.inflation_radius, self.inflation_radius + 1):
                        for dj in range(-self.inflation_radius, self.inflation_radius + 1):
                            # If this cell is within the grid
                            if 0 <= i + di < temp_grid.shape[0] and 0 <= j + dj < temp_grid.shape[1]:
                                # Mark it as an obstacle
                                temp_grid[i + di, j + dj] = 100

      
        # TODO: Run the WFD algorithm - see the presentation slides for details on how to implement it
        queuem = []
        queuef = []

        # Start with the robot's current position
        start_pos = self.get_robot_grid_position(robot_pose)  # Implement this method
        queuem.append(start_pos)
        self.map_open_list.add(start_pos)

        while queuem:
            p = queuem.pop(0)
            if p in self.map_close_list:
                continue

            if self.is_frontier_point(p):
                queuef.append(p)
                self.frontier_open_list.add(p)

                new_frontier = []
                while queuef:
                    q = queuef.pop(0)
                    if q in self.map_close_list or q in self.frontier_close_list:
                        continue

                    if self.is_frontier_point(q):
                        new_frontier.append(q)
                        for w in self.adj(q):  # Implement adjacency list generator
                            if w not in self.frontier_open_list and w not in self.frontier_close_list and w not in self.map_close_list:
                                queuef.append(w)
                                self.frontier_open_list.add(w)
                        self.frontier_close_list.add(q)

                # Save the new frontier
                self.frontiers.append(new_frontier)
                for point in new_frontier:
                    self.map_close_list.add(point)

            for v in self.adj(p):
                if v not in self.map_open_list and v not in self.map_close_list and self.is_map_open_space(v):  # Implement is_map_open_space
                    queuem.append(v)
                    self.map_open_list.add(v)
            self.map_close_list.add(p)

            def adj(self, point):
                """
                Generate adjacent points (4-connectivity)
                """
                directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                x, y = point
                return [(x + dx, y + dy) for dx, dy in directions if self.is_within_bounds(x + dx, y + dy)]

            def is_within_bounds(self, x, y):
                return 0 <= x < self.grid_info.width and 0 <= y < self.grid_info.height


    def publish_frontiers_vis(self, frontiers: np.ndarray):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = marker.DELETEALL
        marker.header.frame_id = self.map_frame
        marker_array.markers.append(marker)
        self.vis_pub.publish(marker_array)

        marker_array = MarkerArray()
        marker_id = 0
        for f in frontiers:
            fx = [f.middle()[0] for f in frontiers]
            fy = [f.middle()[1] for f in frontiers]
            for i in range(len(f.points)):
                marker = Marker()
                marker.ns = 'frontier'
                marker.id = marker_id
                marker_id += 1
                marker.header.frame_id = self.map_frame
                marker.type = Marker.CUBE
                marker.action = 0
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = self.resolution
                x, y = grid_to_map_coordinates(np.array([f.points[i][1], f.points[i][0]]), self.grid_info)
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker_array.markers.append(marker)
        self.vis_pub.publish(marker_array)

    def get_random_frontier(self, request: GenerateFrontierRequest) -> GenerateFrontierResponse:
        """ Return random frontier """
        if self.stale_wfd:
            self.compute_wfd()
        frontiers = self.wfd.frontiers
        if len(frontiers) == 0:
            return GenerateFrontierResponse(invalid_pose)
        frontier = np.random.choice(frontiers)
        
        frontier_center = frontier.middle()
        x, y = grid_to_map_coordinates(np.array([frontier_center[0], frontier_center[1]]), self.grid_info)

        response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
        rospy.loginfo('Returning random frontier at ' + str(x) + ' ' + str(y))
        return response

    def get_closest_frontier(self, request: GenerateFrontierRequest) -> GenerateFrontierResponse:
        """ Return frontier closest to the robot """
        rospy.loginfo('Finding closest frontier.')
        if self.stale_wfd:
            self.compute_wfd()
        frontiers = self.wfd.frontiers
        if len(frontiers) == 0:
            return GenerateFrontierResponse(invalid_pose)

        #best_frontier_idx = 0  # TODO: compute the index of the closest frontier, continue from here
        robot_pos_grid = grid_to_map_coordinates(np.array([self.robot_pose.x, self.robot_pose.y]), self.grid_info)
        best_frontier_idx = np.argmin([np.linalg.norm(np.array(frontier.middle()) - robot_pos_grid) for frontier in frontiers])
        frontier = frontiers[best_frontier_idx]
        frontier_center = frontier.middle()
        x, y = grid_to_map_coordinates(np.array([frontier_center[1], frontier_center[0]]), self.grid_info)
        response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
        rospy.loginfo('Returning closest frontier at ' + str(x) + ' ' + str(y))
        return response

    def get_best_value_frontier(self, request: GenerateFrontierRequest) -> GenerateFrontierResponse:
        """ Return largest frontier """
        rospy.loginfo('Finding best value frontier.')
        if self.stale_wfd:
            self.compute_wfd()
        frontiers = self.wfd.frontiers
        if len(frontiers) == 0:
            return GenerateFrontierResponse(invalid_pose)

        best_frontier_idx = 0  # TODO: compute the index of the best frontier
        # Compute the index of the best value frontier
        best_frontier_idx = np.argmax([frontier.size() for frontier in frontiers])

        frontier = frontiers[best_frontier_idx]


        frontier_center = frontier.middle()
        x, y = grid_to_map_coordinates(np.array([frontier_center[1], frontier_center[0]]), self.grid_info)

        response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
        rospy.loginfo('Returning best frontier at '+str(x)+' '+str(y))
        return response

    def get_random_goal(self, request: GenerateFrontierRequest) -> GenerateFrontierResponse:
        """ Return random empty cell in the grid """
        rospy.loginfo('Finding random goal position.')
        self.get_robot_coordinates()
        robot_position = np.array([self.robot_position[1], self.robot_position[0]])
        goal_position = np.array([0, 0])
        # TODO: get the coordinates of a random empty cell
        # Get the indices of all empty cells in the occupancy grid
        empty_cells = np.argwhere(self.occupancy_grid == 0)
        
        # Choose a random empty cell
        random_index = np.random.choice(len(empty_cells))
        goal_position = empty_cells[random_index]
        
        x, y = grid_to_map_coordinates(np.array([goal_position[1], goal_position[0]]), self.grid_info)
        response = GenerateFrontierResponse(Pose2D(x=x, y=y, theta=0.0))
        rospy.loginfo('Returning random position at .'+str(x)+' '+str(y))
        return response

    def get_robot_coordinates(self):
        """ Get the current robot position in the grid """
        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            self.robot_position = None
            self.robot_vis_position = None
        else:
            self.robot_position = get_grid_position(trans.transform.translation, self.grid_info).astype(int)
            self.robot_vis_position = \
                get_grid_position(trans.transform.translation, self.vis_grid_info).astype(int)

    def extract_grid(self, msg):
        width, height, self.resolution = msg.info.width, msg.info.height, msg.info.resolution
        self.origin_pos = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.grid_info = msg.info
        self.occupancy_grid = np.reshape(msg.data, (height, width))

    def extract_vis_grid(self, msg):
        width, height = msg.info.width, msg.info.height
        self.vis_origin_pos = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.vis_grid_info = msg.info
        self.visibility_grid = np.reshape(msg.data, (height, width))

    def grid_cb(self, msg):
        self.extract_grid(msg)
        self.stale_wfd = True
        if not self.grid_ready:
            # TODO: Do some initialization of necessary variables
            self.wfd = None
             # Extract the grid from the message
            self.grid = self.extract_grid(msg)

            # TODO: Do some initialization of necessary variables
            self.frontiers = []  # List to store the frontiers
            self.visited_cells = set()  # Set to store the visited cells
            self.frontier_map = np.zeros_like(self.grid)  # Map to store the frontiers

            # Create services
            self.grf_service = rospy.Service('get_random_frontier', GenerateFrontier, self.get_random_frontier)
            self.gcf_service = rospy.Service('get_closest_frontier', GenerateFrontier, self.get_closest_frontier)
            self.gbv_service = rospy.Service('get_best_value_frontier', GenerateFrontier, self.get_best_value_frontier)
            self.grg_service = rospy.Service('get_random_goal_pose', GenerateFrontier, self.get_random_goal)
            self.grid_ready = True

    def vis_grid_cb(self, msg):
        self.extract_vis_grid(msg)


if __name__ == "__main__":
    rospy.init_node("frontier_explorer")
    fe = FrontierExplorer()
    rospy.spin()
