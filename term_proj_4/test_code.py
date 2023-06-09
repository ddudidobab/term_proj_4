import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import math
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseWithCovarianceStamped


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.client = self.create_client(GetMap, '/map_server/map')
        self.origin = None
        self.path_publisher = self.create_publisher(Path, '/path', 10)  # Initialize the path publisher
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose',self.get_initial_pose, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose',self.get_goal_pose, 10)
        self.open_node_pub = self.create_publisher(MarkerArray, '/open_node', 10)
        self.close_node_pub = self.create_publisher(MarkerArray, '/close_node', 10)
        self.x_start = 0
        self.y_start = 0
        self.x_goal = 0
        self.y_goal = 0
        self.flag = False
        
    def get_initial_pose(self, msg):
        self.x_start = msg.pose.pose.position.x
        self.y_start = msg.pose.pose.position.y
        self.planning()
        
    def get_goal_pose(self, msg):
        self.x_goal = msg.pose.position.x
        self.y_goal = msg.pose.position.y
        self.planning()
    
    def send_request(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = GetMap.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)
    
    def handle_response(self, future):
        try:
            response = future.result()
            if response:
                # Process the received map
                map_data = response.map
                # ... do something with the map data ...
                self.get_logger().info('Map received successfully')

                self.costmap = map_data.data
                # number of columns in the occupancy grid
                self.width = map_data.info.width
                # number of rows in the occupancy grid
                self.height = map_data.info.height
                # side of each grid map square in meters
                self.resolution = map_data.info.resolution
                # origin of grid map
                self.origin_x = map_data.info.origin.position.x
                self.origin_y = map_data.info.origin.position.y

                self.get_logger().info('Map width: %d' % (self.width))
                self.get_logger().info('Map height: %d' % (self.height))
                self.get_logger().info('Map origin x: %d' % (self.origin_x))
                self.get_logger().info('Map origin y: %d' % (self.origin_y))
                self.flag = True
            else:
                self.get_logger().warn('Failed to receive map')
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            
    def planning(self):
        if(self.flag==True):
            self.get_logger().info(f'Received initial_pose | x : {round(self.x_start, 2)}, y : {round(self.y_start, 2)}')
            self.get_logger().info(f'Received goal_pose    | x : {round(self.x_goal, 2)}, y : {round(self.y_goal, 2)}')

            x_start_grd, y_start_grd = self.from_world_to_grid(self.x_start, self.y_start)                
            self.start_index = self.from_grid_to_index(x_start_grd, y_start_grd)
            x_goal_grd, y_goal_grd = self.from_world_to_grid(self.x_goal, self.y_goal)                
            self.goal_index = self.from_grid_to_index(x_goal_grd, y_goal_grd)

            self.get_logger().info('start index: %d' % (self.start_index))
            self.get_logger().info('goal index: %d' % (self.goal_index))
            
            # calculate the shortes path using Dijkstra
            path = []
            path = self.dijkstra(self.start_index, self.goal_index, self.width, self.height, self.costmap, self.resolution, self.origin)

            if not path:
                self.get_logger().warn("No path returned by Dijkstra's shortes path algorithm")
            else:
                self.draw_path(path)    
        
    def dijkstra(self, start_index, goal_index, width, height, costmap, resolution, origin, grid_viz=None):
        
        # Initialize distances and visited array
        distances = [float('inf')] * (width * height)
        visited = [False] * (width * height)
        all = list(range(5477))
        # Set distance of start node to 0
        distances[start_index] = 0
        index = []
        # Loop until all nodes are visited
        while True:
            # Find the node with the minimum distance
            min_dist = float('inf')
            min_index = -1
            for i in range(width * height):
                if not visited[i] and distances[i] < min_dist:
                    min_dist = distances[i]
                    min_index = i

            # If the minimum distance node is the goal, break the loop
            if min_index == goal_index:
                break

            # Mark the node as visited
            visited[min_index] = True
            if visited[min_index] == True:
                index.append(min_index)
                
                self.draw_close(index)
                self.draw_open(all)

            # Find neighbors of the current node
            neighbors = self.find_neighbors(min_index, width, height, costmap, resolution)

            # Update distances of neighbors
            for neighbor in neighbors:
                neighbor_index = neighbor[0]
                step_cost = neighbor[1]
                if not visited[neighbor_index] and distances[min_index] + step_cost < distances[neighbor_index]:
                    distances[neighbor_index] = distances[min_index] + step_cost

        # Build the path by backtracking from the goal node to the start node
        path = [goal_index]
        current_index = goal_index
        while current_index != start_index:
            neighbors = self.find_neighbors(current_index, width, height, costmap, resolution)
            min_dist = float('inf')
            min_index = -1
            for neighbor in neighbors:
                neighbor_index = neighbor[0]
                step_cost = neighbor[1]
                if distances[neighbor_index] < min_dist:
                    min_dist = distances[neighbor_index]
                    min_index = neighbor_index
            path.append(min_index)
            current_index = min_index
        path.reverse()

        return path

    def draw_path(self, path):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.frame_id = 'map'

        for index in path:
            pose = PoseStamped()
            pose.pose.position.x = (index % self.width) * self.resolution + self.origin_x
            pose.pose.position.y = (index // self.width) * self.resolution + self.origin_y
            pose.pose.orientation.w = 10.0
            path_msg.poses.append(pose)

        # Publish the path
        self.path_publisher.publish(path_msg)
        
    def draw_close(self,close):
        arr_msg = MarkerArray()
        
        close_mk = Marker()
        close_mk.header.frame_id = "map"
        close_mk.id = 1
        close_mk.type = Marker.POINTS
        close_mk.action = Marker.ADD
        close_mk.pose.orientation.w = 1.0
        close_mk.scale.x = 0.1
        close_mk.scale.y = 0.1
        close_mk.color.r = 1.0
        close_mk.color.g = 0.5
        close_mk.color.a = 1.0
        
        for ixx in close:
            ppoint = Point()
            ppoint.x = (ixx % self.width) * self.resolution + self.origin_x
            ppoint.y = (ixx // self.width) * self.resolution + self.origin_y
            close_mk.points.append(ppoint)
            
        arr_msg.markers.append(close_mk)
                
        self.close_node_pub.publish(arr_msg)
    
    def draw_open(self, open):        
        arr_msg = MarkerArray()
        
        open_mk = Marker()
        open_mk.header.frame_id = "map"
        open_mk.id = 1
        open_mk.type = Marker.POINTS
        open_mk.action = Marker.ADD
        open_mk.pose.orientation.w = 1.0
        open_mk.scale.x = 0.1
        open_mk.scale.y = 0.1
        open_mk.color.r = 1.0
        open_mk.color.g = 1.0
        open_mk.color.a = 0.3
        
        for ix in open:
            point = Point()
            point.x = (ix % self.width) * self.resolution + self.origin_x
            point.y = (ix // self.width) * self.resolution + self.origin_y
            open_mk.points.append(point)
        
        arr_msg.markers.append(open_mk)
        
        self.open_node_pub.publish(arr_msg)

    def find_neighbors(self, index, width, height, costmap, orthogonal_step_cost):
        """
        Identifies neighbor nodes inspecting the 8 adjacent neighbors
        Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
        Returns a list with valid neighbour nodes as [index, step_cost] pairs
        """
        neighbors = []
        # length of diagonal = length of one side by the square root of 2 (1.41421)
        diagonal_step_cost = orthogonal_step_cost * 1.41421
        # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]
        lethal_cost = 1

        upper = index - width
        if upper > 0:
            if costmap[upper] < lethal_cost:
                step_cost = orthogonal_step_cost + costmap[upper]/255
                neighbors.append([upper, step_cost])

        left = index - 1
        if left % width > 0:
            if costmap[left] < lethal_cost:
                step_cost = orthogonal_step_cost + costmap[left]/255
                neighbors.append([left, step_cost])

        upper_left = index - width - 1
        if upper_left > 0 and upper_left % width > 0:
            if costmap[upper_left] < lethal_cost:
                step_cost = diagonal_step_cost + costmap[upper_left]/255
                neighbors.append([index - width - 1, step_cost])

        upper_right = index - width + 1
        if upper_right > 0 and (upper_right) % width != (width - 1):
            if costmap[upper_right] < lethal_cost:
                step_cost = diagonal_step_cost + costmap[upper_right]/255
                neighbors.append([upper_right, step_cost])

        right = index + 1
        if right % width != (width + 1):
            if costmap[right] < lethal_cost:
                step_cost = orthogonal_step_cost + costmap[right]/255
                neighbors.append([right, step_cost])

        lower_left = index + width - 1
        if lower_left < height * width and lower_left % width != 0:
            if costmap[lower_left] < lethal_cost:
                step_cost = diagonal_step_cost + costmap[lower_left]/255
                neighbors.append([lower_left, step_cost])

        lower = index + width
        if lower <= height * width:
            if costmap[lower] < lethal_cost:
                step_cost = orthogonal_step_cost + costmap[lower]/255
                neighbors.append([lower, step_cost])

        lower_right = index + width + 1
        if (lower_right) <= height * width and lower_right % width != (width - 1):
            if costmap[lower_right] < lethal_cost:
                step_cost = diagonal_step_cost + costmap[lower_right]/255
                neighbors.append([lower_right, step_cost])

        return neighbors

    def from_grid_to_index(self, x_grd, y_grd):
        return y_grd * self.width + x_grd

    def from_world_to_grid(self, x, y):
        x_grd = int((x - self.origin_x) / self.resolution)
        y_grd = int((y - self.origin_y) / self.resolution)
        return x_grd, y_grd
    

def main(args=None):
    rclpy.init(args=args)
    client = PathPlanner()
    client.send_request()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
