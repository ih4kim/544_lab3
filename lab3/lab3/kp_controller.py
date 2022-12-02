from .a_star_skeleton1 import astar
from interfaces.srv import StartAndEnd
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
import rclpy
import numpy as np

from rclpy.node import Node
import math
import numpy as np
import cv2
import json

DEBUG = True
OFFLINE_MAP = True

class Pose():
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        # save unit vector as heading
        self.heading = [math.cos(theta), math.sin(theta)]

    def set_theta(self, theta):
        deg = 38.48
        self.theta = theta+((180-deg)*math.pi/180)
        self.heading = [math.cos(self.theta), math.sin(self.theta)]

    def __str__(self) -> str:
        return "x: %.2f y: %.2f theta: %.2f" % (self.x, self.y, self.theta*180/math.pi)

class KpController(Node):
    def __init__(self):
        super().__init__('kp_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(TFMessage, '/tf', self.update_pose, 10)
        self.timer = self.create_timer(0.01, self.kp_controller)
        self.srv = self.create_service(StartAndEnd, 'start_and_end', self.start_controller)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.create_costmap, 10)
        self.created_map = False
        self.tf_to_odom_x = None
        self.tf_to_odom_y = None
        self.curr_pose = Pose()
        # Node has pixel coordinates as positions
        # (end point) -> (2nd last point) -> ... -> (start point)
        self.path_nodes = []

        # Resolution of the map (currently arbituary)
        self.resolution = 0.05
        # Map width and height
        self.map_width = 310
        self.map_height = 195
        # Bottom left coordinate wrt to odom frame
        self.origin_x = -5.75
        self.origin_y = -3.72
        self.maze = None
        self.created_map = False

        if OFFLINE_MAP:
            f = open('scaledmap.json')
            data = json.load(f)
            self.resolution = data["resolution"]

            # Map width and height
            self.map_width = data["width"]
            self.map_height = data["height"]

            # Bottom left coordinate wrt to odom frame
            self.origin_x = data["origin_x"]
            self.origin_y = data["origin_y"]
            self.maze = cv2.imread("ScaledMap.png")
            self.maze = cv2.cvtColor(self.maze, cv2.COLOR_BGR2GRAY)
            print(self.maze.shape)
            self.created_map = True

        # Error threshold
        self.error_threshold = 0.05
        self.ogm_threshold = 75
        # Constant linear vel
        self.lin_vel = 0.05
        #Pose Offset
        self.tf_to_odom_x = None#-2.0
        self.tf_to_odom_y = None#-0.5
        # Kp controller constant
        self.kp = 1
        

    def start_controller(self, req, resp):
        print(req.start.x)
        print(req.end.x)
        
        start_x_px, start_y_px = self.cartesian_to_pixel(req.start.x, req.start.y)
        end_x_px, end_y_px = self.cartesian_to_pixel(req.end.x, req.end.y)
        if (self.check_valid_pixel(start_x_px, start_y_px) and self.check_valid_pixel(end_x_px, end_y_px)):
            # Check if start and end points are valid positions
            start_point = (start_x_px, start_y_px)
            end_point = (end_x_px, end_y_px)
            path = astar(self.maze, start_point, end_point, self.ogm_threshold)
            if DEBUG:
                self.draw_path(path)
            path.pop()
            path.pop()
            self.path_nodes = path
            print(path)
            resp.status = True
        else:
            resp.status = False
        return resp
    
    def draw_path(self, path):
        test_image = cv2.cvtColor(self.maze, cv2.COLOR_GRAY2BGR)
        path_len = len(path)
        for i in range(1,path_len):
            end_point = path[i]
            start_point = path[i-1]
            test_image = cv2.line(test_image, end_point, start_point, color=(0,0,255))
        # Draw start and end
        #End
        test_image = cv2.circle(test_image, path[0], 1, color=(255,0,0))
        #Start
        test_image = cv2.circle(test_image, path[-1], 1, color=(0,255,0))
        # Save iamge
        cv2.imwrite("path_draw.png", test_image)
            

    def kp_controller(self):
        ang_vel = float(0)
        if len(self.path_nodes)>0:
            # Get last node
            target_node = self.path_nodes[-1]
            target_px_x, target_px_y = target_node
            target_odom_x, target_odom_y = self.pixel_to_cartesian(target_px_x, target_px_y)
            print(target_odom_x, target_odom_y)
            # Check if tolerance is met
            # print(self.euclidean_distance(target_odom_x, target_odom_y, self.curr_pose.x, self.curr_pose.y))
            if (self.euclidean_distance(target_odom_x, target_odom_y, self.curr_pose.x, self.curr_pose.y) < self.error_threshold):
                #print("HERE1")
                # Remove the node, and continue for the next node on next iteration
                self.path_nodes.pop()
            else:
                #print("HERE")
                # does not meet tolerance, calculate error in heading
                desired_heading = self.get_desired_heading(target_odom_x, target_odom_y)
                print("Desired heading", desired_heading)
                print("Current heading", self.curr_pose.heading)
                theta_error = math.acos(np.dot(desired_heading, self.curr_pose.heading))
                # print(theta_error)
                # determine direction of error
                direction = 1 if np.cross(self.curr_pose.heading, desired_heading)>0 else -1
                if abs(theta_error) > 0.01:
                    ang_vel = float(theta_error*direction*self.kp)
                    print(ang_vel)
                print("Target Theta: %.2f" % (theta_error*direction))
                msg = Twist()
                msg.linear.x = self.lin_vel
                msg.angular.z = ang_vel
                self.publisher.publish(msg)
                
        else:
            # no nodes, stop sending signal
            msg = Twist()
            self.publisher.publish(msg)
    
    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt(pow(x1-x2,2)+pow(y1-y2,2))

    def pixel_to_cartesian(self, px_x, px_y):
        odom_x = (px_x*self.resolution)+self.origin_x
        odom_y = (self.map_height*self.resolution+self.origin_y)-(px_y*self.resolution)
        return odom_x, odom_y

    def get_desired_heading(self, target_x, target_y):
        delta_x = target_x-self.curr_pose.x
        delta_y = target_y-self.curr_pose.y
        desired_heading = math.atan2(delta_y, delta_x)
        return [math.cos(desired_heading), math.sin(desired_heading)]

    def update_pose(self, msg: TFMessage):
        for tf in msg.transforms:
            # Find the base footprint TF
            if tf.child_frame_id == "base_footprint":
                if not self.tf_to_odom_x:
                    # Update offset to tranform into odom frame
                    self.tf_to_odom_x = tf.transform.translation.x
                    self.tf_to_odom_y = tf.transform.translation.y
                # Update position
                tempx = tf.transform.translation.x - self.tf_to_odom_x
                tempy = tf.transform.translation.y - self.tf_to_odom_y
                deg = 38.48+180
                offset = 0.15
                self.curr_pose.x = (tempx * math.cos(deg*math.pi/180) + tempy * math.sin(deg*math.pi/180))-offset

                self.curr_pose.y = tempx * -math.sin(deg*math.pi/180) + tempy * math.cos(deg*math.pi/180)

                # Update heading 
                q_x = tf.transform.rotation.x
                q_y = tf.transform.rotation.y
                q_z = tf.transform.rotation.z
                q_w = tf.transform.rotation.w

                siny_cosp = 2 * (q_w * q_z + q_x * q_y)
                cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
                # curr_theta ranges from -pi to pi
                self.curr_pose.set_theta(math.atan2(siny_cosp, cosy_cosp))
            
                # Output current pose
                self.get_logger().info("Current Position: " + str(self.curr_pose))
                break

    def check_valid_pixel(self, px_x, px_y):
        # Check if within bounds
        self.get_logger().info("Target: x: %d y: %d" %(px_x, px_y))
        if px_x < 0 or px_x > self.map_width-1:
            self.get_logger().warn("x out of bounds!")
            return False
        if px_y < 0 or px_y > self.map_height-1:
            self.get_logger().warn("y out of bounds!")
            return False
        print(self.maze[px_y][px_x])
        if self.maze[px_y][px_x] > self.ogm_threshold:
            self.get_logger().warn("Point occupied!")
            return False
        return True

    def cartesian_to_pixel(self, odom_x, odom_y):
        px_x = int((odom_x - self.origin_x) / self.resolution)
        px_y = int((odom_y - (self.map_height * self.resolution + self.origin_y)) / -self.resolution)
        return px_x, px_y

    def create_costmap(self, msg: OccupancyGrid):
        if (not self.created_map):
            # Save data from costmap topic
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin = msg.info.origin
            data = np.array(msg.data)

            # Reshape array according to width and height
            reshaped_data = np.reshape(data, (height, width)).astype('float32')

            # Determine new image dimentions for downscaling
            resize_factor = 0.2 / resolution
            new_width = int(width / resize_factor)
            new_height = int(height / resize_factor)

            # Rescale and flip the data
            rescaled_data = cv2.resize(reshaped_data, (new_width, new_height))
            flipped_data = cv2.flip(rescaled_data, 0)

            cv2.imwrite("ScaledMap.png", flipped_data)

            self.map_width = new_width
            self.map_height = new_height
            self.resolution = 0.2
            self.origin_x = origin.position.x
            self.origin_y = origin.position.y
            self.maze = flipped_data
            self.created_map = True

            dict = {}
            dict["width"] = new_width
            dict["height"] = new_height
            dict["resolution"] = 0.2
            dict["origin_x"] = origin.position.x
            dict["origin_y"] = origin.position.y

            with open("scaledmap.json", "w") as f:
                json.dump(dict, f)

        
        
def main():
    rclpy.init()

    controller = KpController()

    rclpy.spin(controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()