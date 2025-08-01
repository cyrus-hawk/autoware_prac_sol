#!/usr/bin/env python3

import rospy

from autoware_mini.msg import Path
from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import VehicleCmd

from shapely.geometry import LineString, Point
from shapely import prepare, distance
import numpy as np

import math
from scipy.interpolate import interp1d

from tf.transformations import euler_from_quaternion

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.path_linstring = None
        # Reading in the parameter values
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")
        self.distance_to_velocity_interpolator = None

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher(
            "/control/vehicle_cmd", VehicleCmd, queue_size=10
        )

        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        try:
            # convert waypoints to shapely linestring
            path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
            # prepare path - creates spatial tree, making the spatial queries more efficient
            prepare(path_linestring)
            self.path_linstring = path_linestring

            # Create a distance-to-velocity interpolator for the path
            # collect waypoint x and y coordinates
            waypoints_xy = np.array([(w.position.x, w.position.y) for w in msg.waypoints])
            # Calculate distances between points
            distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0)**2, axis=1)))
            # add 0 distance in the beginning
            distances = np.insert(distances, 0, 0)
            # Extract velocity values at waypoints
            velocities = np.array([w.speed for w in msg.waypoints])     
            self.distance_to_velocity_interpolator = interp1d(distances, velocities, kind='linear', bounds_error=False, fill_value=0.0)           
        except np.AxisError:
            self.distance_to_velocity_interpolator = interp1d([0, 0], [0, 0], kind='linear')   

    def current_pose_callback(self, msg):
        velocity = steering_angle = 0
        
        try:
            current_pose = Point([msg.pose.position.x, msg.pose.position.y])
            d_ego_from_path_start = self.path_linstring.project(current_pose)
       

        

            # current heading angle
            _, _, current_heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

            # lookahead_circle = current_pose.buffer(self.lookahead_distance)
            

      
            # intersection = self.path_linstring.intersection(lookahead_circle)

            # coords = list(intersection.coords)
            # lookahead_point = Point(coords[-1])
            lookahead_point = self.path_linstring.interpolate(d_ego_from_path_start + self.lookahead_distance)
            
        
            # lookahead point heading calculation
            lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)

            lookahead_distance = distance(lookahead_point, current_pose)

            heading_difference = lookahead_heading - current_heading

            curvature = 2 * math.sin(heading_difference) / lookahead_distance

            steering_angle = math.atan(self.wheel_base * curvature)

            velocity = self.distance_to_velocity_interpolator(d_ego_from_path_start)   

        except (IndexError, AttributeError):
            steering_angle = 0
        except (TypeError, ValueError):
            velocity = 0

        # print("lookahead_distance", lookahead_distance)
        # print("steering angle", steering_angle)
        # print("velocity", velocity)
        # print("d_ego_from_path_start", d_ego_from_path_start)

        vehicle_cmd = VehicleCmd()
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.ctrl_cmd.linear_velocity = velocity
        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"
        self.vehicle_cmd_pub.publish(vehicle_cmd)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()