#!/usr/bin/env python3


# All these imports from lanelet2 library should be sufficient
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
from autoware_mini.lanelet2 import load_lanelet2_map

from autoware_mini.msg import Waypoint, Path
from shapely.geometry import Point
import shapely
import rospy

from geometry_msgs.msg import PoseStamped

from shapely import distance


class GloablPlanner:
    def __init__(self):
    
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.distance_to_goal_limit = rospy.get_param("distance_to_goal_limit", 4.0)
        self.current_pose = None
        self.goal_pose = None

        self.speed_limit = rospy.get_param("~speed_limit", 40.0) / 3.6 # division converts from km/h to m/s
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        
        # Subscribers
        rospy.Subscriber("/localization/current_pose", PoseStamped, self.current_pose_sub)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_point_sub)

        

        # Publisher
        self.global_path_pub = rospy.Publisher(
            "global_path", Path, queue_size=10,
            latch=True
        )


    def current_pose_sub(self, msg):
        current_pose = msg.pose
        self.current_pose = current_pose

        current_point = shapely.Point(current_pose.position.x, current_pose.position.y, current_pose.position.z)

        if self.goal_pose:
            goal_point = shapely.Point(self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z,)

            d = shapely.distance(current_point, goal_point)
            if d < self.distance_to_goal_limit:
                self.goal_pose = None                
                self.publish_waypoints([])
                rospy.loginfo("%s - goal reached, clearing path!", rospy.get_name())

    def publish_waypoints(self, waypoints):
        path = Path()
        path.header.frame_id = self.output_frame
        path.header.stamp = rospy.Time.now()
        path.waypoints = waypoints
        
        self.global_path_pub.publish(path)

    def goal_point_sub(self, msg):
        # loginfo message about receiving the goal point
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w, msg.header.frame_id)
        
        self.goal_pose = msg.pose


                # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.VehicleTaxi)
        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        self.current_location = BasicPoint2d(self.current_pose.position.x, self.current_pose.position.y)

        try:
            self.goal_point = BasicPoint2d(self.goal_pose.position.x, self.goal_pose.position.y)


            # get start and end lanelets
            start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
            goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]
            # find routing graph
            route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)
            
            if not route:
                rospy.logwarn("%s - no route has been found for goal position (%f, %f, %f) in %s frame",
                              rospy.get_name(),
                              self.goal_pose.position.x,
                              self.goal_pose.position.y,
                              self.goal_pose.position.z,
                              msg.header.frame_id
                              )
                return

            # find shortest path
            path = route.shortestPath()
            # This returns LaneletSequence to a point where a lane change would be necessary to continue
            path_no_lane_change = path.getRemainingLane(start_lanelet)
            waypoints = self.convert_lanelets_to_path(path_no_lane_change)
            
            self.publish_waypoints(waypoints)
        except AttributeError as e:
            print("we have attribute error:", e)
            



    def convert_lanelets_to_path(self, lanelet_sequence):
        waypoints = []
        waypoint_from_last_seq = Waypoint()

        goal_point = Point(self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z)
        def has_goal_reached(x, y, z):
            current_point = Point(x, y, z)
            d = distance(current_point, goal_point)
            if d < 1:
                return True
            return False

        for lanelet in lanelet_sequence:
            # code to check if lanelet has attribute speed_ref
            if 'speed_ref' in lanelet.attributes:
                speed = float(lanelet.attributes['speed_ref'])
                speed /= 3.6 # convert from km/h to m/s
            else:
                speed = self.speed_limit
            
            for point in lanelet.centerline:
                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.speed = speed
                if waypoint != waypoint_from_last_seq: # avoid overlaps
                    waypoints.append(waypoint)
                if has_goal_reached(point.x, point.y, point.z):
                    return waypoints
            waypoint_from_last_seq = waypoint

        goal_waypoint = Waypoint()
        goal_waypoint.position.x = self.goal_pose.position.x
        goal_waypoint.position.y = self.goal_pose.position.y
        goal_waypoint.position.z = self.goal_pose.position.z
        goal_waypoint.speed = self.speed_limit

        waypoint_A, waypoint_B = waypoints[-2:]
        def is_goal_in_between(a, b, g):
            if (a.position.x <= g.position.x <= b.position.x
                or
                a.position.x >= g.position.x >= b.position.x):
                if (a.position.y <= g.position.y <= b.position.y
                    or
                    a.position.y >= g.position.y >= b.position.y):
                    return True
            return False


        if is_goal_in_between(waypoint_A, waypoint_B, goal_waypoint):
            waypoints[-1] = goal_waypoint
        else:
            waypoints.append(goal_waypoint)

        return waypoints


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("global_planner")
    node = GloablPlanner()
    node.run()