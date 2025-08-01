#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, TwistStamped
from novatel_oem7_msgs.msg import INSPVA
from pyproj import CRS, Proj, Transformer
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler


class Localizer:
    def __init__(self):

        # Parameters
        self.undulation = rospy.get_param("undulation")
        utm_origin_lat = rospy.get_param("utm_origin_lat")
        utm_origin_lon = rospy.get_param("utm_origin_lon")

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)

        # create a coordinate transformer
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
        self.origin_x, self.origin_y = self.transformer.transform(
            utm_origin_lat, utm_origin_lon
        )

        # Subscribers
        rospy.Subscriber("/novatel/oem7/inspva", INSPVA, self.transform_coordinates)

        # Publishers
        self.current_pose_pub = rospy.Publisher(
            "current_pose", PoseStamped, queue_size=10
        )
        self.current_velocity_pub = rospy.Publisher(
            "current_velocity", TwistStamped, queue_size=10
        )
        self.br = TransformBroadcaster()

    def transform_coordinates(self, msg):

        x, y = self.transformer.transform(msg.latitude, msg.longitude)

        x = x - self.origin_x
        y = y - self.origin_y
        z = msg.height - self.undulation

        # print(x, y)

        # calculate azimuth correction
        azimuth_correction = self.utm_projection.get_factors(
            msg.longitude, msg.latitude
        ).meridian_convergence

        # convert azimuth to yaw angle
        def convert_azimuth_to_yaw(azimuth):
            """
            Converts azimuth to yaw. Azimuth is CW angle from the north. Yaw is CCW angle from the East.
            :param azimuth: azimuth in radians
            :return: yaw in radians
            """
            yaw = -azimuth + math.pi / 2
            # Clamp within 0 to 2 pi
            if yaw > 2 * math.pi:
                yaw = yaw - 2 * math.pi
            elif yaw < 0:
                yaw += 2 * math.pi

            return yaw

        corrected_azimuth_in_rad = math.radians(msg.azimuth - azimuth_correction)
        yaw = convert_azimuth_to_yaw(corrected_azimuth_in_rad)
        # Convert yaw to quaternion
        _x, _y, _z, w = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion(_x, _y, _z, w)

        # publish current pose
        current_pose_msg = PoseStamped()
        current_pose_msg.header.stamp = msg.header.stamp
        current_pose_msg.header.frame_id = "map"
        current_pose_msg.pose.position.x = x
        current_pose_msg.pose.position.y = y
        current_pose_msg.pose.position.z = z
        current_pose_msg.pose.orientation = orientation

        self.current_pose_pub.publish(current_pose_msg)

        nv, ev = msg.north_velocity, msg.east_velocity
        norm = math.sqrt(nv**2 + ev**2)
        twst_msg = TwistStamped()
        twst_msg.header.stamp = msg.header.stamp
        twst_msg.header.frame_id = "base_link"
        twst_msg.twist.linear.x = norm
        self.current_velocity_pub.publish(twst_msg)

        trnsfrm_msg = TransformStamped()

        trnsfrm_msg.header.stamp = msg.header.stamp
        trnsfrm_msg.child_frame_id = "base_link"
        trnsfrm_msg.header.frame_id = "map"
        trnsfrm_msg.transform.translation.x = x
        trnsfrm_msg.transform.translation.y = y
        trnsfrm_msg.transform.translation.z = z
        trnsfrm_msg.transform.rotation = orientation

        self.br.sendTransform(trnsfrm_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("localizer")
    node = Localizer()
    node.run()