#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # rospy.Subscriber('/traffic_waypoint', PoseStamped, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.current_pose = None
        self.base_waypoints = None
        self.traffic_waypoint = None
        self.obstable_waypoint = None
        self.final_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        self.send_waypoints()

    def waypoints_cb(self, lane):
        if self.base_waypoints is None:
            self.base_waypoints = lane.waypoints
            self.send_waypoints()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstable_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.dist(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def send_waypoints(self):
        if self.base_waypoints is None or self.current_pose is None:
            return

        next_index = self.get_next_waypoint(self.current_pose)

        self.final_waypoints = self.base_waypoints[next_index:(next_index + LOOKAHEAD_WPS)]
        self.publish_final_waypoints()

    def publish_final_waypoints(self):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = self.final_waypoints

        self.final_waypoints_pub.publish(lane)

    # Path planning project
    # https://github.com/hecspc/CarND-Path-Planning-Project/blob/master/src/main.cpp

    def dist(self, p1, p2):
        return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2 + (p2.z - p1.z) ** 2)

    def get_closest_waypoint(self, pose):

        closest_len = 100000
        closest_index = 0

        waypoints = self.base_waypoints

        for i in range(len(waypoints)):
            waypoint = waypoints[i].pose.pose.position
            d = self.dist(pose.position, waypoint)
            if d < closest_len:
                closest_len = d
                closest_index = i

        return closest_index

    def get_next_waypoint(self, pose):

        next_index = self.get_closest_waypoint(pose)
        p1 = pose.position
        p2 = self.base_waypoints[next_index].pose.pose.position

        heading = math.atan2((p2.y - p1.y), (p2.x - p1.x))

        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)

        # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        euler_orientation = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler_orientation[0]
        # pitch = euler_orientation[1]
        yaw = euler_orientation[2]

        angle = abs(yaw - heading)

        if angle > math.pi / 4:
            next_index += 1

        return next_index


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
