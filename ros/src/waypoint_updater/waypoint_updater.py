#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number. Originally 200
MAX_DECEL = 0.5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=100) #100)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=10)
        #rospy.Subscriber('/obstacle_waypoint', ??, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)


        self.loop()
        #rospy.spin()

    def loop(self):
        # Gives us control over the publish frequency, which is 50 Hz
        #rate = rospy.Rate(50)
        rate = rospy.Rate(5)
        #rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #if self.pose and self.base_lane:
            #print('self.pose = ', str(self.pose), ' -- self.base_lane = ', str(self.base_lane), ' -- self.waypoint_tree = ', str(self.waypoint_tree))
            if self.pose and self.base_lane and self.waypoint_tree:
                ## Get closest waypoint
                ##closest_waypoint_idx = self.get_closest_waypoint_idx()
                ##self.publish_waypoints(closest_waypoint_idx)
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        #check if closest is ahead or behind vehicle using numpy.dot()
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        # We're only interested in waypoints ahead of the vehicle
        if val > 0:
            print('wp_upd.get_closestwaypoint_idx -- closest_idx = ', closest_idx, ' -- len(self.way.points_2d) = ', len(self.waypoints_2d))
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
            print('closest_idx = (closest_idx + 1) % len(self.waypoints_2d) = ', closest_idx)
        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        # Create new Lane message type object
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        # important to remember to slice base_lane before decelerating so no uncessecary latency is introduced
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        #rospy.loginfo('lane.waypoints len = %s', str(len(lane.waypoints)))
        #rospy.loginfo("first lane waypoint ({0}, {1})".format(lane.waypoints[1].pose.pose.position.x, lane.waypoints[1].pose.pose.position.y))
        return lane

        ##lane.header = self.base_waypoints.header
        ## slice the right amount of waypoints and publish them
        ##lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        ##self.final_waypoints_pub.publish(lane)

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 3, 0) #three waypoints back from line so front of car stops at line
            dist = self.distance(waypoints, i, stop_idx)
            # using sqrt gives a steep deceleration, try using a linear funktion instead, e.g. multiply with a constant
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.0:
                vel = 0.0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        rospy.loginfo("WP_U.py recieved pose (x,y) : ({0}, {1})".format(x,y))

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0.0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
