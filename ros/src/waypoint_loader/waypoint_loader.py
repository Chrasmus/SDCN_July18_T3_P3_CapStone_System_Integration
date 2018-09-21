#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion

from styx_msgs.msg import Lane, Waypoint

import tf
import rospy

CSV_HEADER = ['x', 'y', 'z', 'yaw']
MAX_DECEL = 1.0


class WaypointLoader(object):

    def __init__(self):
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        self.pub = rospy.Publisher('/base_waypoints', Lane, queue_size=1, latch=True)

        self.velocity = self.kmph2mps(rospy.get_param('~velocity'))
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin()

    def new_waypoint_loader(self, path):
        if os.path.isfile(path):
            waypoints = self.load_waypoints(path)
            rospy.loginfo('Original Waypoints len = %s', str(len(waypoints)))
            waypoints = self.add_missing_waypoints(waypoints)
            rospy.loginfo('Original plus added missing Waypoints len = %s', str(len(waypoints)))
            self.publish(waypoints)
            rospy.loginfo('Waypoints Loaded')
        else:
            rospy.logerr('%s is not a file', path)
    
    def add_missing_waypoints(self, waypoints):
        # There are some waypoint that are spaced to far from each other,
        # which causes problems in the simulator.
        # solution: add one or two waypoints in between the longer distances
        new_waypoints = []
        for i in range(len(waypoints)-1):
            first = waypoints[i]
            second = waypoints[i+1]
            first_x = first.pose.pose.position.x
            first_y = first.pose.pose.position.y
            second_x = second.pose.pose.position.x
            second_y = second.pose.pose.position.y
            dist  = self.distance(first.pose.pose.position, second.pose.pose.position)
            if dist < 1.0:
                new_waypoints.append(first)
            elif (dist > 1.0 and dist < 1.5):
                new_waypoints.append(first)
                p = first
                p.pose.pose.position.x = first_x + (second_x - first_x)/2
                p.pose.pose.position.y = first_y + (second_y - first_y)/2
                new_waypoints.append(p)
            elif dist > 1.5:
                new_waypoints.append(first)
                p = first
                p.pose.pose.position.x = first_x + (second_x - first_x)/3
                p.pose.pose.position.y = first_y + (second_y - first_y)/3
                new_waypoints.append(p)
                p = first
                p.pose.pose.position.x = first_x + (second_x - first_x)*2/3
                p.pose.pose.position.y = first_y + (second_y - first_y)*2/3
                new_waypoints.append(p)
                
        last = waypoints[-1]
        new_waypoints.append(last)
        
        return new_waypoints

    def quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0., 0., yaw)

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def load_waypoints(self, fname):
        waypoints = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                p = Waypoint()
                p.pose.pose.position.x = float(wp['x'])
                p.pose.pose.position.y = float(wp['y'])
                p.pose.pose.position.z = float(wp['z'])
                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                p.twist.twist.linear.x = float(self.velocity)

                waypoints.append(p)
        return self.decelerate(waypoints)

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
