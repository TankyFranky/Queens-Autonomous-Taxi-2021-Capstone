#!/usr/bin/env python

import rospy
from driver import UsDigitalSei
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64
# Speedometer(ticks_per_metre, drive_ticks) where drive_ticks is 'left', 'right', or 'both'.
# left: USDigitialEncoders.ticks[0]
# right: USDigitialEncoders.ticks[1]
# both: USDigitialEncoders.ticks[0] and USDigitalEncoders[1] (take average of the two)

class Steering_angle(object):
    def __init__(self, ticks_per_metre, drive_ticks='both', window=None):
        rospy.init_node('steering_angle')
        self.encoder = UsDigitalSei()
	    self.last_ticks = 35000
        self.sub = rospy.Subscriber('scanmatch_odom', Odometry, self._changeOdom)
        #self.pub_angle = rospy.Publisher('steer/angle', Float64, queue_size=5)
        self.pub_new_odom = rospy.Publisher('changed_odom', Odometry, queue_size=5)
        self.steering_state = rospy.Publisher('steering/state', Float64, queue_size =5)
        self.drive_state = rospy.Publisher('drive/state', Float64, queue_size =5)
        self.steer_msg = Float64()
        self.drive_msg = Float64()

    def _changeOdom(self, msg):
        current_angle = self._ticks_to_angle()
        current_velocity = msg.twist.twist.linear.x

        msg.twist.twist.angular.z = current_angle
        self.pub_new_odom.publish(msg)

        self.steer_msg.data = current_angle
        self.drive_msg.data = current_velocity
        self.steering_state.publish(self.steer_msg)
        self.drive_state.publish(self.drive_msg)
        
    def _ticks_to_angle(self):
        try:
            self.last_ticks = int(self.encoder.sei_read('1010'))
            rads = (0.102*self.last_ticks)
	    tads = (rads/1000) -3.483998
        except:
            rads = (0.102*self.last_ticks)
	    tads = (rads/1000) -3.483998
        return -tads


if __name__ == '__main__':
    steering_angle = Steering_angle(1, drive_ticks='right')
    while(True):
        continue

