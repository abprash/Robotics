#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Robot():

    def __init__(self, name, listen):
        self._name = name
        self._listen = listen
        self._cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('{0}_controller'.format(self._name), anonymous=True)
        rospy.Subscriber('odom', Odometry, self._record_and_transform_odom)
        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._rate = rospy.Rate(10)

    def run(self):
        rospy.sleep(2)
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                past = now - rospy.Duration(1.0)
                self._tf_listener.waitForTransformFull(self._name, now, self._listen, past, "world", rospy.Duration(1.0))
                (translation, rotation) = self._tf_listener.lookupTransformFull(self._name, now, self._listen, past, "world")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                continue

            angular = 4 * math.atan2(translation[1], translation[0])
            linear = 0.5 * math.sqrt(translation[0] ** 2 + translation[1] ** 2)
            self._cmd_vel.publish(self._make_twist([linear, 0, 0], [0, 0, angular]))
            self._rate.sleep()

    def _make_twist(self, linear=[0, 0, 0], angular=[0, 0, 0]):
        t = Twist()
        t.linear.x = linear[0]
        t.linear.y = linear[1]
        t.linear.z = linear[2]

        t.angular.x = angular[0]
        t.angular.y = angular[1]
        t.angular.z = angular[2]

        return t

    def _record_and_transform_odom(self, data):
        self._odom = data
        translation = data.pose.pose.position
        rotation = data.pose.pose.orientation
        self._tf_broadcaster.sendTransform( (translation.x, translation.y, translation.z)
                                          , (rotation.x, rotation.y, rotation.z, rotation.w)
                                          , rospy.Time.now()
                                          , self._name
                                          , "world")

def main():
    r = Robot("pursuer", "evader")
    r.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
