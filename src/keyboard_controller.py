#!/usr/bin/env python
"""
Chainatee Tanakulrungson
Feb 2016

SUBSCRIPTIONS:

PUBLISHERS:

SERVICES:

PARAMS:

"""

# ROS import:
import rospy
import tf
import kbhit
import tf.transformations as tr
import numpy as np
# from pykdl_utils.kdl_kinematics import KDLKinematics
# from urdf_parser_py.urdf import URDF
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import (
#     PoseStamped, Point, Quaternion
# )
# from std_srvs.srv import (
#     SetBool, SetBoolRequest, Empty, EmptyRequest
# )

# misc import:




# local imports:


###################
# global constant #
###################
# STEP_SIZE in metre
STEP_SIZE = 0.05
# STEP_DEG in deg
STEP_DEG = 5



class KeyboardController:
    def __init__(self):
        rospy.loginfo("Creating KeyboardController Class")

        # flag
        self.start_flag = False

        # create broadcaster and listener
        self.br = tf.TransformBroadcaster()
        self.ln = tf.TransformListener()
        
        # kbhit instance
        self.kb = kbhit.KBHit()
        self.print_help()

        # initialize start point of Pepper
        self.p = (0,0,0)
        self.q = tr.quaternion_from_euler(0, 0, 0, 'sxyz')

        # call 
        rospy.Timer(rospy.Duration(0.3), self.keyboard_cb)
        rospy.Timer(rospy.Duration(0.1), self.sendTF_cb)
        
        return

    def sendTF_cb(self, event):
        self.br.sendTransform(self.p, self.q, rospy.Time.now(), "base_link", "base")        
        return

    def keyboard_cb(self, event):
        # listen to present tf
        self.listen_tf()
        # check if any keys hit
        if self.kb.kbhit():
            k = self.kb.getch()
            if ord(k) == 27:
                rospy.signal_shutdown("Escape pressed!")
            if k == 'w':
                rospy.loginfo('pressed w : forward')
                self.move_forward()
            if k == 'a':
                rospy.loginfo('pressed a : turn counterclockwise ')
                self.rotate_left()
            if k == 's':
                rospy.loginfo('pressed s : backward')
                self.move_backward()
            if k == 'd':
                rospy.loginfo('pressed d : turn clockwise')
                self.rotate_right()
        return

    def listen_tf(self):
        if self.start_flag:
            now = rospy.Time.now()
            self.ln.waitForTransform('/base', '/base_link', now, rospy.Duration(4.0))
            (self.p, self.rot) = self.ln.lookupTransform('/base', '/base_link', now)
            self.rot = tr.quaternion_matrix(self.rot)
        else:
            self.p = np.zeros(3) 
            self.rot = np.eye(4)
            self.start_flag = not self.start_flag
        self.q = tr.quaternion_from_matrix(self.rot)
        return

    def move_backward(self):
        self.pres_euler = list(tr.euler_from_quaternion(self.q))
        self.old_pos = list(self.p)
        self.pres_rad = self.pres_euler[2]
        self.old_pos[0] -= STEP_SIZE*np.cos(self.pres_rad) 
        self.old_pos[1] -= STEP_SIZE*np.sin(self.pres_rad)
        self.p = tuple(self.old_pos)
        return

    def move_forward(self):
        self.pres_euler = list(tr.euler_from_quaternion(self.q))
        self.old_pos = list(self.p)
        self.pres_rad = self.pres_euler[2]
        self.old_pos[0] += STEP_SIZE*np.cos(self.pres_rad) 
        self.old_pos[1] += STEP_SIZE*np.sin(self.pres_rad)
        self.p = tuple(self.old_pos)
        return

    def rotate_left(self):
        self.old_euler = list(tr.euler_from_quaternion(self.q))
        self.old_euler[2] += np.deg2rad(STEP_DEG)
        self.q = tr.quaternion_from_euler(*self.old_euler)
        return

    def rotate_right(self):
        self.old_euler = list(tr.euler_from_quaternion(self.q))
        self.old_euler[2] -= np.deg2rad(STEP_DEG)
        self.q = tr.quaternion_from_euler(*self.old_euler)
        return


    def print_help(self):
        help_string = \
        """
        'w'   ~  Move forward
        's'   ~  Move backward
        'a'   ~  Rotate counterclockwise
        'd'   ~  Rotate clockwise
        'ESC' ~  Quit
        """
        print help_string
        return


def main():
    rospy.init_node('keyboard_controller')

    try:
        kb_cont = KeyboardController()
    except rospy.ROSInterruptException: pass


    rospy.spin()



if __name__=='__main__':
    main()
