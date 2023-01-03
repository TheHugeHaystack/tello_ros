#!/usr/bin/env python3

import rospy
import sys
from getkey import getkey, keys
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Empty


class Keyboard:
    def __init__(self) -> None:
        self.pub_takeoff =  rospy.Publisher('/tello/takeoff', Empty, queue_size=10)
        self.pub_land =  rospy.Publisher('/tello/land', Empty, queue_size=10)
        self.pub_cmd_vel =  rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.pub_override =  rospy.Publisher('/keyboard/override', Int8, queue_size=10)
        self.vel_msg = Twist()
        self.ovr_msg = 0
        self.speed_val = 0.3
        self.init_msg= """
        #################################################################################
        COMMAND   |  KEY
        -------------------------------------------------------
        LAND        -SPACE BAR
        TAKEOFF     -T
        FORWARD     -W
        BACKWARD    -S
        RIGHT       -D
        LEFT        -A
        ROTATE_L    -J
        ROTATE_R    -L
        UP          -I
        DOWN        -K

        AUTONOMOUS  -X
        MANUAL      -C
        #################################################################################
        """
        self.final_msg = """
        CNTRL + E TO QUIT
        """
        self.cmd_vel()

    def cmd_vel(self):
        while not rospy.is_shutdown():
            key = getkey()
            if key == keys.T:
                msg = Empty()
                self.pub_takeoff.publish(msg)
                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: TAKEOFF")
                print(self.final_msg)

            
            elif key == keys.SPACE:
                msg = Empty()
                self.pub_land.publish(msg)
                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: LAND")
                print(self.final_msg)
            
            elif key == keys.W:
                self.vel_msg.linear.x = round(self.speed_val,2)
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)

                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: FORWARD")
                print(self.final_msg)

            elif key == keys.S:
                self.vel_msg.linear.x = -round(self.speed_val,2)
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)

                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: BACKWARD")
                print(self.final_msg)

            elif key == keys.A:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = round(self.speed_val,2)
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)

                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: LEFT")
                print(self.final_msg)

            elif key == keys.D:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = -round(self.speed_val,2)
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)

                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: RIGHT")
                print(self.final_msg)
            
            elif key == keys.J:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = round(self.speed_val,2)
                self.pub_cmd_vel.publish(self.vel_msg)

                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: ROTATE_L")
                print(self.final_msg)

            elif key == keys.L:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = -round(self.speed_val,2)
                self.pub_cmd_vel.publish(self.vel_msg)

                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: ROTATE_R")
                print(self.final_msg)
            
            elif key == keys.I:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = round(self.speed_val,2)
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)

                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: UP")
                print(self.final_msg)

            elif key == keys.K:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = -round(self.speed_val,2)
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)

                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: DOWN")
                print(self.final_msg)


            elif key == keys.X:
                self.ovr_msg = 5
                self.pub_override(self.ovr_msg)
                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: AUTOMATIC")
                print(self.final_msg)

            elif key == keys.X:
                self.ovr_msg = 10
                self.pub_override(self.ovr_msg)
                print(self.init_msg)
                print(f"            SPEED DRONE: {self.speed_val, 2}")
                print(f"            LAST COMMAND SENT: MANUAL")
                print(self.final_msg)

            

def main():
    rospy.init_node('keyboard', anonymous=True)
    keyboard = Keyboard()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    main()
