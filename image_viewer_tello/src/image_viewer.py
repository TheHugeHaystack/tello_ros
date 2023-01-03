#!/usr/bin/env python3

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/tello/image_raw",
                                            Image,
                                            self.callback,
                                            queue_size=1,
                                            buff_size=2**24)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
   
        (rows,cols,channels) = cv_image.shape
        print(f'rows: {rows}, cols: {cols}, channels: {channels}')
   
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
   
def main(args):
    ic = Image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
