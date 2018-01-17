#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Written by Lam Ngo

class create_data:

  def __init__(self):
    self.image_pub = rospy.Publisher("create_data",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    #save the database of the current person
    name = raw_input("what is your name?")
    self.path = "/home/generic/picture/" + name
    if not os.path.isdir(self.path):
        os.mkdir(self.path)
    #fixed image_size for better performance.
    self.width = 130
    self.height = 100

  #call back to create datasets.
  def callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    face_cascade = cv2.CascadeClassifier('/home/generic/catkin_ws/src/image_test/src/haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(gray,1.3,4)
    count = 0
    while count < 51:
        for (x,y,w,h) in faces:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            face = gray[y:y + h, x:x + w]
            face_resize = cv2.resize(face,(self.width,self.height))
            cv2.imwrite('%s/%s.png' % (self.path,count), face_resize)
        count = count + 1

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        print(e)

def main(args):
    ic = create_data()
    rospy.init_node('createdataset', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
