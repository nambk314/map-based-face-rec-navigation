#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import rospy
from std_msgs.msg import String
import cv2, sys, numpy, os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Written by Lam Ngo
#Adapted from https://github.com/aquibjaved/Real-time-face-recognition-in-python-using-opencv-

class face_regconition:

    def __init__(self):
	#publish string so other nodes can subctibe to it
        self.image_pub = rospy.Publisher("face_recognition",String,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        #self.path = os.path.join('/home/generic/catkin_ws/src/image_test/src/datasets', '/home/generic/catkin_ws/src/image_test/src/lam')
        self.datasets = "/home/turtlebot/picture/"
        self.width = 130
        self.height = 100
        (self.images, self.lables, self.names, self.id) = ([], [], {}, 0)
        for (subdirs, dirs, files) in os.walk(self.datasets):
            for subdir in dirs:
                self.names[self.id] = subdir
                subjectpath = os.path.join(self.datasets, subdir)
                for filename in os.listdir(subjectpath):
                    path = subjectpath + '/' + filename
                    lable = self.id
                    self.images.append(cv2.imread(path, 0))
                    self.lables.append(int(lable))
                self.id += 1
        (self.images, self.lables) = [numpy.array(lis) for lis in [self.images, self.lables]]
        self.model = cv2.createFisherFaceRecognizer()
        self.model.train(self.images, self.lables)
        self.faces = None
        self.gray = None
        self.seen = False
        self.NAME = None
        self.notrec = True

    #face recognition function
    def callback(self,data):
        #bridge between ros msg and opencv
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        face_cascade = cv2.CascadeClassifier('/home/turtlebot/catkin_ws/src/finalproject/src/haarcascade_frontalface_default.xml')
        self.faces = face_cascade.detectMultiScale(self.gray,1.3,4)
        #detect faces from the current video stream
        rospy.loginfo(len(self.faces))
        if len(self.faces) > 0:
            #if len is > 0, at least one human.
            self.seen = True
            for (x,y,w,h) in self.faces:
                face = self.gray[y:y + h, x:x + w]
                face_resize = cv2.resize(face,(self.width,self.height))
                prediction = self.model.predict(face_resize)
                rospy.loginfo(prediction)
                if prediction[1] < 400:
                    #based on the current prediction to check if we have detected someone
                    rospy.loginfo('I see a person')
                    rospy.loginfo(self.names[prediction[0]])
                    self.NAME = self.names[prediction[0]]
                    ropsy.loginfo(self.NAME)
                    self.notrec = False
                else:
                    self.NAME = None
                    self.notrec = True
                    rospy.loginfo('Do not recognize')
        else:
            self.notrec = False
            count = 0
            self.NAME = None
            self.seen = False
            rospy.loginfo('I do not see anyone')

    #publish current information for other node to subcribe to.
    def facerec(self):
        rate = rospy.Rate(10)
        if self.seen:
            if self.NAME == None:
                if self.notrec:
                    self.image_pub.publish("no reg")
            else:
                self.image_pub.publish(self.NAME)
        else:
            self.image_pub.publish("see no one")
            rate.sleep()

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
            rospy.loginfo("Mission failed")
        else:
            rospy.loginfo("Mission complete")
            rospy.sleep(1)


def main(args):
    ic = face_regconition()
    rospy.init_node('face_regconitionfuckyeah', anonymous=True)
    #run face recognition until node is shut down, constantly publishing images.
    while not rospy.is_shutdown():
        ic.facerec()
        rospy.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
