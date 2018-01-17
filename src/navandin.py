#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import math
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from cob_perception_msgs.msg import PositionMeasurementArray, PositionMeasurement


#Written by Lam Ngo, Nam Bui and Dae Kwang Lee.
#Based on code from http://learn.turtlebot.com/2015/02/01/14/ and Nexus.

class myRobot:
    def __init__(self):
        #if it has seen a person
        self.see = False
        #if it has recognized
        self.rec = False
        self.goal_sent = False
        #subcribe to the face recognition node to get identification.
        rospy.Subscriber("/face_recognition", String, self.callback)
    	# What to do if shut down (e.g. Ctrl-C or failure)
    	rospy.on_shutdown(self.shutdown)

    	# Tell the action client that we want to spin a thread by default
    	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    	rospy.loginfo("Wait for the action server to come up")

    	# Allow up to 5 seconds for the action server to come up
    	self.move_base.wait_for_server(rospy.Duration(5))
        self.done = False

        #if leg_detection has detected a person
        self.go = False
        #empty people array for leg_detection
        self.person_pose = PositionMeasurementArray()
        #Name of the current recognized person
        self.name = None

    #callback to get the pose of the detected person (by leg_detection)
    def poseCallback(self,data):
        #if it has successfully detected a person
        if data != None:
            self.person_pose = data
            if len(self.person_pose.people) > 0:
                if self.person_pose.people[0].reliability > 0.5:
                    self.go = True
                else:
                    self.go = False
            else:
                self.go = False

    #callback to get human identification information from face_regconition node.
    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if (data.data == "no reg"):
            #see someone but does not recognize (not on the database)
            self.see = True
            self.rec = False
            self.name = None
        elif (data.data == "see no one"):
            #Nobody is in sight.
            self.see = False
            self.rec = False
            self.name = None
        else:
            #It has seen a person and successfully identified it.
            self.see = True
            self.rec = True
            self.name = data.data

    #to check whether there is a human or not
    def goOrnot(self):
        return self.go

    #Get current person pose
    def getPerson(self):
        return self.person_pose

    #Go to a specific point in map
    def goto(self, pos, quat):
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
	    # Start moving
        self.move_base.send_goal(goal)

	    # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # Goal was successfully recieved.
            result = True
        else:
            #failed to reach goal.
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
            rospy.loginfo("Mission failed")
        else:
            rospy.loginfo("Mission complete")
        rospy.sleep(1)

    #Using festival speech synthesis and sound_play node.
    def say(self,whattosay):
        voice = 'voice_kal_diphone'
        volume = 1.0
        s = whattosay
        rospy.loginfo(whattosay)
        soundhandle = SoundClient()
        rospy.sleep(1)
        soundhandle.say(s, voice)
        rospy.sleep(1)

    #Perform face recognition based on information recieved by the callback function
    def facerec(self):
        if self.see == True and self.rec == True:
            self.say("I found" + self.name)
            rospy.sleep(1)
            self.done = True
            return True
        elif self.see == True and self.rec == False:
            self.say("I do not know who you are")
            rospy.sleep(1)
            return False
        else:
            self.say("I do not see anyone")
            return False

#distance between two points
def findDistance(x1,x0,y1,y0):
    newDistance = math.sqrt((x1-x0)**2 + (y1-y0)**2)
    return newDistance

#convert degrees to radians
def degrees2radians(angle):
	return angle * (math.pi/180.0)

#Rotate a certain amount of angle
#Does not work too well.
def rotate(relative_angle, isClockwise):
    rospy.loginfo("rotating")
    #self.say("i am fucking rotating")
	# publish to the right topic
    pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=10)
	#creates a blank twist message
    outData = Twist()
	#get initial time
    t0 = rospy.get_rostime().secs
    while t0 == 0:
        t0 = rospy.get_rostime().secs
	#initialize the current angle to be 0
    current_angle = 0
	#Set rospy rate to be 10hz
    rate = rospy.Rate(10)
	# speed is 10 degrees per second.
    speed = degrees2radians(30)
    if isClockwise:
        outData.angular.z = -abs(speed)
    else:
        outData.angular.z = abs(speed)
    while current_angle < relative_angle:
        pub.publish(outData)
        current_angle = speed * (rospy.get_rostime().secs-t0)
        rate.sleep()

#Move forward or backward a current distance
def move(distance, isForward):
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    outData = Twist()
    t0 = rospy.get_rostime().secs
    while (t0 == 0):
        t0 = rospy.get_rostime().secs
    current_distance = 0
    rate = rospy.Rate(10)
    if(isForward == True):
    	outData.linear.x = 0.2
    else:
    	outData.linear.x = -0.2
    while (current_distance < distance):
    	pub.publish(outData)
    	t1 = rospy.get_rostime().secs
    	current_distance = 0.2*(t1 - t0)
    	rate.sleep()

#Function to stop the robot
def stop():
	pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
	outData = Twist()
	rate = rospy.Rate(10)
	rospy.loginfo("stop never stop stopping")
	pub.publish(outData)
	rate.sleep()

#perform human detection, if sucessful, perform face recognition.
def mission():
    detected = detectHuman()
    t0 = rospy.get_rostime().secs
    t1 = rospy.get_rostime().secs
    #do so for 12 seconds
    while (t1 - t0 <= 12) and not detected:
        rotate(degrees2radians(130),True)
        rospy.sleep(1)
        detected = detectHuman()
        t1 = rospy.get_rostime().secs
    rospy.sleep(1)
    if detected:
        navigator.say("I have detected a person")
        rospy.sleep(1)
        navigator.say("Start face recognition")
        rospy.sleep(1)
        t0 = rospy.get_rostime().secs
        t1 = rospy.get_rostime().secs
        recognized = False
        #do so for 10 seconds, as face recognition message are often delayed.
        while t1 - t0 < 10 and not recognized:
            recognized = navigator.facerec()
            t1 = rospy.get_rostime().secs
            rospy.sleep(1)
    else:
        navigator.say("I did not see anyone. Moving to the next goal.")
        rospy.sleep(1)

#Function to rotate the robot and move to the detected human
def detectHuman():
    rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, navigator.poseCallback, queue_size=1)
    rate = rospy.Rate(100)
    t0 = rospy.get_rostime().secs
    rate.sleep()
    y = 1
    go = navigator.goOrnot()
    test = False
    t1 = rospy.get_rostime().secs
    while not go and (t1 - t0 < 6) and not rospy.is_shutdown():
        rospy.loginfo("looking for human")
        go = navigator.goOrnot()
        t1 = rospy.get_rostime().secs
    if go:
        person = navigator.getPerson()
        x = person.people[0].pos.x
        y = person.people[0].pos.y
        rospy.loginfo(x)
        rospy.loginfo(y)
        distance = findDistance(x,0,y,0) - 1
        angle = math.atan(abs(y)/abs(x))
        rospy.loginfo(angle)
        while y != 0 and not rospy.is_shutdown():
            if len(navigator.getPerson().people) > 0:
                rospy.loginfo(navigator.getPerson().people[0].pos.y)
                y = navigator.getPerson().people[0].pos.y
                if abs(navigator.getPerson().people[0].pos.y) <= 0.4:
                    y = 0
                else:
                    if y > 0:
                        rotate(degrees2radians(0.5), False)
                        rate.sleep()
                    else:
                        rotate(degrees2radians(0.5), True)
                        rate.sleep()
            else:
                y = 0
                stop()
        move(distance, True)
        rate.sleep()
        rospy.loginfo("moved to person")
        return True
    else:
        rospy.loginfo("did not identify a person")
        return False

if __name__ == '__main__':
    rospy.init_node('finalproject_test', anonymous = False)
    navigator = myRobot()
    quaterion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    #set of points to go in map
    coordinates = [{'x': 7.29, 'y': -1.4}, {'x': 0.06, 'y' : 5.3}, {'x': -7.74, 'y' : 1.92},
		{'x': -7.81, 'y' : 4.46}, {'x': -11.3, 'y' : 2.96}, {'x': -12.7, 'y' : -0.087}, {'x': -8.93, 'y' : -0.908}]
    #loop to go to each one of them.
    count = 0
    while count < 7:
        if count == 0:
            rospy.loginfo("Intializing coordinates to x: %s, y: %s", coordinates[count]["x"], coordinates[count]["y"])
            navigator.goto(coordinates[count], quaterion)
            navigator.say("I have reached my destination")
            rospy.sleep(1)
            count = count + 1
            mission() #perform human detection and human identification
            rospy.sleep(1)
            rospy.loginfo("Go to x: %s, y: %s", coordinates[count]["x"], coordinates[count]["y"])
        else:
            if navigator.goto(coordinates[count], quaterion):
                rospy.loginfo("The base reached the desired pose at x: %s, y: %s", coordinates[count]["x"], coordinates[count]["y"])
                navigator.say("I have reached my destination")
                mission() #perform human detection and human identification
                rospy.sleep(1)
            else:
                rospy.loginfo("The base failed to reach the desired pose")
                rospy.sleep(1)
            count = count + 1
