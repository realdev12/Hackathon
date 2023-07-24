#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.79, Y + 0.66).
#you need to name this node "omnibase_controller"


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from robotics_hackathon_automation.msg import NextPoint


class controller:

    def __init__(self):
        print('orange')

        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0

        self.counter = 0
        self.path = []

        
        self.sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.sub2 = rospy.Subscriber("/planned_path", NextPoint, self.getPath)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        
        rospy.init_node("omnibase_controller")

        speed = Twist()
        
        r = rospy.Rate(5)
        
        goal = Point()
        # goal.x, goal.y = self.path[0]

        next = True
        n = 0

        # while not rospy.is_shutdown():
        #     print(self.path)
        #     r.sleep()



        while not rospy.is_shutdown():
            # print('blue')
            print(self.counter)
            if self.counter > 0:
                if n <= self.counter:
                    if next:
                        goal.x, goal.y = self.path[n]
                        n = n+1
                        next = False
                    
                    inc_x = goal.x -self.x
                    
                    inc_y = goal.y -self.y
                
                    angle_to_goal = atan2(inc_y, inc_x)
                    
                    if self.distance(goal.x, goal.y, self.x, self.y) < 0.1:
                        speed.linear.x = 0.0
                        speed.angular.z = 0.0
                        print('stopped')
                        next = True

                    elif abs(angle_to_goal - self.theta) > 0.1:
                        speed.linear.x = 0.0
                        speed.angular.z = 0.3
                        print('turning')

                    else:
                        speed.linear.x = 0.05
                        speed.angular.z = 0.0
                        print('Ahead')

                else:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    print('intial/final state')
                self.pub.publish(speed)
                r.sleep()    

    def newOdom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
    
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
    def distance(self,x1, y1, x2, y2):
        return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))

    # def getNextPoint(self):
    #     pass

    def getPath(self, Path):
        self.path.append((Path.x + 1.79, Path.y + 0.66))
        self.counter = self.counter + 1
        
controller() 



