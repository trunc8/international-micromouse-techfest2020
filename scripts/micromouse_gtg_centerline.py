#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import pow,sqrt,atan2
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from pkg_techfest_imc.msg import dest
import sys

class gtg_controller():
    def __init__(self):
        rospy.init_node("go_to_goal_controller")
       
        
        self.vel_x_pub = rospy.Publisher("/cmd_vel_x", Twist , queue_size = 1)
        self.vel_y_pub = rospy.Publisher("/cmd_vel_y", Twist , queue_size = 1)
        self.theta_pub = rospy.Publisher("/theota" , Float32 , queue_size = 1)
        rospy.Subscriber('/odom', Odometry , self.odom_callback)
        rospy.Subscriber('/dest' , dest , self.goal_pose_callback , queue_size = 1 , buff_size = 40)
        self.ack_pub = rospy.Publisher('/acknowledge', Float32 , queue_size=1)

        #self.prev_goal = [0.0 , 0.0]
        self.goal = [-1.0 , -1.36]
        self.current_pose = [0.0 , 0.0 , 0.0]  #x,y,theta
        self.current_orientation_euler = [0.0 , 0.0 , 0.0 ]
        self.current_orientation_quaternion = [0.0 , 0.0 , 0.0 , 0.0]

        self.vel_x = Twist()
        self.vel_x.linear.x = 0.0
        self.vel_x.linear.y = 0.0
        self.vel_x.linear.z = 0.0
        self.vel_x.angular.x = 0.0
        self.vel_x.angular.y = 0.0
        self.vel_x.angular.z = 0.0

        self.vel_y = Twist()
        self.vel_y.linear.x = 0.0
        self.vel_y.linear.y = 0.0
        self.vel_y.linear.z = 0.0
        self.vel_y.angular.x = 0.0
        self.vel_y.angular.y = 0.0
        self.vel_y.angular.z = 0.0

        self.sample_time = 0.05
        self.previous_theota = 0.0

        self.reached_x = False
        self.reached_y = False

        self.publishing_rate = rospy.Rate(20)

        self.max_speed_y = 0.05
        self.max_speed_x = 0.11
        self.max_speed_scaled_x = self.max_speed_x*10
        self.max_speed_scaled_y = self.max_speed_y*10
        self.min_thresh = 0.01 #if distance to goal is btw 0.05 and 0.01, then p controller is used to prevent overshoot
        self.max_thresh = 0.1  #min distance upto which the speed given to the bot is 0.4
        self.init_current_pose = [0.0 , 0.0 ,0.0]

    def odom_callback(self,msg):
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y

        self.current_orientation_quaternion[0] = msg.pose.pose.orientation.x
        self.current_orientation_quaternion[1] = msg.pose.pose.orientation.y
        self.current_orientation_quaternion[2] = msg.pose.pose.orientation.z
        self.current_orientation_quaternion[3] = msg.pose.pose.orientation.w
        (self.current_orientation_euler[1] , self.current_orientation_euler[0] , self.current_orientation_euler[2]) = euler_from_quaternion([self.current_orientation_quaternion[0] , self.current_orientation_quaternion[1] , self.current_orientation_quaternion[2] , self.current_orientation_quaternion[3]])
        self.current_pose[2] = self.current_orientation_euler[2]
        
        

    def goal_theota(self):
        self.goal_yaw = atan2(self.goal[1] - self.current_pose[1], self.goal[0] - self.current_pose[0])  
        # print("goal_YAW", self.goal_yaw)

        
    def distance_from_goal(self , pose_x , pose_y , goal_x , goal_y):
        distance = sqrt(pow((goal_x - pose_x), 2) + pow((goal_y - pose_y), 2))
        return distance


    def control(self):
        #distance = self.distance_from_goal(self.current_pose[0] , self.current_pose[1] , self.goal[0] , self.goal[1])
        if self.goal[0] - self.current_pose[0] >= 0:

            if self.goal[0] - self.current_pose[0] >= self.max_thresh:
                self.vel_x.linear.x = self.max_speed_x #0.1 may change, or to cover the path faster we can put constant 0.4 and when it reaches goal we can make it 0
                #print("away from goal in X_111111111111111111")
            elif (self.goal[0] - self.current_pose[0]) <= self.max_thresh and (self.goal[0] - self.current_pose[0]) >= self.min_thresh:
                self.vel_x.linear.x = self.max_speed_scaled_x * (self.goal[0] - self.current_pose[0])
                #print("getting close in X_11111111111")
            else:    
                self.vel_x.linear.x = 0.0
                #print("Give next goal in X_111111")
                
                   
        else:
            if self.goal[0] - self.current_pose[0] <= (-1 * self.max_thresh):
                self.vel_x.linear.x = (-1 * self.max_speed_x) #0.1 may change, or to cover the path faster we can put constant 0.4 and when it reaches goal we can make it 0
                #print("away from goal in X_2222222222222")
            elif (self.goal[0] - self.current_pose[0]) >= (-1 * self.max_thresh) and (self.goal[0] - self.current_pose[0]) <= (-1 * self.min_thresh):
                self.vel_x.linear.x = self.max_speed_scaled_x * (self.goal[0] - self.current_pose[0])
                #print("getting close in X_222222222222")
            else:    
                self.vel_x.linear.x = 0.0
                #print("Give next goal in X_2222222222")
                    

        if self.goal[1] - self.current_pose[1] >= 0:
            if self.goal[1] - self.current_pose[1] > self.max_thresh:
                self.vel_y.linear.x = -self.max_speed_y #0.1 may change, or to cover the path faster we can put constant 0.4 and when it reaches goal we can make it 0
                # print("away from goal in Y_111111111111")
            elif (self.goal[1] - self.current_pose[1]) <= self.max_thresh and (self.goal[1] - self.current_pose[1]) >= self.min_thresh:
                self.vel_y.linear.x = -self.max_speed_scaled_y * ((self.goal[1] - self.current_pose[1]))
                # print("getting close in Y_111111111111")
            else:    
                self.vel_y.linear.x = -0.0  
                # print("give next goal in Y_11111111")
                
        else:
            if self.goal[1] - self.current_pose[1] < (-1 * self.max_thresh):
                self.vel_y.linear.x = self.max_speed_y #0.1 may change, or to cover the path faster we can put constant 0.4 and when it reaches goal we can make it 0
                # print("away from goal in Y_2222222222")
            elif (self.goal[1] - self.current_pose[1]) >= (-1 * self.max_thresh) and (self.goal[1] - self.current_pose[1]) <= (-1 * self.min_thresh):
                self.vel_y.linear.x = -self.max_speed_scaled_y * ((self.goal[1] - self.current_pose[1]))
                # print("getting close in Y_222222222222")
            else:    
                self.vel_y.linear.x = -0.0  
                # print("give next goal in Y_22222222222222")
                  
        
        #if round(abs(self.goal[0] - self.current_pose[0]), 2) == 0:
        if round(abs(self.current_pose[2]),3) != 0.0000:
            self.vel_x.angular.z = 2.5 * (-self.current_pose[2]) + 0.0000001 * ((-self.current_pose[2] + self.previous_theota)/self.sample_time)
            self.previous_theota = self.current_pose[2]
                #self.vel_x.angular.z = 0.0
            self.vel_y.angular.z = 0.0
        else:
            self.vel_x.angular.z = 0.0  
            self.vel_y.angular.z = 0.0

       
       

        self.vel_x_pub.publish(self.vel_x)
        self.vel_y_pub.publish(self.vel_y)
        self.theta_pub.publish(self.current_pose[2])
        # print("x:" , self.current_pose[0])
        # print("y:" , self.current_pose[1])
        # print("theota:" , self.current_pose[2])     

    def goal_pose_callback(self,msg):
        self.goal[0] = msg.dest_x_coordinate
        self.goal[1] = msg.dest_y_coordinate
        
        print("call_back recieved")
        print("Goal location: (%f,%f)" % (self.goal[0], self.goal[1]))
        
        print("ready to go to desination")

        
        # while True:
        #     self.control()
        #     self.publishing_rate.sleep()
            
   
if __name__ == "__main__":
    yo = gtg_controller()
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 3:
        print("Incoorect number of arguments")
        sys.exit()

    yo.goal[0] = float(args[1])
    yo.goal[1] = float(args[2])   
    while not rospy.is_shutdown():
        yo.control()
        yo.publishing_rate.sleep()
    # rospy.spin()
    






