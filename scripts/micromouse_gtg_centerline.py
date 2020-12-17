#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import pow,sqrt,atan2
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from mybot_description.msg import dest

class gtg_controller():
    def __init__(self):
        rospy.init_node("go_to_goal_controller")
       
        
        self.vel_x_pub = rospy.Publisher("/cmd_vel_x", Twist , queue_size = 1)
        self.vel_y_pub = rospy.Publisher("/cmd_vel_y", Twist , queue_size = 1)
        self.theta_pub = rospy.Publisher("/theota" , Float32 , queue_size = 1)
        rospy.Subscriber('/odom', Odometry , self.odom_callback)
        rospy.Subscriber('/dest' , dest , self.goal_pose_callback , queue_size = 1 , buff_size = 40)
        
        self.goal = [0.0 , 0.0]
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

    def odom_callback(self,msg):
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y

        self.current_orientation_quaternion[0] = msg.pose.pose.orientation.x
        self.current_orientation_quaternion[1] = msg.pose.pose.orientation.y
        self.current_orientation_quaternion[2] = msg.pose.pose.orientation.z
        self.current_orientation_quaternion[3] = msg.pose.pose.orientation.w
        (self.current_orientation_euler[1] , self.current_orientation_euler[0] , self.current_orientation_euler[2]) = euler_from_quaternion([self.current_orientation_quaternion[0] , self.current_orientation_quaternion[1] , self.current_orientation_quaternion[2] , self.current_orientation_quaternion[3]])
        self.current_pose[2] = self.current_orientation_euler[2]

    def control(self):
        
        if self.goal[0] - self.current_pose[0] >= 0:

            if self.goal[0] - self.current_pose[0] > 0.1:
                self.vel_x.linear.x = 0.4    
            elif (self.goal[0] - self.current_pose[0]) <= 0.1 and (self.goal[0] - self.current_pose[0]) >= 0.005:
                self.vel_x.linear.x = 4.0 * (self.goal[0] - self.current_pose[0])          
            else:    
                self.vel_x.linear.x = 0.0
                self.reached_x = True
                   
        else:
            if self.goal[0] - self.current_pose[0] < -0.1:
                self.vel_x.linear.x = -0.4 
            elif (self.goal[0] - self.current_pose[0]) >= -0.1 and (self.goal[0] - self.current_pose[0]) <= -0.005:
                self.vel_x.linear.x = 4.0 * (self.goal[0] - self.current_pose[0])
            else:    
                self.vel_x.linear.x = 0.0
                self.reached_x = True    

        if self.goal[1] - self.current_pose[1] >= 0:
            if self.goal[1] - self.current_pose[1] > 0.01:
                self.vel_y.linear.x = -0.4
            elif (self.goal[1] - self.current_pose[1]) <= 0.1 and (self.goal[1] - self.current_pose[1]) >= 0.005:
                self.vel_y.linear.x = -4.0 * ((self.goal[1] - self.current_pose[1]))
            else:    
                self.vel_y.linear.x = -0.0  
                self.reached_y = True    
         
        else:
            if self.goal[1] - self.current_pose[1] < -0.1:
                self.vel_y.linear.x = 0.4 
            elif (self.goal[1] - self.current_pose[1]) >= -0.1 and (self.goal[1] - self.current_pose[1]) <= -0.005:
                self.vel_y.linear.x = -4.0 * ((self.goal[1] - self.current_pose[1]))
            else:    
                self.vel_y.linear.x = -0.0  
                self.reached_y = True     
        

        if round(abs(self.current_pose[2]),4) != 0.0000:
            self.vel_x.angular.z = 2.5 * (-self.current_pose[2]) + 0.0000001 * ((-self.current_pose[2] + self.previous_theota)/self.sample_time)
            self.previous_theota = self.current_pose[2]
        else:
            self.vel_x.angular.z = 0.0  

        self.vel_x_pub.publish(self.vel_x)
        self.vel_y_pub.publish(self.vel_y)
        self.theta_pub.publish(self.current_pose[2])
        print("x:" , self.current_pose[0])
        print("y:" , self.current_pose[1])
        print("theota:" , self.current_pose[2])     

    def goal_pose_callback(self,msg):
        self.goal[0] = msg.dest_x_coordinate
        self.goal[1] = msg.dest_y_coordinate
        
        print("call_back recieved")
        self.publishing_rate = rospy.Rate(20)
        print("ready to go to desination")

        self.reached_x = False
        self.reached_y = False
        while True:
            self.control()
            if (self.reached_x and self.reached_y) :
                print("brokeee")
                break
            self.publishing_rate.sleep()
            
   
if __name__ == "__main__":
    yo = gtg_controller()
    rospy.spin()
    






