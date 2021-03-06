#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from math import pow,atan2,sqrt
from comp0037_planner_controller.planned_path import PlannedPath
from comp0037_planner_controller.controller_base import Improved2ControllerBase
import math
import angles

# This sample controller works a fairly simple way. It figures out
# where the goal is. It first turns the robot until it's roughly in
# the correct direction and then keeps driving. It monitors the
# angular error and trims it as it goes.

class Improved2Controller(Improved2ControllerBase):

    def __init__(self, occupancyGrid):
        Improved2ControllerBase.__init__(self, occupancyGrid)
        
        # Get the proportional gain settings
        self.distanceErrorGain = rospy.get_param('distance_error_gain', 1)
        self.angleErrorGain = rospy.get_param('angle_error_gain', 4)

        self.driveAngleErrorTolerance = math.radians(rospy.get_param('angle_error_tolerance', 1))
    
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def shortestAngularDistance(self, fromAngle, toAngle):
        delta = toAngle - fromAngle
        if (delta < -math.pi):
            delta = delta + 2.0*math.pi
        elif(delta > math.pi):
            delta = delta - 2.0*math.pi
        return delta
        
    def driveToWaypoint(self, waypoint,waypoint1):
        vel_msg = Twist()

        dX = waypoint[0] - self.pose.x
        
        dY = waypoint[1] - self.pose.y
        
        distanceError = sqrt(dX * dX + dY * dY)
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX))
        init_de = distanceError
        init_ae = angleError
        time_count = 0
        while (distanceError >= init_de*0.15) & (not rospy.is_shutdown()):
            print("Current Pose: x: {}, y:{} , theta: {}\nGoal: x: {}, y: {}\n".format(self.pose.x, self.pose.y,
                                                                                       self.pose.theta, waypoint[0],
                                                                                         waypoint[1]))
            print("Distance Error: {}\nAngular Error: {}".format(distanceError, angleError))  
            # Proportional Controller
            # linear velocity in the x-axis: only switch on when the angular error is sufficiently small
            if math.fabs(angleError) < self.driveAngleErrorTolerance:
                
                vel_msg.linear.x = init_de
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                


            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))
            
            
            print("Linear Velocity: {}\nAngular Velocity: {}\n\n".format(vel_msg.linear.x, math.degrees(vel_msg.angular.z)))
            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
            time_count = time_count + 1
            print(time_count)    
            self.rate.sleep()
            # print('Position:')
            # print(self.pose.theta)
            # print(self.pose.x)
            # print(self.pose.y)
            # print('Distance Moved:') 
            # print(sqrt(pow((init_pos_x - self.pose.x), 2) + pow((init_pos_y - self.pose.y), 2))) 
            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta,
                                                      atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))
        if waypoint1 == [0,0]:
            dX1 = dX
            dY1 = dY
        else:
            dX1 = waypoint1[0] - self.pose.x
            dY1 = waypoint1[1] - self.pose.y

        while (distanceError >= self.distanceErrorTolerance) & (not rospy.is_shutdown()):
            print("Current Pose: x: {}, y:{} , theta: {}\nGoal: x: {}, y: {}\n".format(self.pose.x, self.pose.y,
                                                                                       self.pose.theta, waypoint[0],
                                                                                         waypoint[1]))
            print("Distance Error: {}\nAngular Error: {}".format(distanceError, angleError))  
            # Proportional Controller
            # linear velocity in the x-axis: only switch on when the angular error is sufficiently small
            if math.fabs(angleError) < self.driveAngleErrorTolerance:
                
                vel_msg.linear.x = init_de*0.5
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                


            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * NangleError, 5.0))
            
            
            print("Linear Velocity: {}\nAngular Velocity: {}\n\n".format(vel_msg.linear.x, math.degrees(vel_msg.angular.z)))
            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
            time_count = time_count + 1
            print(time_count)    
            self.rate.sleep()
            # print('Position:')
            # print(self.pose.theta)
            # print(self.pose.x)
            # print(self.pose.y)
            # print('Distance Moved:') 
            # print(sqrt(pow((init_pos_x - self.pose.x), 2) + pow((init_pos_y - self.pose.y), 2))) 
            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta,
                                                      atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))
        NangleError = self.shortestAngularDistance(self.pose.theta, atan2(dY1, dX1))
        # Make sure the robot is stopped once we reach the destination.
        print(init_de)
        print(distanceError)
        print(init_ae)
        print(angleError)
        total_distance = init_de - distanceError
        total_theta = abs(init_ae)
        if waypoint1 == [0,0]:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocityPublisher.publish(vel_msg)
        print('Waypoint Distance:')
        print(total_distance)
        print(total_theta)
        return(total_distance,total_theta,time_count)

    def rotateToGoalOrientation(self, goalOrientation):
        vel_msg = Twist()

        goalOrientation = math.radians(goalOrientation)

        angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)
        init_ae = angleError
        t_theta = 0
        while (math.fabs(angleError) >= self.goalAngleErrorTolerance) & (not rospy.is_shutdown()):
            #print 'Angular Error: ' + str(angleError)

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))
            
            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
            t_theta = t_theta +1
            self.rate.sleep()
            angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)

        # Stop movement once finished
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)
        return((init_ae - angleError),t_theta)
        
