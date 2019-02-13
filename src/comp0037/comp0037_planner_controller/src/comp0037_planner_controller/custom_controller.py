#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from math import pow,atan2,sqrt
from comp0037_planner_controller.planned_path import PlannedPath
from comp0037_planner_controller.controller_base import CustomControllerBase
import math
import angles

# This Controller is implementing a similar algorithm to move the robot in stdr, however, 
# this implementation will amend the default one by slowly incrementing velocity and stop the 
# wiggling of the robot as it move, making the implementation unrealistic and/or wear out the robot
# significantly. 
# At first, the robot will turn to its goal
# Afterward, velocity will increase and decrease in step. 
# Turning time and moving time for each cell is 0.5s each.
# Two speed step, maximum speed = 2 * minimum speed
# By increasing number of step, we can ensure smoother acceleration and decceleration. 
# We are using fixed number of step for easier implementation

class CustomController(CustomControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)
        
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

    def publishVel(self,lx,az):
        vel_msg = Twist()
        vel_msg.linear.x = lx
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = az
        print("Linear Velocity: {}\nAngular Velocity: {}\n\n".format(vel_msg.linear.x, math.degrees(vel_msg.angular.z)))
        self.velocityPublisher.publish(vel_msg)
        if (self.plannerDrawer is not None):
            self.plannerDrawer.flushAndUpdateWindow()
        self.rate.sleep()
    
    def driveToWaypoint(self, waypoint,turn_time,move_time,speed_ratio):

        dX = waypoint[0] - self.pose.x
        
        dY = waypoint[1] - self.pose.y
        #  Find Total Distance need to move and total angle need to turn
        distanceError = sqrt(dX * dX + dY * dY)
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX))
        angleTurn = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX)) - self.driveAngleErrorTolerance/2
        distanceMove = distanceError - self.distanceErrorTolerance/2
        init_de = distanceError
        init_ae = angleError
        time_count = 0
        print("Current Pose: x: {}, y:{} , theta: {}\nGoal: x: {}, y: {}\n".format(self.pose.x, self.pose.y,
                                                                                    self.pose.theta, waypoint[0],
                                                                                        waypoint[1]))
        print("Distance Error: {}\nAngular Error: {}".format(distanceError, angleError))  
        # Calculate Ratio
        
        minimum_turn_ratio = (turn_time - 0.2) * speed_ratio + 0.2
        minimum_move_ratio = (move_time - 0.2) * speed_ratio + 0.2
        # Calculate each speed step
        minimum_turn_speed = angleTurn/ minimum_turn_ratio
        minimum_move_speed = distanceMove/ minimum_move_ratio
        maximum_turn_speed = minimum_turn_speed * speed_ratio
        maximum_move_speed = minimum_move_speed * speed_ratio
        lx = [0,0,0,0,0,minimum_move_speed,maximum_move_speed,maximum_move_speed,maximum_move_speed,minimum_move_speed,0]
        ax = [minimum_turn_speed,maximum_turn_speed,maximum_turn_speed,maximum_turn_speed,minimum_turn_speed,0,0,0,0,0,0]
        # Publish the vel_msg to run the robot
        for i in range(len(lx)):
            self.publishVel(lx[i],ax[i])
            time_count = time_count + 1

        print(time_count)
        # Find Error when arrive at waypoint
        distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
        angleError = self.shortestAngularDistance(self.pose.theta,
                                                    atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        # Output Distance Travelled
        total_distance = init_de - distanceError
        total_theta = abs(init_ae)
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
        
