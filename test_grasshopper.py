#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Basic TVC combined controller using 3 PD controllers.
# Based off of simple pygame framework by kne / sirkne at gmail dot com
from framework import *
from math import pi, sin, cos
from random import random
from settings import fwSettings
from time import clock

def clamp(val, minVal, maxVal):
    return max(minVal, min(maxVal, val))


class PID:
    """Simple PID config file"""
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d
    def __str__(self):
        return "PID(%g,%g,%g)" % (self.p, self.i, self.d)
        

# PID values [P, I, D] for TVC
pid_values = {'altitude':       PID(0.3, 0.02, 0.67),
              'lateral drift':  PID(1.5, 0, 3.55),
              'attitude':       PID(0.4, 0, 0.2)}


class Grasshopper(Framework):
    """You can use this class as an outline for your tests.

    """
    namebb = "Grasshopper" # Name of the class to display
    description="Dynamics of single-stage VTOL rocket"
    do_hover = False

    # Thrust Vector Control angle of rocket in radians
    tvc_angle = 0 
    goal_pos = b2Vec2(5,5)

    # Constants
    ship_mass = 10000
    max_tvc_angle = 10 * pi / 180 # max TVC angle +-
    max_tilt_angle = 20 * pi / 180 # Max tilt from verticle for rocket
    max_tvc_thrust_mult = 100000
    max_tvc_thrust = 500000
    ship_dimensions = (0.55, 3.3) # 10s of meters = 5.5 meters x 33 meters

    do_random_goal = False

    # Integral portion
    alpha = [0.2, 0.2, 0.2] # ratio of old to new data when calculating mean
    altitude_mean_error = 0.0


    settings = fwSettings
    def __init__(self):
        """ 
        Initialize all of your objects here.
        Be sure to call the Framework's initializer first.
        """
        super(Grasshopper, self).__init__()
        # self.world.gravity = (0.0,0.0)

        # Initialize all of the objects
        ground = self.world.CreateBody(position=(0, 20))
        ground.CreateEdgeChain(
                            [ (-20,-20),
                              (-20, 20),
                              ( 20, 20),
                              ( 20,-20),
                              (-20,-20) ]
                            )

        # Initialize sliders
        self.settings.altitude_p = pid_values['altitude'].p * 100
        self.settings.altitude_d = pid_values['altitude'].d * 100
        self.settings.lateral_p = pid_values['lateral drift'].p * 20
        self.settings.lateral_d = pid_values['lateral drift'].d * 20
        self.settings.attitude_p = pid_values['attitude'].p * 100
        self.settings.attitude_d = pid_values['attitude'].d * 100

        # Rocket
        self.ship=self.world.CreateDynamicBody(
            position=(0,6), angle=0.1,
            angularDamping=0, linearDamping=0)

        # And add a box fixture onto it (with a nonzero density, so it will move)
        box=self.ship.CreatePolygonFixture(box=self.ship_dimensions, density=self.ship_mass/(self.ship_dimensions[0]*self.ship_dimensions[1]), friction=0.3)

    def MouseDown(self, p):
        if (abs(p.x) < 20 and abs(p.y) < 40):
            self.goal_pos = p
        super(Grasshopper, self).MouseDown(p)


    def Keyboard(self, key):
        """
        The key is from Keys.K_*
        (e.g., if key == Keys.K_z: ... )
        """
        if not self.ship:
            return

        if key==Keys.K_w:
            f = self.ship.GetWorldVector(localVector=(0.0, 1400.0))
            p = self.ship.GetWorldPoint(localPoint=(0.0, 0.0))
            self.ship.ApplyForce(f, p, True)
        elif key==Keys.K_a:
            self.tvc_angle = max(-5 * pi/180, self.tvc_angle - 0.5 * pi/180)
        elif key==Keys.K_d:
            self.tvc_angle = min(5 * pi/180, self.tvc_angle + 0.5 * pi/180)
        elif key==Keys.K_h:
            self.do_hover = not self.do_hover
        elif key==Keys.K_i:
            # Info
            print "PID Values:"
            for pid in pid_values:
                print "%20s : %s" % (pid, pid_values[pid])
        elif key==Keys.K_r:
            # Random goals
            self.do_random_goal = ~self.do_random_goal
            if self.do_random_goal:
                print "Starting Random Goal generation."
                self.timer = clock()
                self.total_time = 0
                self.goals_reached = 0
                self.total_distance = 0
                self.last_pos = self.ship.position
            else:
                print "Stopping Random Goal generation."
                print "Number of goals reached: %d" % self.goals_reached
                if (self.goals_reached > 0):
                    avg_time = self.total_time / self.goals_reached
                    print "Average time to reach goal: %gs" % avg_time
                    print "Total distance: %g meters (Avg %g meters per second)" % (self.total_distance*10, self.total_distance*10 / avg_time)

    def hover(self, goalPos):
        """ Attempt to hover at a fixed point"""
        pos = self.ship.GetWorldPoint(localPoint=(0.0, -self.ship_dimensions[1]))

        ########################################
        # PD controller for altitude
        pos_error = goalPos.y - pos.y
        vel_error = 0 - self.ship.linearVelocity.y

        pid_values['altitude'].p = self.settings.altitude_p/100.
        pid_values['altitude'].d = self.settings.altitude_d/100.
        force_mag = pid_values['altitude'].p*pos_error + \
                    pid_values['altitude'].d*vel_error + \
                    self.ship.mass*10 / self.max_tvc_thrust_mult # gravity bias

        force_mag = clamp(force_mag * self.max_tvc_thrust_mult, 0, self.max_tvc_thrust)

        # 0 points up, left is +, right is -

        ########################################
        # PD for lateral position to goal orientation
        lat_error = goalPos.x - pos.x
        lat_vel_error = 0 - self.ship.linearVelocity.x

        # Values must be converted from 0-100 to whatever.
        pid_values['lateral drift'].p = self.settings.lateral_p/20.
        pid_values['lateral drift'].d = self.settings.lateral_d/20.

        tvc_offset = (lat_error*pid_values['lateral drift'].p + \
                      lat_vel_error*pid_values['lateral drift'].d) * -pi/180.0
        goal_orientation = clamp(tvc_offset, -self.max_tilt_angle, self.max_tilt_angle)


        ########################################
        # PD controller for TVC
        ang_error = goal_orientation - (self.ship.angle)
        ang_vel_error = 0 - self.ship.angularVelocity
        # ang vel is - clockwise
        
        pid_values['altitude'].p = self.settings.altitude_p/100.
        pid_values['altitude'].d = self.settings.altitude_d/100.
        d_angle = pid_values['attitude'].p * ang_error + \
                  pid_values['attitude'].d * ang_vel_error
        self.tvc_angle = clamp(d_angle, -self.max_tvc_angle, self.max_tvc_angle)

        ########################################
        fx = sin(self.tvc_angle) * force_mag
        fy = cos(self.tvc_angle) * force_mag

        f = self.ship.GetWorldVector(localVector=(fx, fy))
        l1 = self.renderer.to_screen(pos)
        l2 = self.renderer.to_screen(pos - f/self.max_tvc_thrust_mult)

        # Draw original TVC direction vector
        partial_down = self.renderer.to_screen(pos - self.ship.GetWorldVector(localVector=(0, fy))/self.max_tvc_thrust_mult)
        self.renderer.DrawSegment(l1,partial_down, b2Color(0.4,0.4,0.9))

        self.renderer.DrawSegment(l1,l2, b2Color(0.9,0.4,0.4))
        self.ship.ApplyForce(f,pos, True)

    def randGoalPos(self):
        " Uses ship dimension to avoid walls "
        wallBuffer = 3
        x = (40 - 2*self.ship_dimensions[0] - wallBuffer*2) *random() + wallBuffer + self.ship_dimensions[0]/2.0 - 20
        y = (40 - 2*self.ship_dimensions[1] - wallBuffer*2) *random() + wallBuffer + self.ship_dimensions[1]/2.0 - 20 + 20 # offset initial world box pos
        return b2Vec2(x,y)

    def Step(self, settings):
        """Called upon every step.
        You should always call
         -> super(Your_Test_Class, self).Step(settings)
        at the beginning or end of your function.

        If placed at the beginning, it will cause the actual physics step to happen first.
        If placed at the end, it will cause the physics step to happen after your code.
        """

        super(Grasshopper, self).Step(settings)

        if self.ship.angle >= 0:
            self.ship.angle %= 2*pi
        else:
            self.ship.angle %= -2*pi

        if self.do_random_goal:
            pos = self.ship.GetWorldPoint(localPoint=(0.0, -self.ship_dimensions[1]))
            if (self.goal_pos - pos).length < 0.1:
                self.total_distance += (self.goal_pos - self.last_pos).length

                self.timer = clock() - self.timer
                self.total_time += self.timer
                
                self.goals_reached += 1
                print "Goal reached in %gs (distance of %g meters)." % (self.timer, (self.goal_pos - self.last_pos).length*10)
                self.goal_pos = self.randGoalPos()
                self.timer = 0

        # do stuff
        if self.do_hover:
            self.hover(self.goal_pos)

        self.renderer.DrawPoint(self.renderer.to_screen(self.goal_pos), 2.0, b2Color(0.3,0.9,0.3))

        # Placed after the physics step, it will draw on top of physics objects
        self.Print("*** 'h' to enable Hover mode. Click to set goal position ***")

    def ShapeDestroyed(self, shape):
        """
        Callback indicating 'shape' has been destroyed.
        """
        pass

    def JointDestroyed(self, joint):
        """
        The joint passed in was removed.
        """
        pass

    # More functions can be changed to allow for contact monitoring and such.
    # See the other testbed examples for more information.

if __name__=="__main__":
    main(Grasshopper)

