#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version by Ken Lauer / sirkne at gmail dot com
# 
# Implemented using the pybox2d SWIG interface for Box2D (pybox2d.googlecode.com)
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

class fwSettings(object):
    backend='pygame'        # The default backend to use in (can be: pyglet, pygame, etc.)

    # Physics options
    hz=60.0
    velocityIterations=8
    positionIterations=3
    enableWarmStarting=True   # Makes physics results more accurate (see Box2D wiki)
    enableContinuous=True     # Calculate time of impact
    enableSubStepping=True
    
    # Drawing
    drawStats=True
    drawShapes=True
    drawJoints=True
    drawCoreShapes=False
    drawAABBs=False
    drawOBBs=False
    drawPairs=False
    drawContactPoints=False
    maxContactPoints=100
    drawContactNormals=False
    drawFPS=True
    drawMenu=True             # toggle by pressing F1
    drawCOMs=False            # Centers of mass
    pointSize=2.5             # pixel radius for drawing points

    # PID gains, set by test program
    # Range from 0-100
    altitude_p = 1
    altitude_d = 1
    lateral_p = 1
    lateral_d = 1 
    attitude_p = 1 
    attitude_d = 1 

    # Miscellaneous testbed options
    pause=False
    singleStep=False
    onlyInit=False            # run the test's initialization without graphics, and then quit (for testing)

#             text                  variable
checkboxes =( ("Warm Starting"   , "enableWarmStarting"), 
              ("Time of Impact"  , "enableContinuous"), 
              ("Sub-Stepping"    , "enableSubStepping"),
              ("Draw"            , None),
              ("Shapes"          , "drawShapes"), 
              ("Joints"          , "drawJoints"), 
              ("AABBs"           , "drawAABBs"), 
              ("Pairs"           , "drawPairs"), 
              ("Contact Points"  , "drawContactPoints"), 
              ("Contact Normals" , "drawContactNormals"), 
              ("Center of Masses", "drawCOMs"), 
              ("Statistics"      , "drawStats"),
              ("FPS"             , "drawFPS"),
              ("Control"         , None),
              ("Pause"           , "pause"),
              ("Single Step"     , "singleStep") )

sliders = [
    { 'name' : 'altitude_p', 'text' : 'Altitude - P Gain', 'min' : 0.0, 'max' : 100 },
    { 'name' : 'altitude_d', 'text' : 'Altitude - D Gain', 'min' : 0.0, 'max' : 100 },
    { 'name' : 'lateral_p', 'text' : 'Lateral Drift - P Gain', 'min' : 0.0, 'max' : 100 },
    { 'name' : 'lateral_d', 'text' : 'Lateral Drift - D Gain', 'min' : 0.0, 'max' : 100 },
    { 'name' : 'attitude_p', 'text' : 'Attitude - P Gain', 'min' : 0.0, 'max' : 100 },
    { 'name' : 'attitude_d', 'text' : 'Attitude - D Gain', 'min' : 0.0, 'max' : 100 },
]

from optparse import OptionParser

parser = OptionParser()
list_options = [i for i in dir(fwSettings) if not i.startswith('_')]

for opt_name in list_options:
    value = getattr(fwSettings, opt_name)
    if isinstance(value, bool):
        if value:
            parser.add_option('','--NO'+opt_name, dest=opt_name, default=value,
                              action='store_'+str(not value).lower(),
                              help="don't "+opt_name)
        else:
            parser.add_option('','--'+opt_name, dest=opt_name, default=value,
                              action='store_'+str(not value).lower(),
                              help=opt_name)
            
    else:
        if isinstance(value, int):
            opttype = 'int'
        elif isinstance(value, float):
            opttype = 'float'
        else:
            opttype = 'string'
        parser.add_option('','--'+opt_name, dest=opt_name, default=value,
                          type=opttype,
                          help='sets the %s option'%(opt_name,))


(fwSettings, args) = parser.parse_args()
