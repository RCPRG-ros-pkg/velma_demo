#!/usr/bin/env python

## Runs test for motions in cart_imp mode.
# @ingroup integration_tests
# @file test_cimp_imp.py
# @namespace scripts.test_cimp_imp Integration test

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    # define some configurations
    rospy.init_node('test_cimp_imp')

    rospy.sleep(0.5)

    print "This test/tutorial executes simple impedance commands"\
        " in Cartesian impedance mode.\n"

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    if velma.enableMotors() != 0:
        exitError(2)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(3)

    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(4)
    print "Planner init ok"


    def makeWrench(lx,ly,lz,rx,ry,rz):
        return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(10)
    if velma.waitForEffectorRight() != 0:
        exitError(11)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(12)

    print "To see the tool frame add 'tf' in rviz and enable 'right_arm_tool' frame."
    print "At every state switch to cart_imp, the tool frames are reset."
    print "Also, the tool impedance parameters are reset to 1500N/m in every"\
        " direction for linear stiffness and to 150Nm/rad in every direction for angular"\
        " stiffness, i.e. (1500,1500,1500,150,150,150)."

    print "Set impedance to (1000,1000,1000,150,150,150) in tool frame in 8 seconds."
    if not velma.moveCartImpRight(None, None, None, None, [makeWrench(1000,1000,1000,150,150,150)], [8], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(0.5)

    exitError(0)

