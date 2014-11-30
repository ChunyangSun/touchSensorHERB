# !/usr/bin/env python

import argparse, numpy, openravepy, time, IPython
import roslib
roslib.load_manifest("touchSensor")

import herbpy, prpy, logging, numpy, sys

from HerbRobot import HerbRobot
from HerbEnvironment import HerbEnvironment
# from ListenToArmTorque import ListenToArmTorque
from ListenToArmTorque import ListenToArmTorque

from SimpleMove import SimpleMove
# from SimpleMove import SimpleMove
# impoer threading and process
import threading, os 
from multiprocessing import Process

def main(robot, planning_env, planner, iterations = 1, show = False):
    pass 


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    
    parser.add_argument('-p', '--planner', type=str, default='simple')

    parser.add_argument('-m', '--manip', type=str, default='left')
    parser.add_argument('-v', '--visualize', action='store_true')
    parser.add_argument('-d', '--debug', action='store_true')
    parser.add_argument('-n', '--noshow', action='store_true')
    parser.add_argument('-i', '--iterations', type=int, default=1)
    parser.add_argument('-s', '--sim', action='store_true',
                        help='simulation mode')
    parser.add_argument('-gui', '--viewer', nargs='?', const=True, help='attach a viewer of the specified type')


    args = parser.parse_args()
    # args.robot_xml = 'models/robots/herb2_padded.robot.xml'

    herbpy_args = {'sim':args.sim,
                   'attach_viewer':args.viewer,
                   # 'robot_xml':args.robot_xml}
                   # 'env_path':args.env_path,
                   'segway_sim':True,
                   'vision_sim':True}

    if args.sim == "True":
        # uncomment for simulation
        env = openravepy.Environment()
        robot = HerbRobot(env, args.manip)
        print "simple move"

    else:
        env, robot = herbpy.initialize(**herbpy_args)
    
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
        openravepy.misc.InitOpenRAVELogging()

        if args.debug:
            openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)
        
        left_manip = robot.GetManipulator('left')
        robot.SetActiveDOFs(left_manip.GetArmIndices())

        robot.manip = left_manip
        robot.SetActiveManipulator('left')
        robot.SetActiveDOFs(robot.manip.GetArmIndices())
        robot.controller = openravepy.RaveCreateController(robot.GetEnv(), 'IdealController')

    env.SetViewer('qtcoin')
    env.GetViewer().SetName('Touch Me Not Viewer')
    planning_env = HerbEnvironment(robot)

    # Next setup the planner
    if args.planner == 'simple':
        planner = SimpleMove(planning_env, robot)
    elif args.planner == 'fancy':
        planner = FancyMove(planning_env, robot)
    else:
        print 'Unknown planner option: %s' % args.planner
        exit(0)


    # First setup the environment and the robot
    visualize = args.visualize
    planner.listenToFilteredTorque()

    main(robot, planning_env, planner, args.iterations, not args.noshow)
