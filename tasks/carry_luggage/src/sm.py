#!/usr/bin/env python3

import smach
from Default import Default
from States.Follow.look_for_people import LookForPeople
from States.Follow.go_to_person import GoToPerson
from States.add_to_costmap import AddToCostMap
from States.Pointing.detect_pointing_direction import DetectPointingDirection
from States.Cleanup.return_to_original_pose import ReturnToOriginalPose

'''
change the commented code to make the robot perform specific tasks. 
PLEASE ensure the simulation is correct before running the code
Right now the code waits for the robot to detect a pointing direction and then returns the direction
but for this to work, the simulation should have a person pointing and the person should be in front of the robot
'''
class CarryLuggage(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = Default()

        with self:
            # smach.StateMachine.add('START', LookForPeople(self.robot), transitions={'succeeded' : 'succeeded', 'failed' : 'failed'})
            # smach.StateMachine.add('GO_TO_PERSON', GoToPerson(self.robot), transitions={'succeeded' : 'START',  'failed' : 'failed'})
            # smach.StateMachine.add('ADD_TO_COSTMAP', AddToCostMap(self.robot), transitions={'succeeded' : 'succeeded', 'failed' : 'failed'})
            # smach.StateMachine.add('START', ReturnToOriginalPose(self.robot), transitions={'succeeded' : 'succeeded', 'failed' : 'failed'})
            smach.StateMachine.add('DETECT_POINTING_DIRECTION', DetectPointingDirection(self.robot), transitions={'Left' : 'succeeded', 'Right': 'succeeded', 'failed' : 'failed'})