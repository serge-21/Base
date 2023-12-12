#!/usr/bin/env python3

import smach
from Default import Default
from States.look_for_people import LookForPeople
from States.go_to_person import GoToPerson

class CarryLuggage(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = Default()

        with self:
            smach.StateMachine.add('START', LookForPeople(self.robot), transitions={'succeeded' : 'GO_TO_PERSON', 'failed' : 'failed'})
            smach.StateMachine.add('GO_TO_PERSON', GoToPerson(self.robot), transitions={'succeeded' : 'START',  'failed' : 'failed'})