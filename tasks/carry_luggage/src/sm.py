#!/usr/bin/env python3

import smach
from Default import Default
from States.look_for_people import LookForPeople
from States.go_to_person import GoToPerson

class CarryLuggage(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        self.default = Default()

        with self:
            smach.StateMachine.add('START', LookForPeople(self.default), transitions={'succeeded' : 'GO_TO_PERSON', 'look_for_people' : 'START'})
            smach.StateMachine.add('GO_TO_PERSON', GoToPerson(self.default), transitions={'succeeded' : 'succeeded',  'failed' : 'failed'})