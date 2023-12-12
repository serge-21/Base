import smach
import rospy

class Control(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=["STOP", "CONTINUE"])
        self.robot = default

    def execute(self, userdata):
        if self.robot.base_controller.is_moving():
            return "CONTINUE"
        else:
            return "STOP"