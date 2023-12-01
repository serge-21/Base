import smach
import rospy

class DeattachBag(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.voice.speak("I am going to give you 8 seconds to deattach the bag")
        rospy.sleep(8)

        return "succeeded"