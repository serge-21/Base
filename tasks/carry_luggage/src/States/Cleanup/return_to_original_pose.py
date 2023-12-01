import smach

from geometry_msgs.msg import Pose, Point

class ReturnToOriginalPose(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot

    def execute(self, userdata):
        x, y, quat = self.robot.initial_pose
        pose = Pose(position=Point(x, y, 0.0), orientation=quat)

        self.robot.base_controller.sync_to_pose(pose)
        return "succeeded"