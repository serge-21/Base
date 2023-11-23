import rospy
import smach
import numpy as np

from geometry_msgs.msg import Pose, PointStamped, Point
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from tf_module.srv import TfTransformRequest

class GoToPerson(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['coords'])

        self.header = None
        self.default = default

    def execute(self, userdata):
        cords = self.estimate_person_coords(userdata.coords)
        pose = self.estimate_pose(cords)

        self.default.base_controller.sync_to_pose(pose)
        self.default.voice.speak("I am here please give me hug?")
        
        return "succeeded"

    def estimate_person_coords(self, cords_of_person):
        depth = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        self.header = depth.header

       # Check if depth information is available
        if depth is not None:
            try:
                # Convert PointCloud2 message to a numpy array
                depth_data = point_cloud2.read_points(depth, field_names=("x", "y", "z"), skip_nans=True)
                depth_array = np.array(list(depth_data))

                # Extract depth values for the coordinates of the person
                person_depth_values_x = []
                person_depth_values_y = []
                person_depth_values_z = []
                for cord in cords_of_person:
                    x, y, z = depth_array[cord]
                    person_depth_values_x.append(x)
                    person_depth_values_y.append(y)
                    person_depth_values_z.append(z)

                # Calculate the average depth
                average_depth_x = np.mean(person_depth_values_x)
                average_depth_y = np.mean(person_depth_values_y)
                average_depth_z = np.mean(person_depth_values_z)

                rospy.loginfo("Estimated depth of the person: {}".format((average_depth_x, average_depth_y, average_depth_z)))
                return (average_depth_x, average_depth_y, average_depth_z)

            except Exception as e:
                rospy.logerr("Error in estimating depth: {}".format(str(e)))

    def estimate_pose(self, person_cords):
        x, y, z = person_cords

        centroid = PointStamped()
        centroid.point = Point(x, y, z)
        centroid.header = self.header
        
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = centroid
        
        response = self.default.tf_service(tf_req)
        
        target_pose = Pose()
        target_pose.position.x = response.target_point.point.x
        target_pose.position.y = response.target_point.point.y
        target_pose.position.z = 0.0
        target_pose.orientation.w = 1.0

        return target_pose