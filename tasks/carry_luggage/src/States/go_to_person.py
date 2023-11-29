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
                             input_keys=['coords', 'pcl'])

        self.header = None
        self.default = default

    def execute(self, userdata):
        cords = self.estimate_person_coords(userdata.coords, userdata.pcl)        
        pose = self.estimate_pose(cords)

        if pose != self.default.last_person_pose:
            self.default.last_person_pose = pose
            print("Person pose: {}".format(pose))

            # new_cords = self.calculate_point_along_line(pose)
            # pose.position.x, pose.position.y = new_cords
            self.default.base_controller.sync_to_pose(pose)
            
        return "succeeded"
    
    def calculate_speed_of_person(self, pose):
        speed = 0.0
        if self.default.last_person_pose:
            x_curr, y_curr = pose.position.x, pose.position.y
            x_prev, y_prev = self.default.last_person_pose.position.x, self.default.last_person_pose.position.y
            speed = np.sqrt((x_curr - x_prev)**2 + (y_curr - y_prev)**2)

        return speed

    def calculate_point_along_line(self, person_pose, speed_threshold=0.2):
        x0, y0, _ = self.default.base_controller.get_current_pose()
        x_person, y_person = person_pose.position.x, person_pose.position.y

        speed = self.calculate_speed_of_person(person_pose)
        distance = 0.3 if speed <= speed_threshold else 0.1

        # Calculate the slope and intercept of the line
        slope = (y_person - y0) / (x_person - x0)

        # calculate an x and y value that is distance away from the person, along the line, in the direction of the robot
        angle = np.arctan(slope)
        x_new = x_person + distance * np.cos(angle)
        y_new = y_person + distance * np.sin(angle)

        return (x_new, y_new)

    def estimate_person_coords(self, cords_of_person, depth):
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