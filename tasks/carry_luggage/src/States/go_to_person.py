import rospy
import smach
import numpy as np

from geometry_msgs.msg import Pose
from sensor_msgs import point_cloud2

class GoToPerson(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['coords', 'pcl'])

        self.header = None
        self.robot = default

    def execute(self, userdata):
        cords = self.estimate_person_coords(userdata.coords, userdata.pcl)
        map_cords = self.robot.translate_coord_to_map(cords, self.header)    
        pose = self.create_pose(map_cords)

        if pose != self.robot.last_person_pose:
            self.robot.last_person_pose = pose

            new_cords = self.calculate_point_along_line(pose)
            pose.position.x, pose.position.y = new_cords
            self.robot.base_controller.sync_to_pose(pose)
            
        return "succeeded"
    
    def calculate_speed_of_person(self, pose):
        speed = 0.0
        if self.robot.last_person_pose:
            x_curr, y_curr = pose.position.x, pose.position.y
            x_prev, y_prev = self.robot.last_person_pose.position.x, self.robot.last_person_pose.position.y
            speed = np.sqrt((x_curr - x_prev)**2 + (y_curr - y_prev)**2)

        return speed

    def calculate_point_along_line(self, person_pose, speed_threshold=0.2):
        robot_x, robot_y, _ = self.robot.base_controller.get_current_pose()
        person_x, person_y = person_pose.position.x, person_pose.position.y

        speed = self.calculate_speed_of_person(person_pose)
        distance = 1 if speed <= speed_threshold else 0.5

        vector_x = person_x - robot_x
        vector_y = person_y - robot_y

        # Normalise the vector
        length = np.sqrt(vector_x**2 + vector_y**2)
        normalised_vector_x = vector_x / length
        normalised_vector_y = vector_y / length

        # Calculate the new point that is "distance" meters away from the person along the vector
        new_point_x = person_x - distance * normalised_vector_x
        new_point_y = person_y - distance * normalised_vector_y

        return (new_point_x, new_point_y)

    def estimate_person_coords(self, cords_of_person, depth):
        self.header = depth.header

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

            return (average_depth_x, average_depth_y, average_depth_z)

        except Exception as e:
            rospy.logerr("Error in estimating depth: {}".format(str(e)))

    def create_pose(self, cords):
        quat = self.robot.base_controller.get_current_pose()[2]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = cords
        pose.orientation.z, pose.orientation.w = quat.z, quat.w
        
        return pose