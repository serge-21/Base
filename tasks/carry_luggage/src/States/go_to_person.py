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

        self.robot.last_person_pose = pose
        self.robot.base_controller.sync_to_pose(pose)

        # if pose != self.robot.last_person_pose:
        #     self.robot.last_person_pose = pose

        #     new_cords = self.calculate_point_along_line(pose)
        #     pose.position.x, pose.position.y = new_cords
        #     self.robot.base_controller.sync_to_pose(pose)
            
        return "succeeded"
    
    def calculate_speed_of_person(self, pose):
        speed = 0.0
        if self.robot.last_person_pose:
            x_curr, y_curr = pose.position.x, pose.position.y
            x_prev, y_prev = self.robot.last_person_pose.position.x, self.robot.last_person_pose.position.y
            speed = np.sqrt((x_curr - x_prev)**2 + (y_curr - y_prev)**2)

        return speed

    def calculate_point_along_line(self, person_pose, speed_threshold=0.2):
        x0, y0, _ = self.robot.base_controller.get_current_pose()
        x_person, y_person = person_pose.position.x, person_pose.position.y

        speed = self.calculate_speed_of_person(person_pose)
        distance = 1 if speed <= speed_threshold else 0.5

        # Calculate the slope and intercept of the line
        slope = (y_person - y0) / (x_person - x0)

        # calculate an x and y value that is distance away from the person, along the line, in the direction of the robot
        angle = np.arctan2(y_person - y0, x_person - x0)
        x_new = x_person - distance * np.cos(angle)
        y_new = y_person - distance * np.sin(angle)

        return (x_new, y_new)

    def estimate_person_coords(self, cords_of_person, depth):
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

                return (average_depth_x, average_depth_y, average_depth_z)

            except Exception as e:
                rospy.logerr("Error in estimating depth: {}".format(str(e)))

    def create_pose(self, cords):
        quat = self.robot.base_controller.get_current_pose()[2]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = cords
        pose.orientation.z, pose.orientation.w = quat.z, quat.w
        
        return pose