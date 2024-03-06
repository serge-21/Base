#!/usr/bin/env python3

import numpy as np
import rospy
import rosparam
import smach

from geometry_msgs.msg import Pose
from sensor_msgs import point_cloud2
from gazebo_msgs.srv import SpawnModel
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2
from lasr_vision_msgs.srv import YoloDetectionRequest

class ObjectDetected():
    def __init__(self, cords, xywh):
        self.center_of_mass = None
        self.object_seg_cords = cords
        self.xywh = xywh
        self.boarder_cords = []

'''
source trust me bro, i know what im doing
'''
class AddToCostMap(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.robot = default
        self.counter = 0

    def execute(self, userdata):
        rospy.logwarn('Adding object to costmap')

        result, pcl = self.get_list_of_objects_in_view()
        self.convert_detected_object_to_map_coords(result, pcl)
        self.calculate_boarder_cords(result)
        self.add_objects_as_VO(result)

        for obj in result:
            x, y, z = obj.center_of_mass
            self.sapwn_model(x, y, z)

        rospy.logwarn('detected objects in view: {}'.format(len(result)))
        return "succeeded"
        
    '''from the center of mass and the xywh cords, calculate the 4 boarder cords of the object'''
    def calculate_boarder_cords(self, objects):
        for obj in objects:
            x, y, _ = obj.center_of_mass
            _, _, w, h = obj.xywh

            w = self.pixel_to_meter(w)
            h = self.pixel_to_meter(h)
            obj.boarder_cords = [(x - w/2, y - h/2), (x + w/2, y - h/2), (x - w/2, y + h/2), (x + w/2, y + h/2)]

    def pixel_to_meter(self, pixel_length):
        # 1 pixel = 0.0002645833 meters
        return pixel_length * 0.0002645833

    def sapwn_model(self, x, y, z):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

            # Define the SDF of the box
            box_sdf = f"""
            <?xml version="1.0"?>
            <sdf version="1.4">
            <model name="my_box">
                <pose>{x} {y} 0 0 0</pose>

                <static>true</static>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                        <box>
                            <size>0.5 0.5 0.1</size>
                        </box>
                        </geometry>
                    </visual>
                    
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>0.5 0.5 0.1</size>
                            </box>
                        </geometry>
                    </collision>
                </link>
            </model>
            </sdf>
            """

            # Define the pose of the box
            box_pose = Pose()
            box_pose.position.x = x
            box_pose.position.y = y
            box_pose.position.z = z

            # Call the service to spawn the box
            response = spawn_model(f'my_box_{self.counter}', box_sdf, '', box_pose, 'world')
            self.counter += 1

            rospy.loginfo("Box spawned successfully: %s", response.status_message)
            return response.status_message

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def add_objects_as_VO(self, objects):
        if rosparam.list_params("/mmap"):
            rosparam.delete_param("mmap")

        mmap_dict = {"vo": {"submap_0": dict()}, "numberOfSubMaps" : 1}

        count = 0
        for obj in objects:
            obj_counter = 0
            for cord in obj.boarder_cords:
                vo = f"vo_00{count}"
                mmap_dict["vo"]["submap_0"][vo] = ["submap_0", f"obstacle_{obj_counter}", *cord, 0.0]
                count +=1

            obj_counter += 1

        rospy.logwarn("added objects to mmap")
        rosparam.upload_params("mmap", mmap_dict)
        rospy.logwarn("published new map")

    def add_objects_as_row_major_order(self, objects):
        msg = rospy.wait_for_message('/move_base/local_costmap/costmap', OccupancyGrid)
        
        for obj in objects:
            for cord in obj.major_order_cords:
                if 0 <= cord < len(msg.data):
                    msg.data[cord] = 99

        # self.local_costmap_pub.publish(msg)

    def get_index(self, map_width, map_height, x, y):
        adjusted_x = x + map_width
        adjusted_y = y + map_height
        return adjusted_y * map_width + adjusted_x

    def get_list_of_objects_in_view(self):
        result = []

        self.robot.controllers.head_controller.look_down()
        msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

        request = YoloDetectionRequest()
        request.image_raw = msg                # sensor_msgs/Image
        request.dataset = "yolov8n-seg.pt"     # YOLOv8 model, auto-downloads
        request.confidence = 0.09               # minimum confidence to include in results
        request.nms = 0.4                      # non maximal supression

        # send request
        response = self.robot.detect_service(request)
        for detection in response.detected_objects:
            # cords of an object in image
            print("detected a: ", detection.name)
            result.append(ObjectDetected(detection.xyseg, detection.xywh))

        self.robot.controllers.head_controller.look_straight()
        return (result, pcl)
    
    def convert_detected_object_to_map_coords(self, objects_detected, pcl):
        for object_detected in objects_detected:
            cords = self.convert_seg_to_xy_cords(object_detected.object_seg_cords, pcl)
            (x, y, _) = self.robot.translate_coord_to_map(cords, pcl.header)    
            object_detected.center_of_mass = (x, y, 0.06)

    def convert_seg_to_xy_cords(self, seg_cords, pcl):
        try:
            depth_data = point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True)
            depth_array = np.array(list(depth_data))

            # Extract depth values for the coordinates of the person
            depth_values_x = []
            depth_values_y = []
            depth_values_z = []
            for cord in seg_cords:
                x, y, z = depth_array[cord]
                depth_values_x.append(x)
                depth_values_y.append(y)
                depth_values_z.append(z)

            # Calculate the average depth
            average_depth_x = np.mean(depth_values_x)
            average_depth_y = np.mean(depth_values_y)
            average_depth_z = np.mean(depth_values_z)

            return (average_depth_x, average_depth_y, average_depth_z)
        
        except Exception as e:
            rospy.logerr("Error in estimating depth: {}".format(str(e)))

    def convert_map_coords_to_row_major_order(self, objects, width):
        for obj in objects:
            result = []
            for cord in obj.object_map_cords:
                x, y, z = cord
        
                # x + width*y
                converted_row_major_order = y * width + x
        
                # add a buffer of +/- 10 indexes to the object
                for i in range(-11, 11):
                    result.append(converted_row_major_order + i)
        
            obj.major_order_cords = result