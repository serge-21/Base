#!/usr/bin/env python3

import numpy as np
import rospy
import rosparam
import yaml

from Default import Default
from geometry_msgs.msg import Pose
from sensor_msgs import point_cloud2
from gazebo_msgs.srv import SpawnModel
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2
from lasr_vision_msgs.srv import YoloDetectionRequest

'''
# from arm_navigation_msgs.msg import CollisionObject
# march 18th draft of thesis
try next:
vo_cloud
Type: sensor_msgs/PointCloud
'''

class ObjectDetected():
    def __init__(self, cords):
        self.object_seg_cords = cords
        self.object_map_cords = []
        self.major_order_cords = []

'''
source trust me bro, i know what im doing
'''
class AddToCostMap():
    def __init__(self, default):
        self.robot = default

    def sapwn_model(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

            # Define the SDF of the box
            box_sdf = """
            <?xml version="1.0"?>
            <sdf version="1.4">
            <model name="my_box">
                <pose>1.63 -2 0 0 0</pose>

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
            box_pose.position.x = 1.63
            box_pose.position.y = -2
            box_pose.position.z = 0.06

            # Call the service to spawn the box
            response = spawn_model('my_box_26', box_sdf, '', box_pose, 'world')
            rospy.loginfo("Box spawned successfully: %s", response.status_message)

            return response.status_message

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def add_object_to_map(self):
        rospy.logwarn("adding object to map")

        if rosparam.list_params("/mmap"):
            rosparam.delete_param("mmap")

        mmap_dict = {"vo": {"submap_0": dict()}, "numberOfSubMaps" : 1}

        # open yaml file
        with open("/home/serge/lasr_ws/src/laser-base/tasks/carry_luggage/config/vo.yaml", "r") as fp:
            data = yaml.safe_load(fp)

        rospy.logwarn("loaded yaml")

        count = 0
        for i, table in enumerate(data["tables"].keys()):
            for j, corner in enumerate(data["tables"][table]["objects_cuboid"]):
                vo = f"vo_00{count}"
                mmap_dict["vo"]["submap_0"][vo] = ["submap_0", f"table{i}", *corner, 0.0]
                count +=1

        for j, corner in enumerate(data["counter"]["cuboid"]):
            vo = f"vo_00{count}"
            mmap_dict["vo"]["submap_0"][vo] = ["submap_0", f"counter", *corner, 0.0]
            count +=1

        rospy.logwarn("added objects to mmap")
        rosparam.upload_params("mmap", mmap_dict)
        rospy.logwarn("published new map")

    def add_objects_as_VO(self, objects):
        if rosparam.list_params("/mmap"):
            rosparam.delete_param("mmap")

        mmap_dict = {"vo": {"submap_0": dict()}, "numberOfSubMaps" : 1}

        count = 0
        for obj in objects:
            for cord in obj.object_map_cords:
                vo = f"vo_00{count}"
                mmap_dict["vo"]["submap_0"][vo] = ["submap_0", "obstacle", *cord, 0.0]
                count +=1

        rospy.logwarn("added objects to mmap")
        rosparam.upload_params("mmap", mmap_dict)
        rospy.logwarn("published new map")

    def add_objects_as_row_major_order(self, objects):
        msg = rospy.wait_for_message('/move_base/local_costmap/costmap', OccupancyGrid)
        
        for obj in objects:
            for cord in obj.major_order_cords:
                if 0 <= cord < len(msg.data):
                    msg.data[cord] = 99

        # instead of publishing we need to configure a yaml file and upload it to the parameter server
        # still need to figure out how to do this
        self.local_costmap_pub.publish(msg)

    def local_costmap_callback(self, msg):
        _list, pcl = self.get_list_of_objects_in_view()

        for object_detected in _list:
            self.convert_detected_object_to_map_coords(object_detected, pcl)

        width = msg.info.width
        self.convert_map_coords_to_row_major_order(_list, width)

        # add all objects to the map as obstacles using VO's
        self.add_objects_as_VO(_list)
        
        # add all objects to teh map as obstacles using row major order
        self.add_objects_as_row_major_order(_list)

        # new_msg = OccupancyGrid()
        # new_msg.header = msg.header
        # new_msg.info = msg.info
        # new_msg.data = [99 if i <= 260 else msg.data[i] for i in range(len(msg.data))]
        # self.local_costmap_pub.publish(new_msg)

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
        request.confidence = 0.7               # minimum confidence to include in results
        request.nms = 0.4                      # non maximal supression

        # send request
        response = self.robot.detect_service(request)
        for detection in response.detected_objects:
            # cords of an object in image
            result.append(ObjectDetected(detection.xyseg))

        self.robot.controllers.head_controller.look_straight()
        return (result, pcl)
    
    def convert_detected_object_to_map_coords(self, object_detected, pcl):
        # first convert the seg cords to xy cords
        real_xy_coords = self.convert_seg_to_xy_cords(object_detected.object_seg_cords, pcl)
        
        # then convert the xy cords to map cords
        map_coords = []
        for xy_cords in real_xy_coords:
            map_coords.append(self.robot.translate_coord_to_map(xy_cords, pcl.header))
        
        object_detected.object_map_cords = map_coords

    def convert_seg_to_xy_cords(self, seg_cords, pcl):
        try:
            depth_data = point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True)
            depth_array = np.array(list(depth_data))

            converted_coords = []
            for cord in seg_cords:
                x, y, z = depth_array[cord]
                converted_coords.append((x, y, z))

            return converted_coords
        
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
    
    """
    notes:
    movebase publishes on /move_base/global_costmap/costmap & /move_base/local_costmap/costmap

    the type of the message is nav_msgs/OccupancyGrid
    msg definition
    
    # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    std_msgs/Header header

    # MetaData for the map
    nav_msgs/MapMetaData info

    # The map data, in row-major order, starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.
    int8[] data

    should look something like this:
    header:
        seq: 0
        stamp:
            secs: 820
            nsecs: 393000000
        frame_id: "odom"
    info:
        map_load_time:
            secs: 0
            nsecs:         0
        resolution: 0.02500000037252903
        width: 160
        height: 160
        origin:
            position:
                x: -2.0
                y: -1.975
                z: 0.0
            orientation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 1.0
    data: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ...]
    """

if __name__ == '__main__':
    rospy.init_node("costmap")
    costmap = AddToCostMap(Default())
    rospy.spin()