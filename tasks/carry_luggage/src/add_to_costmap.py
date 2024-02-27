#!/usr/bin/env python3

import numpy as np
import rospy
import rosparam
import yaml

from costmap_converter.msg import ObstacleArrayMsg
from costmap_converter.msg import ObstacleMsg
from costmap_2d.cfg import Costmap2DConfig
from costmap_2d.msg import VoxelGrid

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2, LaserScan, PointCloud, ChannelFloat32
from sensor_msgs import point_cloud2
from lasr_vision_msgs.srv import YoloDetectionRequest
from Default import Default

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point32, Quaternion, Vector3, Pose

from visualization_msgs.msg import Marker, MarkerArray
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker, InteractiveMarkerUpdate
from interactive_markers.menu_handler import MenuHandler


# from arm_navigation_msgs.msg import CollisionObject
# march 18th draft of thesis

'''
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

        # self.pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
        # self.local_costmap = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.local_costmap_callback)
        # self.laser_scan = rospy.Subscriber('scan', LaserScan, self.add_object_to_map_3)
        # self.pub_scan = rospy.Publisher('own_scan', LaserScan, queue_size=10)
        # rosparam.set_param("/move_base/local_costmap/obstacle_laser_layer/base_scan/topic", "own_scan")
        # self.local_costmap_pub = rospy.Publisher('/move_base/local_costmap/costmap', OccupancyGrid, queue_size=1)
        # self.real_map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        # self.global_costmap = rospy.Publisher('/move_base/global_costmap/costmap', OccupancyGrid, queue_size=1)
        # self.point_cloud_pub = rospy.Publisher('/xtion/depth_registered/points', PointCloud2, queue_size=1)
        # self.map_pub = rospy.Publisher('/move_base/global_costmap/realmap_cost', OccupancyGrid, queue_size=1)
        # self.local_costmap_pub = rospy.Publisher('/move_base/local_costmap/costmap', VoxelGrid, queue_size=1)
        # self.test_new_map()
        # self.add_object_to_map_2()
        # self.vo_pub = rospy.Publisher('vo_cloud', PointCloud, queue_size=1)
        # self.marker_pub = rospy.Publisher('/marker_server/update', InteractiveMarker, queue_size=10)

        self.vo_pub = rospy.Publisher('vo_map', OccupancyGrid, queue_size=10)

    def add_object_to_map_5(self):
        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = rospy.Time.now()
        occ_grid.header.frame_id = "map"
        occ_grid.info.resolution = 0.05
        occ_grid.info.width = 100
        occ_grid.info.height = 100
        occ_grid.info.origin.position.x = -5
        occ_grid.info.origin.position.y = -5
        occ_grid.info.origin.position.z = 0
        occ_grid.info.origin.orientation.x = 0
        occ_grid.info.origin.orientation.y = 0
        occ_grid.info.origin.orientation.z = 0
        occ_grid.info.origin.orientation.w = 1
        occ_grid.data = [99 if i <= 260 else 0 for i in range(10000)]

        rospy.logwarn("published new map")
        self.vo_pub.publish(occ_grid)

    def make_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.63
        marker.pose.position.y = -2
        marker.pose.position.z = 0.06
        marker.pose.orientation.w = 1
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

        return marker
    
    def add_marker(self):
        # server = InteractiveMarkerServer("marker_server")

        # Create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.pose.position.x = 1.63
        int_marker.pose.position.y = -2
        int_marker.pose.position.z = 0
        int_marker.pose.orientation.w = 1
        int_marker.scale = 1

        int_marker_update = InteractiveMarkerUpdate()
        int_marker_update.server_id = "marker_server"
        int_marker_update.seq_num = 0
        int_marker_update.type = InteractiveMarkerUpdate.UPDATE
        int_marker_update.markers.append(int_marker)


        # Create a marker and add it to the interactive marker
        # marker = self.make_marker()
        # int_marker.controls.append(make_box_control())

        # Add the interactive marker to the server
        # server.insert(int_marker, None)
        # server.applyChanges()

        self.marker_pub.publish(int_marker_update)

    def vo_pubisher(self):
        
        msg = PointCloud()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        dense_point_cloud = []
        # Define range and density
        x_min, x_max = -0.5, 0.0
        y_min, y_max = -1.0, -0.5
        step_size = 0.1

        # Generate points
        x = x_min
        while x <= x_max:
            y = y_min
            while y <= y_max:
                point = Point32()
                point.x = x
                point.y = y
                point.z = 0.06
                dense_point_cloud.append(point)
                y += step_size
            x += step_size

        msg.points = dense_point_cloud
        # msg.channels = []

        while True:
            self.vo_pub.publish(msg)
            # rospy.logwarn("published new map")

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

    def add_object_to_map_4(self):
        obstcl = ObstacleMsg()
        obstcl.header.stamp = rospy.Time.now()
        obstcl.header.frame_id = "map"

        obstcl.polygon.points = [Point32(-5, -5, 0), Point32(-5, 5, 0), Point32(5, 5, 0), Point32(5, -5, 0)]
        obstcl.radius = 2.0
        obstcl.id = 0
        obstcl.orientation = Quaternion(0, 0, 0, 1)
        # obstcl.velocities = Vector3(0, 0, 0)

        obstcl_array = ObstacleArrayMsg()
        obstcl_array.header.stamp = rospy.Time.now()
        obstcl_array.header.frame_id = "map"
        obstcl_array.obstacles = [obstcl]

        self.pub.publish(obstcl_array)
        rospy.logwarn("published new map")

    def add_object_to_map_3(self, msg):
        print("got a laser scan")
        msg.ranges = [1.0 for _ in range(len(msg.ranges))]
        self.pub_scan.publish(msg)

    def add_object_to_map_2(self):
        
        voxel_grid = VoxelGrid()
        voxel_grid.header.stamp = rospy.Time.now()
        voxel_grid.header.frame_id = "map"
        voxel_grid.origin.x = Point32(-5, -5, 0)
        voxel_grid.resolutions = Vector3(0.05, 0.05, 0.05)
        voxel_grid.size_x = 100
        voxel_grid.size_y = 100
        voxel_grid.size_z = 1
        voxel_grid.data = [99 if i <= 260 else 0 for i in range(10000)]

        self.local_costmap_pub.publish(voxel_grid)
        rospy.logwarn("published new map")
        # rosparam.upload_params("costmap", costmap)
        
    def test_new_map(self):
        costmap = [99 if i <= 260 else 0 for i in range(10000)]

        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = 0.05
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = -5
        msg.info.origin.position.y = -5
        msg.data = costmap

        self.map_pub.publish(msg)

        rospy.logwarn("published new map")

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
    for i in range(100000):
        costmap.add_object_to_map_5()
    # costmap.sapwn_model()
    # while True:
        # costmap.add_marker()
        # costmap.vo_pubisher()
    # rospy.spin()