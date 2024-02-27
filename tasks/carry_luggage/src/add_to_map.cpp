#include <ros/ros.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

// make a function to add the object to the planning scene
void addObject(arm_navigation_msgs::GetPlanningScene::Request &planning_scene_req, int x, int y, int z){
    arm_navigation_msgs::CollisionObject object;
    
    object.id = "pole";
    object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    object.header.frame_id = "odom_combined";
    object.header.stamp = ros::Time::now();
    arm_navigation_msgs::Shape object;
    
    object.type = arm_navigation_msgs::Shape::CYLINDER;
    
    object.dimensions.resize(2);
    object.dimensions[0] = .1;
    object.dimensions[1] = 1.0;

    // set the pose of the object
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    
    object.shapes.push_back(object);
    object.poses.push_back(pose);
    
    planning_scene_req.planning_scene_diff.collision_objects.push_back(object);
}

int main(int argc, char **argv){
  ros::init (argc, argv, "get_state_validity_test");
  ros::NodeHandle rh;

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;

  vis_marker_publisher_ = rh.advertise<visualization_msgs::Marker>("state_validity_markers", 128);
  vis_marker_array_publisher_ = rh.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);

  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ros::ServiceClient get_planning_scene_client = rh.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

  arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
  arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

  if(argc > 1) {

    std::stringstream s(argv[1]);
    bool add;
    s >> add;

    if(add) {
      //add the cylinder into the collision space
      addObject(planning_scene_req);
    }
  }

  if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene");
    return -1;
  }

  planning_environment::CollisionModels collision_models("robot_description");
  planning_models::KinematicState* state = 
    collision_models.setPlanningScene(planning_scene_res.planning_scene);

  std::vector<std::string> arm_names = 
    collision_models.getKinematicModel()->getModelGroup("right_arm")->getUpdatedLinkModelNames();
  std::vector<std::string> joint_names = 
    collision_models.getKinematicModel()->getModelGroup("right_arm")->getJointModelNames();

  std::map<std::string, double> nvalues;
  nvalues["r_shoulder_lift_joint"] = .9;

  if(argc > 2) {
    std::stringstream s(argv[2]);
    double val;
    s >> val;
    nvalues["r_shoulder_pan_joint"] = val;
  }
  state->setKinematicState(nvalues);

  
  std_msgs::ColorRGBA good_color, collision_color, joint_limits_color;
  good_color.a = collision_color.a = joint_limits_color.a = .8;

  good_color.g = 1.0;
  collision_color.r = 1.0;
  joint_limits_color.b = 1.0;
  
  std_msgs::ColorRGBA point_markers;
  point_markers.a = 1.0;
  point_markers.r = 1.0;
  point_markers.g = .8;

  std_msgs::ColorRGBA color;
  visualization_msgs::MarkerArray arr;
  if(!state->areJointsWithinBounds(joint_names)) {
    color = joint_limits_color;
  } else if(collision_models.isKinematicStateInCollision(*state)) {
    color = collision_color;
    collision_models.getAllCollisionPointMarkers(*state,
                                                 arr,
                                                 point_markers,
                                                 ros::Duration(0.2));
  } else {
    color = good_color;
  }

  collision_models.getRobotMarkersGivenState(*state,
                                             arr,
                                             color,
                                             "right_arm",
                                             ros::Duration(0.2),
                                             &arm_names);

  while(ros::ok()) {    
    vis_marker_array_publisher_.publish(arr);
    ros::spinOnce();
    ros::Duration(.1).sleep();
  }
  collision_models.revertPlanningScene(state);
  ros::shutdown();
}
