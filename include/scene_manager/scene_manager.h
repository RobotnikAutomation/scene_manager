#ifndef _COROSECT_MOVEIT_CONFIG__SCENE_MANAGER_H_
#define _COROSECT_MOVEIT_CONFIG__SCENE_MANAGER_H_

//ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>


//CPP
#include <string>
#include <map>
#include <iostream>


//Custom
#include <scene_manager/object_builder.h>
#include <scene_manager_msgs/SelectObjects.h>
#include <scene_manager_msgs/ModifyObject.h>
#include <scene_manager_msgs/MoveTo.h>
#include <scene_manager_msgs/Layout.h>


//MOVEIT
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

//MOVEIT PLANNING SCENE
#include <moveit_msgs/PlanningScene.h>

//MONGODB
#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/planning_scene_storage.h>

//VISUALIZATION
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>


class SceneManager : public moveit::planning_interface::PlanningSceneInterface
{
  public:
    
    SceneManager(ros::NodeHandle nh, bool wait);
    virtual ~SceneManager();
    void loadSceneYaml();

    // Functionalities
    bool initScene();
    bool addObjects(const std::vector<std::string>& object_names);
    bool removeObjects(const std::vector<std::string>& object_names);
    bool attachObjects(const std::vector<std::string>& object_names);
    bool detachObjects(const std::vector<std::string>& object_names);
    bool moveRelativeTo(const std::string& object_id, const geometry_msgs::Pose& rel_pose);
    bool allowCollision(const std::string& name, const std::vector< std::string >& other_names);
    bool restoreCollision(const std::string& name, const std::vector< std::string >& other_names);
    std::vector<double> getObjectSize(const std::string& object_id);
    geometry_msgs::PoseStamped getObjectPose(const std::string& object_id); // parent pose 
    moveit_msgs::CollisionObject getObjectMsg(const std::string& object_id);
    bool paddObject(const std::string& object_name, const double& padding);
   

    // ROS Services
    ros::ServiceServer add_objects_srv;
    ros::ServiceServer remove_objects_srv;
    ros::ServiceServer attach_objects_srv;
    ros::ServiceServer detach_objects_srv;
    ros::ServiceServer move_relative_to_srv;
    ros::ServiceServer modify_object_srv;
    ros::ServiceServer save_scene_srv;
    ros::ServiceServer load_scene_srv;
    bool addObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res);
    bool removeObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res);
    bool attachObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res);
    bool detachObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res);
    bool moveRelativeToCB(scene_manager_msgs::MoveTo::Request &req, scene_manager_msgs::MoveTo::Response &res);
    bool modifyObjectCB(scene_manager_msgs::ModifyObject::Request &req, scene_manager_msgs::ModifyObject::Response &res);
    bool saveSceneCB(std_srvs::Trigger::Request &req, std_srvs::TriggerResponse &res);
    bool loadSceneCB(std_srvs::Trigger::Request &req, std_srvs::TriggerResponse &res);

    // Service Client
    ros::ServiceClient planning_scene_diff_client_;
    
    // Frame Timer Callback
    void frameTimerCB();

  protected: 
  
    std::map<std::string, Object_Builder> parsed_scene_objects_; // map containing object id and object's custom object_builder class object
    std::map<std::string, std::vector<moveit_msgs::CollisionObject>> collision_object_map_;
  
  private:

    ros::NodeHandle nh_; // node handle
    ros::NodeHandle pnh_; // private node handle
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr}; // tf listener
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_; // tf buffer
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::string robot_base_link_; // Robot base link name  
    std::string robot_eef_link_; // Robot end-effector link name  
    std::string group_name_; // MoveIt robot group name
    double move_group_timeout_; // MoveIt connection timeout
    std::vector<std::string> static_objects_names_; // List of names of static objects in scene
    std::vector<std::string> spawn_objects_names_; // List of names of objects to spawn in scene during init
    std::vector<std::string> scene_objects_names_; // List of names of all objects available in scene. 
    bool publish_tf_; // Set to true to publish tf of objects in tf tree

    // For visualizing things in rviz
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    ros::Timer frame_publisher_timer_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    
    // Moveit Initialization 
    //ros::WallDuration move_group_timeout_ = ros::WallDuration(20);
    std::shared_ptr<tf2_ros::Buffer> move_group_tf2_buffer_; 
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;

    // MongoDB database
    warehouse_ros::DatabaseConnection::Ptr conn_;
    bool connect_db_;
    std::string host_;
    int port_;
    std::unique_ptr<moveit_warehouse::PlanningSceneStorage> scene_storage_;
};

#endif // _COROSECT_MOVEIT_CONFIG__SCENE_MANAGER_H_
