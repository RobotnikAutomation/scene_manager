#ifndef _COROSECT_MOVEIT_CONFIG__SCENE_MANAGER_H_
#define _COROSECT_MOVEIT_CONFIG__SCENE_MANAGER_H_

//ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>

//VISUALIZATION
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

class SceneManager : public moveit::planning_interface::PlanningSceneInterface
{
  public:
    
    SceneManager(ros::NodeHandle nh, bool wait);
    virtual ~SceneManager();
    void loadSceneYaml();
    bool initScene();
    bool addObjects(std::vector<std::string> object_names);
    bool removeObjects(std::vector<std::string> object_names);
    bool attachObjects(std::vector<std::string> object_names);
    bool detachObjects(std::vector<std::string> object_names);

    // ROS Services
    ros::ServiceServer add_objects_srv;
    ros::ServiceServer remove_objects_srv;
    ros::ServiceServer attach_objects_srv;
    ros::ServiceServer detach_objects_srv;
    ros::ServiceServer move_to_srv;
    ros::ServiceServer modify_object_srv;
    bool addObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res);
    bool removeObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res);
    bool attachObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res);
    bool detachObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res);
    bool moveToCB(scene_manager_msgs::MoveTo::Request &req, scene_manager_msgs::MoveTo::Response &res);
    bool modifyObjectCB(scene_manager_msgs::ModifyObject::Request &req, scene_manager_msgs::ModifyObject::Response &res);
    
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
    std::string robot_base_link_; // Robot base link name  
    std::string robot_eef_link_; // Robot end-effector link name  
    std::string group_name_; // MoveIt robot group name
    double move_group_timeout_; // MoveIt connection timeout
    std::vector<std::string> static_objects_names_; // List of names of static objects in scene
    std::vector<std::string> spawn_objects_names_; // List of names of objects to spawn in scene during init
    std::vector<std::string> scene_objects_names_; // List of names of all objects available in scene. 

    // For visualizing things in rviz
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    ros::Timer frame_publisher_timer_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    
    // Moveit Initialization 
    //ros::WallDuration move_group_timeout_ = ros::WallDuration(20);
    std::shared_ptr<tf2_ros::Buffer> move_group_tf2_buffer_; 
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
};

#endif // _COROSECT_MOVEIT_CONFIG__SCENE_MANAGER_H_