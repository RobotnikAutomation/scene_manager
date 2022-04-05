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
#include <scene_manager_msgs/ModifyObjects.h>

//MOVEIT
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

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
    bool addObjectsCB(scene_manager_msgs::ModifyObjects::Request &req, scene_manager_msgs::ModifyObjects::Response &res);
    bool removeObjectsCB(scene_manager_msgs::ModifyObjects::Request &req, scene_manager_msgs::ModifyObjects::Response &res);
    bool attachObjectsCB(scene_manager_msgs::ModifyObjects::Request &req, scene_manager_msgs::ModifyObjects::Response &res);
    bool detachObjectsCB(scene_manager_msgs::ModifyObjects::Request &req, scene_manager_msgs::ModifyObjects::Response &res);
    
  protected: 
  
    std::map<std::string, Object_Builder> parsed_scene_objects_; // map containing object id and object's custom object_builder class object
  
  private:

    ros::NodeHandle nh_; // node handle
    ros::NodeHandle pnh_; // private node handle
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr}; // tf listener
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_; // tf buffer
    std::string robot_base_link_; // Robot base link name  
    std::string robot_eef_link_; // Robot end-effector link name  
    std::vector<std::string> static_objects_names_; // List of names of static objects in scene
    std::vector<std::string> spawn_objects_names_; // List of names of objects to spawn in scene during init
    std::vector<std::string> scene_objects_names_; // List of names of all objects available in scene. 
};

#endif // _COROSECT_MOVEIT_CONFIG__SCENE_MANAGER_H_