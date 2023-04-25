#ifndef _COROSECT_MOVEIT_CONFIG__OBJECT_BUILDER_H_
#define _COROSECT_MOVEIT_CONFIG__OBJECT_BUILDER_H_

#include <ros/ros.h>

#include <geometric_shapes/shapes.h>
#include <stdio.h>
#include <math.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <map>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Object_Builder
{
  public:
    
    Object_Builder(ros::NodeHandle pnh, std::string id);
    virtual ~Object_Builder();
    
    // Query object type
    bool getSpawn();  // Spawn objects are spawned during init
    bool getStatic(); // Static objects are spawned during init and are always in scene
    std::vector<double> getSize();
    // Query object identity
    std::string getID();
    // Query parent object pose
    geometry_msgs::PoseStamped getPose();

    // Set object configuration parameters
    void setPose(const geometry_msgs::Pose& pose);
    void setFrame(const std::string& frame);
    void setLayout(const int& layout_x, const int& layout_y, const int& layout_z);

    // Collision objects
    std::vector<moveit_msgs::CollisionObject> getObjects(); // Retrieve generated output collision objects
    void clearObjects(); // Remove generated collision objects from output variable
    void buildObjects(); // Load collision objects to ouput variable
  
  protected:
    
  private:

    // Global Variables which hold config parameter yaml info
    std::string id_; // Object id
    bool spawn_ = false;
    bool static_ = false;
    bool matrix_ = false;
    std::string frame_id_; // Frame used for object relative positioning
    XmlRpc::XmlRpcValue geometry_; // Stores object geometry parameter
    XmlRpc::XmlRpcValue pose_; // Stores object pose parameter
    int layout_x_ = 0, layout_y_ = 0, layout_z_ = 0;
    double length_, width_, height_;
  
    
    // Global Variables created in Object class after processing parameter info 
    moveit_msgs::CollisionObject parent_collision_object_;
    std::vector<moveit_msgs::CollisionObject> collision_objects_;
    geometry_msgs::Pose pose_msg; // MoveIt Object Pose message
    ros::NodeHandle pnh_; // object paramenter node handle

};

#endif // _COROSECT_MOVEIT_CONFIG__OBJECT_BUILDER_H_
