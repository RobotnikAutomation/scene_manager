#ifndef _COROSECT_MOVEIT_CONFIG__OBJECT_BUILDER_H_
#define _COROSECT_MOVEIT_CONFIG__OBJECT_BUILDER_H_

#include <ros/ros.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <iostream>
#include <string>

class Object_Builder
{
  public:
    
    Object_Builder(ros::NodeHandle pnh, std::string id);
    virtual ~Object_Builder();
    moveit_msgs::CollisionObject getObject();
    bool getSpawn();
    bool getStatic();
    std::string getID();

  protected:

    
  private:

    // Variables which hold config parameter yaml info
    std::string id_; // Object id
    bool spawn_;
    bool static_;
    std::string frame_id_; // Frame used for object relative positioning
    XmlRpc::XmlRpcValue geometry_; // Stores object geometry parameter
    XmlRpc::XmlRpcValue pose_; // Stores object pose parameter
    moveit_msgs::CollisionObject collision_object_;
    
    // Variables created in Object class after processing parameter info 
    geometry_msgs::Pose pose_msg; // MoveIt Object Pose message
    shape_msgs::Mesh mesh; // Will be filled in only for mesh objects
    shape_msgs::SolidPrimitive primitive; // Will be filled in only for primitive shape objects
    ros::NodeHandle pnh_; // object paramenter node handle

};

#endif // _COROSECT_MOVEIT_CONFIG__OBJECT_BUILDER_H_
