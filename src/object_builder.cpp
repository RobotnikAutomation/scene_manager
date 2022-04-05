#include <scene_manager/object_builder.h>

Object_Builder::Object_Builder(ros::NodeHandle pnh, std::string id)
{
    // Get parameters from parameter server

    pnh_ = pnh;
    id_ = id;
    pnh_.getParam("frame_id", frame_id_); 
    pnh_.getParam("pose", pose_); 
    pnh_.getParam("geometry", geometry_); 
    pnh_.getParam("spawn", spawn_); 
    pnh_.getParam("static", static_);

    // Fill in moveit collision object parameters
    collision_object_.id = id_;
    collision_object_.header.frame_id = frame_id_;


    // Process parameter pose

    if (pose_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_.size() == 2)
    {    
        XmlRpc::XmlRpcValue pose_position;
        XmlRpc::XmlRpcValue pose_orientation;
        tf2::Quaternion quaternion_orientation;

        pose_position = pose_[0];
        pose_orientation = pose_[1];

        if (pose_position.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_position.size() == 3){
           
            pose_msg.position.x = (pose_position[0]);
            pose_msg.position.y = (pose_position[1]);
            pose_msg.position.z = (pose_position[2]); 

        }else{
            ROS_WARN("Cannot process pose position parameter, it should contain [x,y,z] array, check object configuration yaml");
        }

        if (pose_orientation.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_orientation.size() == 3){

            quaternion_orientation.setRPY( pose_orientation[0], pose_orientation[1], pose_orientation[2]);
        
            pose_msg.orientation.x = quaternion_orientation[0];
            pose_msg.orientation.y = quaternion_orientation[1];
            pose_msg.orientation.z = quaternion_orientation[2];     
            pose_msg.orientation.w = quaternion_orientation[3];             
        }else{
            ROS_WARN("Cannot process pose orientation parameter, it should contain [r,p,y] array, check object configuration yaml");
        }

    }else{
        ROS_WARN("Cannot process pose parameter, it should contain [[x,y,z],[r,p,y]] list, check object configuration yaml");
    }    


    // Process parameter geometry and check if is of type mesh or type primitive

    if ( geometry_.hasMember("mesh")){


        std::string mesh_path_; 
        pnh_.getParam("geometry/mesh", mesh_path_ );

        shapes::Mesh* m = shapes::createMeshFromResource(mesh_path_); 
        shapes::ShapeMsg mesh_msg;  
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object_.meshes.push_back(mesh);
        collision_object_.mesh_poses.push_back(pose_msg);

    }else if (geometry_.hasMember("box")){
        

        XmlRpc::XmlRpcValue box_primitive_;
        pnh_.getParam("geometry/box", box_primitive_ );

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = box_primitive_["length"];
        primitive.dimensions[1] = box_primitive_["width"];
        primitive.dimensions[2] = box_primitive_["height"];

        collision_object_.primitives.push_back(primitive);
        collision_object_.primitive_poses.push_back(pose_msg);
    }else{
        ROS_WARN("Cannot process geometry parameter, check object configuration yaml");
    }

}

Object_Builder::~Object_Builder(){
    
};


moveit_msgs::CollisionObject Object_Builder::getObject(){
    return collision_object_;
}

bool Object_Builder::getSpawn(){
    return spawn_;
}

bool Object_Builder::getStatic(){
    return static_;
}

std::string Object_Builder::getID(){
    return id_;
}
