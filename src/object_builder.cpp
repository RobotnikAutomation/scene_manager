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

    matrix_ = pnh_.hasParam("layout");

    if(matrix_)
    {   
        pnh_.getParam("layout/x", layout_x_); 
        pnh_.getParam("layout/y", layout_y_); 
        pnh_.getParam("layout/z", layout_z_); 
        ROS_INFO_STREAM("parseando objecto: " << layout_x_ << " " << layout_y_ << " "<< layout_z_ );
    }

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

        mesh_ = true;
        std::string mesh_path_; 
        pnh_.getParam("geometry/mesh", mesh_path_ );

        shapes::Mesh* m = shapes::createMeshFromResource(mesh_path_); 
        shapes::ShapeMsg mesh_msg;  
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object_.meshes.push_back(mesh);
        collision_object_.mesh_poses.push_back(pose_msg);

    }else if (geometry_.hasMember("box")){
        primitive_ = true;

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

    // Build matrix of objects if required 

    if(matrix_)
    { 
      moveit_msgs::CollisionObject collision_object_add_;
      collision_object_add_ = collision_object_;
      // Matrix configuration
      crates_floor_ = layout_x_*layout_y_;
      int id = 1;
      geometry_msgs::Pose pose_;
      
      //  De momento solo para primitives 
      double object_length_ = collision_object_.primitives[0].dimensions[0];
      double object_width_ = collision_object_.primitives[0].dimensions[1];
      double object_height_ = collision_object_.primitives[0].dimensions[2];

      matrix_base_length_ = object_length_*layout_x_; 
      matrix_base_width_ = object_width_*layout_y_;  
      matrix_base_height_ = collision_object_.primitive_poses[0].position.z + object_height_/2; 

/*    if(mesh_){
        matrix_base_length_ = collision_object_.mesh_poses[0].position.x*layout_x_; 
        matrix_base_width_ = collision_object_.mesh_poses[0].position.y*layout_y_;  
        matrix_base_height_ = collision_object_.mesh_poses[0].position.z; 
      }
      if(primitive_){
        matrix_base_length_ = collision_object_.primitive_poses[0].position.x*layout_x_; 
        matrix_base_width_ = collision_object_.primitive_poses[0].position.y*layout_y_;  
        matrix_base_height_ = collision_object_.primitive_poses[0].position.z; 
      } */
  
      for(int z = 0; z <= (layout_z_-1)*crates_floor_; z+=crates_floor_){
        pose_.position.z = matrix_base_height_ + object_height_*z/crates_floor_;
        for(int x = 0; x<layout_x_; x++)
        {
          for(int y = 0; y<layout_y_; y++)
          {
          collision_object_add_.id  = collision_object_.id + "_" + std::to_string(id);
          pose_.position.x =  -matrix_base_length_/2 + object_length_/2 + x*object_length_;
          pose_.position.x += collision_object_.primitive_poses[0].position.x;
          pose_.position.y = -matrix_base_width_/2 + object_width_/2 + y*object_width_;
          pose_.position.y += collision_object_.primitive_poses[0].position.y;
          pose_.orientation = collision_object_.primitive_poses[0].orientation;
          if(mesh_){
            collision_object_add_.mesh_poses[0] = pose_;
          }
          if(primitive_){
            collision_object_add_.primitive_poses[0] = pose_;
          }
          collision_objects_.push_back(collision_object_add_);
          id += 1;
          }
        }
      }   
    }else{ collision_objects_.push_back(collision_object_);}
}

Object_Builder::~Object_Builder(){
    
};


std::vector<moveit_msgs::CollisionObject> Object_Builder::getObjects(){
    return collision_objects_;
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
