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
    {   if(!pnh_.getParam("layout/x", layout_x_) || layout_x_ <=0)
        {
            throw std::runtime_error("Cannot process layout parameter for object: " + id_ + ", it should contain x parameter (positive integer), check object configuration yaml");
        }
        if(!pnh_.getParam("layout/y", layout_y_) || layout_y_ <=0)
        {
            throw std::runtime_error("Cannot process layout parameter for object: " + id_ + ", it should contain y parameter (positive integer), check object configuration yaml");
        }
        if(!pnh_.getParam("layout/z", layout_z_) || layout_z_ <=0)
        {
            throw std::runtime_error("Cannot process layout parameter for object: " + id_ + ", it should contain z parameter (positive integer), check object configuration yaml");
        }
    }

    // Process parameter pose and check position and orientation values are the desired type and size
    if (pose_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_.size() == 2)
    {    
        std::vector<double> pose_position;
        std::vector<double> pose_orientation;

        if (pose_[0].getType() == XmlRpc::XmlRpcValue::TypeArray && pose_[0].size() == 3)
        { 
            for( int i = 0; i < pose_[0].size(); i++ )
            {   
                if(pose_[0][i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    pose_position.push_back((double)((int) pose_[0][i]));     
                }else if(pose_[0][i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    pose_position.push_back( pose_[0][i] );
                }else{
                    throw std::runtime_error("Cannot process pose position parameter for object: " + id_ + ", it should contain [x,y,z] int or double array, check object configuration yaml");                 
                }
            }
        }else{
            throw std::runtime_error("Cannot process pose position parameter for object: " + id_ + ", it should contain [x,y,z] vector, check object configuration yaml");
        }   

        if (pose_[1].getType() == XmlRpc::XmlRpcValue::TypeArray && pose_[1].size() == 3)
        { 
            for( int i = 0; i < pose_[1].size(); i++ )
            {
                if(pose_[1][i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                { 
                    pose_orientation.push_back((double)((int) pose_[1][i]));     
                }else if(pose_[1][i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                { 
                    pose_orientation.push_back( pose_[1][i] );
                }else{
                    throw std::runtime_error("Cannot process pose orientation parameter for object: " + id_ + ", it should contain [r,p,y] int or double array, check object configuration yaml");   
                } 
            }
        }else{
            throw std::runtime_error("Cannot process pose orientation parameter for object: " + id_ + ", it should contain [r,p,y] vector, check object configuration yaml");    
        }  

        // Fill in geometry_msgs::Pose message with processed yaml pose data
        pose_msg.position.x = (pose_position[0]);
        pose_msg.position.y = (pose_position[1]);
        pose_msg.position.z = (pose_position[2]); 

        tf2::Quaternion quat_orientation;
        quat_orientation.setRPY( pose_orientation[0], pose_orientation[1], pose_orientation[2]);

        pose_msg.orientation.x = quat_orientation[0];
        pose_msg.orientation.y = quat_orientation[1];
        pose_msg.orientation.z = quat_orientation[2];     
        pose_msg.orientation.w = quat_orientation[3];   

    }else{
        throw std::runtime_error("Cannot process pose parameter for object: " + id_ + ", it should contain [[x,y,z],[r,p,y]] int or double list, check object configuration yaml");
    }    
    

    // Fill in parent moveit collision object basic parameters
    parent_collision_object_.id = id_;
    parent_collision_object_.header.frame_id = frame_id_;
    parent_collision_object_.pose = pose_msg;      

    // Construct identity pose message
    geometry_msgs::Pose default_pose_msg = geometry_msgs::Pose();
    default_pose_msg.orientation.w = 1;

    // Process parameter geometry and check if is of type mesh or type primitive
    if ( geometry_.hasMember("mesh")){

        std::string mesh_path_; 
        pnh_.getParam("geometry/mesh", mesh_path_ );

        shape_msgs::Mesh mesh;

        shapes::Mesh* m = shapes::createMeshFromResource(mesh_path_); 
        shapes::ShapeMsg mesh_msg;  
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        // Fill in moveit collision object geometry
        parent_collision_object_.meshes.push_back(mesh);
        parent_collision_object_.mesh_poses.push_back(default_pose_msg);

    }else if (geometry_.hasMember("box")){
        double length_, width_, height_;
        
        shape_msgs::SolidPrimitive primitive; 

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
      
        if(pnh_.getParam("geometry/box/length", length_ )){
            primitive.dimensions[0] = length_;
        }else{
            throw std::runtime_error("Cannot process geometry/box parameter for object: " + id_ + ", it should contain length parameter, check object configuration yaml");
        }
        if(pnh_.getParam("geometry/box/width", width_ )){
            primitive.dimensions[1] = width_;
        }else{
            throw std::runtime_error("Cannot process geometry/box parameter for object: " + id_ + ", it should contain width parameter, check object configuration yaml");
        }
        if(pnh_.getParam("geometry/box/height", height_ )){
            primitive.dimensions[2] = height_;
            parent_collision_object_.pose.position.z += height_/2;
        }else{
            throw std::runtime_error("Cannot process geometry/box parameter for object: " + id_ + ", it should contain height parameter, check object configuration yaml");
        }
    
        // Fill in moveit collision object geometry
        parent_collision_object_.primitives.push_back(primitive);
        parent_collision_object_.primitive_poses.push_back(default_pose_msg);

        // Crate subframe at top surface
        parent_collision_object_.subframe_names.resize(1);
        parent_collision_object_.subframe_poses.resize(1);
        parent_collision_object_.subframe_names[0] = "top";
        parent_collision_object_.subframe_poses[0].position.z = height_/2;
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, 0);  
        parent_collision_object_.subframe_poses[0].orientation = tf2::toMsg(orientation);

    }else{
        ROS_WARN("Cannot process geometry parameter, check object configuration yaml");
    }


    // Build matrix of objects if required 
    if(matrix_)
    { 
      // Declare needed variables
      int child_id_ = 1;
      geometry_msgs::Pose local_child_pose_,global_child_pose_;
      double matrix_base_length_, matrix_base_width_, matrix_base_height_;
      int crates_floor_;

      // Create temporary collision object and initialize with parent info
      moveit_msgs::CollisionObject child_collision_object_;
      child_collision_object_ = parent_collision_object_;
      
      // Matrix configuration
      crates_floor_ = layout_x_*layout_y_;

      //  De momento solo para primitives 
      double object_length_ = parent_collision_object_.primitives[0].dimensions[0];
      double object_width_ = parent_collision_object_.primitives[0].dimensions[1];
      double object_height_ = parent_collision_object_.primitives[0].dimensions[2];

      matrix_base_length_ = object_length_*layout_x_; 
      matrix_base_width_ = object_width_*layout_y_;  
      //matrix_base_height_ = parent_collision_object_.pose.position.z; 
  
      for(int z = 0; z <= (layout_z_-1)*crates_floor_; z+=crates_floor_){
        local_child_pose_.position.z = object_height_*z/crates_floor_;
        for(int x = 0; x<layout_x_; x++)
        {
          for(int y = 0; y<layout_y_; y++)
          {
          // Child collision object pose with respect to parent collision object    
          child_collision_object_.id  = parent_collision_object_.id + "_" + std::to_string(child_id_);
          local_child_pose_.position.x =  -matrix_base_length_/2 + object_length_/2 + x*object_length_;
          local_child_pose_.position.y = -matrix_base_width_/2 + object_width_/2 + y*object_width_;
          local_child_pose_.orientation.w = 1;
          
          // Construct transformation matrices 
          tf2::Transform toParentPose, toGlobalPose;
          tf2::fromMsg(local_child_pose_, toParentPose);
          tf2::fromMsg(parent_collision_object_.pose, toGlobalPose);  
          
          // Compute global_child_pose_ by multipying by transformation matrices (from local pose -> parent pose -> global pose = toGlobalPose*toParentPose)   
          tf2::toMsg(toGlobalPose*toParentPose, global_child_pose_);

          // Fill child collision object with computed global child pose
          child_collision_object_.pose = global_child_pose_;  

          // Push back collision object into vector
          collision_objects_.push_back(child_collision_object_);

          child_id_ += 1;
          }
        }
      }   
    }else{ 
        collision_objects_.push_back(parent_collision_object_);
    }
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
