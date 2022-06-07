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

    clearObjects();

    // Fill in parent moveit collision object basic parameters
    parent_collision_object_.id = id_;
    parent_collision_object_.header.frame_id = frame_id_;

    layout_x_ = 0;
    layout_y_ = 0;
    layout_z_ = 0;

    if(matrix_)
    {   if(!pnh_.getParam("layout/x", layout_x_) || layout_x_ < 1)
        {
            throw std::runtime_error("Cannot process layout parameter for object: " + id_ + ", it should contain x parameter (positive integer different from zero), check object configuration yaml");
        }
        if(!pnh_.getParam("layout/y", layout_y_) || layout_y_ < 1)
        {
            throw std::runtime_error("Cannot process layout parameter for object: " + id_ + ", it should contain y parameter (positive integer different from zero), check object configuration yaml");
        }
        if(!pnh_.getParam("layout/z", layout_z_) || layout_z_ < 1)
        {
            throw std::runtime_error("Cannot process layout parameter for object: " + id_ + ", it should contain z parameter (positive integer different from zero), check object configuration yaml");
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

    // Construct identity pose message
    geometry_msgs::Pose default_pose_msg = geometry_msgs::Pose();
    default_pose_msg.orientation.w = 1;

    // Process parameter geometry and check if is of type mesh or type primitive
    if ( geometry_.hasMember("mesh")){

        std::string mesh_path_; 
        pnh_.getParam("geometry/mesh/path", mesh_path_ );

        shape_msgs::Mesh mesh;

        shapes::Mesh* m = shapes::createMeshFromResource(mesh_path_); 

        shapes::ShapeMsg mesh_msg;  
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        if(!pnh_.getParam("geometry/mesh/size/length", length_ )){
            throw std::runtime_error("Cannot process geometry/mesh/size parameter for object: " + id_ + ", it should contain length parameter, check object configuration yaml");
        }
        if(!pnh_.getParam("geometry/mesh/size/width", width_ )){
            throw std::runtime_error("Cannot process geometry/mesh/size parameter for object: " + id_ + ", it should contain width parameter, check object configuration yaml");
        }
        if(!pnh_.getParam("geometry/mesh/size/height", height_ )){
            throw std::runtime_error("Cannot process geometry/mesh/size parameter for object: " + id_ + ", it should contain height parameter, check object configuration yaml");
        }

        // Fill in moveit collision object geometry
        parent_collision_object_.meshes.push_back(mesh);
        default_pose_msg.position.z = height_/2;
        parent_collision_object_.mesh_poses.push_back(default_pose_msg);

    }else if (geometry_.hasMember("box")){
        
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
        }else{
            throw std::runtime_error("Cannot process geometry/box parameter for object: " + id_ + ", it should contain height parameter, check object configuration yaml");
        }
    
        // Fill in moveit collision object geometry
        parent_collision_object_.primitives.push_back(primitive);
        default_pose_msg.position.z = height_/2;
        parent_collision_object_.primitive_poses.push_back(default_pose_msg);

    }else{
        ROS_WARN("Cannot process geometry parameter, check object configuration yaml");
    }

    // Crate subframe at center of object
    parent_collision_object_.subframe_names.resize(1);
    parent_collision_object_.subframe_poses.resize(1);
    parent_collision_object_.subframe_names[0] = "center";
    parent_collision_object_.subframe_poses[0].position.z = height_/2;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);  
    parent_collision_object_.subframe_poses[0].orientation = tf2::toMsg(orientation);

    // Set Pose 
    Object_Builder::setPose(pose_msg);
    if(matrix_)
    {
        Object_Builder::setLayout(layout_x_, layout_y_, layout_z_);
    }
    Object_Builder::buildObjects();

}

Object_Builder::~Object_Builder(){
    
};

void Object_Builder::setLayout(const int& layout_x, const int& layout_y, const int& layout_z)
{   
    if (layout_x <=0 || layout_y <=0 || layout_z <=0){
        ROS_ERROR_STREAM("Cannot process layout parameter for object: " + id_ + ", it should contain positive integers different from zero");
    }else{
        this->layout_x_ = layout_x;
        this->layout_y_ = layout_y;
        this->layout_z_ = layout_z;
    }
}



void Object_Builder::setPose(const geometry_msgs::Pose& pose)
{   
    parent_collision_object_.pose = pose;
}

void Object_Builder::setFrame(const std::string& frame)
{   
    parent_collision_object_.header.frame_id = frame;
}

void Object_Builder::clearObjects(){
    collision_objects_.clear();
}

void Object_Builder::buildObjects()
{

    if(layout_x_!=0 && layout_y_!=0 && layout_z_!=0 ){

        // Clear objects
        Object_Builder::clearObjects();
        // Declare needed variables
        int child_id_ = 1;
        geometry_msgs::Pose local_child_pose_,global_child_pose_;
        double matrix_base_length_, matrix_base_width_, matrix_base_height_;
        int crates_floor_;

        // Create temporary collision object and initialize with parent info
        moveit_msgs::CollisionObject child_collision_object_;
        child_collision_object_ = parent_collision_object_;

        // Padd object    
        for(int i = 0; i < parent_collision_object_.meshes.size(); i++)
        {
          auto mesh = shapes::constructShapeFromMsg(parent_collision_object_.meshes[i]);
          mesh->padd(-0.01);
          shape_msgs::Mesh padded_mesh;
          shapes::ShapeMsg mesh_msg;  
          shapes::constructMsgFromShape(mesh, mesh_msg);
          padded_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
          // Fill in moveit collision object geometry
          child_collision_object_.meshes[i] = padded_mesh;
        }

        for(int i = 0; i < parent_collision_object_.primitives.size(); i++)
        {
          auto shape = shapes::constructShapeFromMsg(parent_collision_object_.primitives[i]);
          shape->padd(-0.01);
          shapes::ShapeMsg shape_msg;  
          shapes::constructMsgFromShape(shape, shape_msg);
          shape_msgs::SolidPrimitive padded_shape;
          padded_shape = boost::get<shape_msgs::SolidPrimitive>(shape_msg);
          child_collision_object_.primitives[i] = padded_shape;
        }
        
        // Matrix configuration
        crates_floor_ = layout_x_*layout_y_;

        double object_length_ = length_;
        double object_width_ = width_;
        double object_height_ = height_;

        matrix_base_length_ = object_length_*layout_x_; 
        matrix_base_width_ = object_width_*layout_y_;  

        for(int z = 0; z <= (layout_z_-1)*crates_floor_; z+=crates_floor_)
        {
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
    }    

    if(collision_objects_.size() <=1)
    {   
        Object_Builder::clearObjects();
        collision_objects_.push_back(parent_collision_object_);
    }
}


std::vector<moveit_msgs::CollisionObject> Object_Builder::getObjects(){
    return collision_objects_;
}

std::vector<double> Object_Builder::getSize(){
    return {length_, width_, height_};
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
