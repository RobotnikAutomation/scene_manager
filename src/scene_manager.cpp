#include <scene_manager/scene_manager.h>

SceneManager::SceneManager(ros::NodeHandle nh, bool wait) : PlanningSceneInterface(nh.getNamespace(), wait)
{   
    ROS_INFO_STREAM("Instantiating SceneManager");

    // Get parameters from parameter server
    nh_ = 
    pnh_ = ros::NodeHandle("~");
    
    // Initialize tf listener and buffer ros objects
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    
    // Read and store robot parameters from parameter server
    pnh_.param<std::string>("robot_base_link", robot_base_link_, robot_base_link_);
    pnh_.param<std::string>("end_effector_link", robot_eef_link_, robot_eef_link_);
    // Read and store scene description
    loadSceneYaml();
}

SceneManager::~SceneManager(){
    
};

void SceneManager::loadSceneYaml()
{ 

  // Read and store parameter objects from parameter server
  pnh_.param<std::vector<std::string>>("objects", scene_objects_names_, scene_objects_names_);

  // Create class Object_Builder objects
  for (auto const & object_name: scene_objects_names_)
  {
    parsed_scene_objects_.insert(make_pair(object_name, Object_Builder(ros::NodeHandle(pnh_ , object_name), object_name)));  
  }

  // Sort scene objects into vectors based on their type : spawn and static properties
  for (auto & [id, parsed_object] : parsed_scene_objects_)
  { 
    if(parsed_object.getSpawn()){
      spawn_objects_names_.push_back(id);
    }

    if(parsed_object.getStatic()){
      static_objects_names_.push_back(id);
    }
  }

}

bool SceneManager::initScene()
{ 
  return addObjects(spawn_objects_names_);
}

bool SceneManager::addObjects(std::vector<std::string> object_names)
{ 
  // Result
  bool result = true; 

  // Initialize vector of collision objects to load into scene
  std::vector <moveit_msgs::CollisionObject> collision_objects;

  // If input vector is empty add all available static objects to scene
  if(object_names.empty()){
    object_names = static_objects_names_;
  }

  for (auto object_name: object_names){
    try{
      // Check if object to add is available in node database and proceed with spawn operation 
      moveit_msgs::CollisionObject collision_object = parsed_scene_objects_.at(object_name).getObject();
      // Check if object frame exists
      tfBuffer_->lookupTransform(robot_base_link_,collision_object.header.frame_id,ros::Time(0),ros::Duration(1.0));
      collision_object.operation = moveit_msgs::CollisionObject::ADD;
      collision_objects.push_back(collision_object);
      ROS_INFO("Adding object: %s", object_name.c_str());
    }
    catch (const std::out_of_range& e)
    {
      result = false;
      ROS_ERROR("The object: %s does not exist in scene description yaml, cannot be added to scene.", object_name.c_str());
    }
    catch(tf2::TransformException ex)
    {
      result = false;
      ROS_ERROR("Error when adding %s object to desired frame. Lookup Transform error: %s", object_name.c_str(), ex.what());
    }   

  }
  
  // Modify objects in the scene
  return result && applyCollisionObjects(collision_objects);
}

bool SceneManager::removeObjects(std::vector<std::string> object_names)
{ 
  // Result
  bool result = true;

  // Initialize vector of collision objects to load into scene
  std::vector <moveit_msgs::CollisionObject> collision_objects;

  // If input vector is empty remove all objects in scene
  if(object_names.empty()){
    object_names = getKnownObjectNames();
  }

  // Store collision objects currently in scene
  std::map< std::string,moveit_msgs::CollisionObject > current_objects_ =  getObjects();

  for (auto object_name: object_names){
    // Check if object to remove is spawned in scene and proceed with removal operation 
    try{
      moveit_msgs::CollisionObject collision_object = current_objects_.at(object_name);
      collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
      collision_objects.push_back(collision_object);
      ROS_INFO("Removing object: %s", object_name.c_str());
    }catch (const std::out_of_range& e){
      result = false;
      ROS_ERROR("The object: %s is not spawned in scene, cannot be removed.", object_name.c_str());
    }  
  }

  // Modify objects in the scene
  return result && applyCollisionObjects(collision_objects);
}



bool SceneManager::attachObjects(std::vector<std::string> object_names)
{
  // Result
  bool result = true;
  // List collision objects in scene
  std::map< std::string,moveit_msgs::CollisionObject > current_objects_ = getObjects();
  std::vector <moveit_msgs::AttachedCollisionObject> attached_objects;

  for (auto object_name: object_names){
    // Check if object to attach is in scene and proceed with attaching
    try{
      moveit_msgs::CollisionObject collision_object = current_objects_.at(object_name);
      moveit_msgs::AttachedCollisionObject attached_collision_object;
      attached_collision_object.object = collision_object;
      attached_collision_object.link_name = robot_eef_link_;
      attached_collision_object.object.operation = moveit_msgs::CollisionObject::ADD;
      attached_objects.push_back(attached_collision_object);
      ROS_INFO("Attaching object: %s", object_name.c_str());
    }catch (const std::out_of_range& e){
      result = false;
      ROS_ERROR("Error when attaching object: %s . Object not available in scene, cannot attach.", object_name.c_str());
    }
  }

  return result && applyAttachedCollisionObjects(attached_objects);
}

bool SceneManager::detachObjects(std::vector<std::string> object_names)
{
  // Result
  bool result = true;
  // List attached objects in scene
  std::map< std::string,moveit_msgs::AttachedCollisionObject > current_attached_objects_ =  getAttachedObjects();
  std::vector <moveit_msgs::AttachedCollisionObject> attached_objects;

  for (auto object_name: object_names){
    // Check if object to modify is attached and proceed with detaching
    try{
      moveit_msgs::AttachedCollisionObject attached_collision_object = current_attached_objects_.at(object_name);
      attached_collision_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
      attached_objects.push_back(attached_collision_object);
      ROS_INFO("Detaching object: %s", object_name.c_str());
    }catch (const std::out_of_range& e)
    { 
      result = false;
      ROS_ERROR("Error when detaching object: %s . Object not attached, cannot detach.", object_name.c_str());
    }
  }

  return result && applyAttachedCollisionObjects(attached_objects); 
}