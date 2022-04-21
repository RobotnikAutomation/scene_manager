#include <scene_manager/scene_manager.h>

SceneManager::SceneManager(ros::NodeHandle nh, bool wait) : PlanningSceneInterface(nh.getNamespace(), wait)
{   
    ROS_INFO_STREAM("Instantiating SceneManager");

    // Get parameters from parameter server
    nh_ = ros::NodeHandle(nh, "scene_manager");
    pnh_ = ros::NodeHandle("~");
    
    // Service
    add_objects_srv = nh_.advertiseService("add_objects", &SceneManager::addObjectsCB,this);
    remove_objects_srv = nh_.advertiseService("remove_objects", &SceneManager::removeObjectsCB,this);
    attach_objects_srv = nh_.advertiseService("attach_objects", &SceneManager::attachObjectsCB,this);
    detach_objects_srv = nh_.advertiseService("detach_objects", &SceneManager::detachObjectsCB,this);

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
    try 
    {
      parsed_scene_objects_.insert(make_pair(object_name, Object_Builder(ros::NodeHandle(pnh_ , object_name), object_name)));  
    } catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("Exception: " << e.what());
    }
  }

/*   for (auto const & object_name: scene_objects_names_)
  {
    collision_object_map_.insert(make_pair(object_name, Object_Builder(ros::NodeHandle(pnh_ , object_name), object_name).getObjects()));  
  } */

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
/*   if(object_names.empty()){
    object_names = static_objects_names_;
  } */

  for (auto object_name: object_names){
    try{
      // Check if object to add is available in node database and proceed with spawn operation 
      std::vector<moveit_msgs::CollisionObject> collision_object = parsed_scene_objects_.at(object_name).getObjects();
      // Check if object frame exists
      tfBuffer_->lookupTransform(robot_base_link_,collision_object[0].header.frame_id,ros::Time(0),ros::Duration(1.0));
      for (auto object: collision_object){
        object.operation = moveit_msgs::CollisionObject::ADD;
        collision_objects.push_back(object);
        ROS_INFO("Adding object: %s", object.id.c_str());
      }  
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
/*   if(object_names.empty()){
    object_names = getKnownObjectNames();
  } */

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

bool SceneManager::addObjectsCB(scene_manager_msgs::ModifyObjects::Request &req, scene_manager_msgs::ModifyObjects::Response &res)
{
 if(req.names.empty() && (req.all == true)){
  res.result = addObjects(static_objects_names_); 
 }else{ 
  res.result = addObjects(req.names);
 } 

 if(!res.result){throw std::runtime_error("Could not load all desired objects to planning scene.");}
 return res.result;
}

bool SceneManager::removeObjectsCB(scene_manager_msgs::ModifyObjects::Request &req, scene_manager_msgs::ModifyObjects::Response &res)
{
 if(req.names.empty() && (req.all == true)){
  res.result = removeObjects(getKnownObjectNames()); 
 }else{ 
  res.result = removeObjects(req.names);
 } 

 if(!res.result){throw std::runtime_error("Could not remove all desired objects to planning scene.");}
 return res.result;
}

bool SceneManager::attachObjectsCB(scene_manager_msgs::ModifyObjects::Request &req, scene_manager_msgs::ModifyObjects::Response &res)
{
 res.result = attachObjects(req.names);
 if(!res.result){throw std::runtime_error("Could not attach all desired objects.");}
 return res.result;
}

bool SceneManager::detachObjectsCB(scene_manager_msgs::ModifyObjects::Request &req, scene_manager_msgs::ModifyObjects::Response &res)
{
 res.result = detachObjects(req.names);
 if(!res.result){throw std::runtime_error("Could not detach all desired objects.");}
 return res.result;
}