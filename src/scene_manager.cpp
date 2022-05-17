#include <scene_manager/scene_manager.h>

SceneManager::SceneManager(ros::NodeHandle nh, bool wait) : PlanningSceneInterface(nh.getNamespace(), wait)
{   
    ROS_INFO_STREAM("Instantiating SceneManager");

    // Get parameters from parameter server
    nh_ = ros::NodeHandle(nh, "scene_manager");
    pnh_ = ros::NodeHandle("~");

    // Read and store robot parameters from parameter server
    pnh_.param<std::string>("robot_base_link", robot_base_link_, robot_base_link_);
    pnh_.param<std::string>("end_effector_link", robot_eef_link_, robot_eef_link_);
    pnh_.param<std::string>("group_name", group_name_, group_name_);
    pnh_.param<double>("move_group_timeout", move_group_timeout_, move_group_timeout_);
    
    // Service
    add_objects_srv = nh_.advertiseService("add_objects", &SceneManager::addObjectsCB,this);
    remove_objects_srv = nh_.advertiseService("remove_objects", &SceneManager::removeObjectsCB,this);
    attach_objects_srv = nh_.advertiseService("attach_objects", &SceneManager::attachObjectsCB,this);
    detach_objects_srv = nh_.advertiseService("detach_objects", &SceneManager::detachObjectsCB,this);
    move_relative_to_srv = nh_.advertiseService("move_relative_to", &SceneManager::moveRelativeToCB,this);
    modify_object_srv = nh_.advertiseService("modify_object", &SceneManager::modifyObjectCB,this);

    // Visualization timer
    frame_publisher_timer_ = pnh_.createTimer(ros::Duration(1), std::bind(&SceneManager::frameTimerCB, this));
    
    // Visualization publisher
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world","/moveit_visual_markers"));

    visual_tools_->deleteAllMarkers();

    // Initialize tf listener and buffer ros objects
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    //Reset planning scene monitor
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));      
    
    // update the planning scene monitor with the current state
    bool success = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
    ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));

    // keep up to date with new changes
    planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    
    // Moveit Initialization 
    move_group_tf2_buffer_.reset(new tf2_ros::Buffer);

    try
    {
      move_group_.reset(
          new moveit::planning_interface::MoveGroupInterface("arm", move_group_tf2_buffer_, ros::WallDuration(move_group_timeout_)));
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR("Cannot create move group with group name: arm. Is MoveIt running? Group name is correct?");
      ROS_INFO_STREAM("Exception: " << e.what());
    }

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
    try 
    {
      collision_object_map_.insert(make_pair(object_name, Object_Builder(ros::NodeHandle(pnh_ , object_name), object_name).getObjects())); 
    } catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("Exception: " << e.what());
    } 
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
      try{
        std::vector<moveit_msgs::CollisionObject> sub_collision_objects = parsed_scene_objects_.at(object_name).getObjects();
        for (auto collision_object: sub_collision_objects){
          collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
          collision_objects.push_back(collision_object);
          ROS_INFO("Removing object: %s", collision_object.id.c_str());
        }  
      }catch (const std::out_of_range& e){
        result = false;
        ROS_WARN("The object: %s is not spawned in scene, cannot be removed.", object_name.c_str());
      }    
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
      ROS_ERROR("Error when attaching object: %s. Object not available in scene, cannot attach.", object_name.c_str());
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
      ROS_ERROR("Error when detaching object: %s. Object not attached, cannot detach.", object_name.c_str());
    }
  }

  return applyAttachedCollisionObjects(attached_objects); 
}

bool SceneManager::moveRelativeTo(std::string object_id, geometry_msgs::Pose rel_pose)
{
  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);
  
  move_group_->setEndEffectorLink("link_6");

  geometry_msgs::PoseStamped pose;
  Eigen::Isometry3d tf;
  tf2::fromMsg(rel_pose, tf);
  /* pose.pose = tf2::toMsg(planning_scene->getFrameTransform((object_id + "/top")) * tf); */
  pose.pose = tf2::toMsg(planning_scene->getFrameTransform((object_id)) * tf);
  pose.header.frame_id = planning_scene->getPlanningFrame();
  pose.header.stamp = ros::Time::now();

  move_group_->clearPoseTargets();
  
  ROS_INFO_STREAM("set goal pose : " << move_group_->setJointValueTarget(pose));
  ROS_INFO_STREAM("goal pose: " << pose);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  ROS_INFO_STREAM("move relative to action executing");
  if (move_group_->plan(myplan) && move_group_->execute(myplan)){
    ROS_INFO_STREAM("move relative to action: " << true);
    return true;
  }else{
    ROS_INFO_STREAM("move relative to action: " << false);
    return false;
  }
}

bool SceneManager::addObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res)
{
 if(req.names.empty() && (req.all == true)){
  res.result = addObjects(static_objects_names_); 
 }else{ 
  res.result = addObjects(req.names);
 } 

 if(!res.result){throw std::runtime_error("Could not load all desired objects to planning scene.");}
 return res.result;
}

bool SceneManager::removeObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res)
{
 if(req.names.empty() && (req.all == true)){
  res.result = removeObjects(getKnownObjectNames()); 
 }else{ 
  res.result = removeObjects(req.names);
 } 

 if(!res.result){throw std::runtime_error("Could not remove all desired objects from planning scene.");}
 return res.result;
}

bool SceneManager::attachObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res)
{
 res.result = attachObjects(req.names);
 if(!res.result){throw std::runtime_error("Could not attach all desired objects.");}
 return res.result;
}

bool SceneManager::detachObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res)
{
 res.result = detachObjects(req.names);
 if(!res.result){throw std::runtime_error("Could not detach all desired objects.");}
 return res.result;
}

bool SceneManager::modifyObjectCB(scene_manager_msgs::ModifyObject::Request &req, scene_manager_msgs::ModifyObject::Response &res)
{
   // Store collision objects currently in scene
  std::map< std::string,moveit_msgs::CollisionObject > current_objects_ =  getObjects();

  std::vector <moveit_msgs::CollisionObject> collision_objects;
  geometry_msgs::Pose default_pose_msg = geometry_msgs::Pose();
  scene_manager_msgs::Layout default_layout_msg = scene_manager_msgs::Layout();

  try{
    moveit_msgs::CollisionObject collision_object = current_objects_.at(req.object_id);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;
    if(req.pose != default_pose_msg)
    {
      collision_object.pose = req.pose;
      collision_object.pose.position.z += collision_object.primitives[0].dimensions[2]/2;
    }
    collision_objects.push_back(collision_object);
    ROS_INFO("Modifying object: %s", req.object_id.c_str());
  }catch (const std::out_of_range& e){
    try{

      /* Object_Builder parsed_object = parsed_scene_objects_.at(req.object_id); */
      if(req.pose != default_pose_msg)
      {
        parsed_scene_objects_.at(req.object_id).setPose(req.pose);
      }

      if(req.layout != default_layout_msg)
      { 
        removeObjects({req.object_id});
        parsed_scene_objects_.at(req.object_id).setLayout(req.layout.x, req.layout.y, req.layout.z );
      }
      parsed_scene_objects_.at(req.object_id).buildObjects();

      for (auto collision_object: parsed_scene_objects_.at(req.object_id).getObjects()){
        collision_object.operation = moveit_msgs::CollisionObject::ADD;
        collision_objects.push_back(collision_object);
        ROS_INFO("Modifying object: %s", collision_object.id.c_str());
      }  
    }catch (const std::out_of_range& e){
/*       result = false; */
      ROS_WARN("The object: %s is not spawned in scene, cannot be modified.", req.object_id.c_str());
    }    
  }  

  applyCollisionObjects(collision_objects); 
  return true;
}

bool SceneManager::moveRelativeToCB(scene_manager_msgs::MoveTo::Request &req, scene_manager_msgs::MoveTo::Response &res)
{
 res.result = moveRelativeTo(req.object, req.pose);
 res.result = true;
 ROS_INFO_STREAM("move relative to action: " << res.result);
 if(!res.result){throw std::runtime_error("Could not move to goal position relative to desired object");}
 return res.result;
}

void SceneManager::frameTimerCB()
{ 
  visual_tools_->deleteAllMarkers();
  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

  std::map< std::string,moveit_msgs::CollisionObject > current_objects =  getObjects();
  for (auto & [id, current_object] : current_objects)
  {
    if(planning_scene->knowsFrameTransform((current_object.id + "/top")))
    {
      /* visual_tools_->publishAxisLabeled(planning_scene->getFrameTransform((current_object.id + "/top")), id, rviz_visual_tools::LARGE  ); */
      visual_tools_->publishAxisLabeled(planning_scene->getFrameTransform((current_object.id)), id, rviz_visual_tools::LARGE  );
    }else
    {
      visual_tools_->publishAxisLabeled(planning_scene->getFrameTransform(current_object.id), id, rviz_visual_tools::LARGE  );
    }
  }
  visual_tools_->trigger();

}

