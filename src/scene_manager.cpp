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
    pnh_.param<bool>("publish_tf", publish_tf_, publish_tf_);    
    pnh_.param<std::string>("host", host_, host_);
    pnh_.param<int>("port", port_, port_);

    // Service
    add_objects_srv = nh_.advertiseService("add_objects", &SceneManager::addObjectsCB,this);
    remove_objects_srv = nh_.advertiseService("remove_objects", &SceneManager::removeObjectsCB,this);
    attach_objects_srv = nh_.advertiseService("attach_objects", &SceneManager::attachObjectsCB,this);
    detach_objects_srv = nh_.advertiseService("detach_objects", &SceneManager::detachObjectsCB,this);
    move_relative_to_srv = nh_.advertiseService("move_relative_to", &SceneManager::moveRelativeToCB,this);
    modify_object_srv = nh_.advertiseService("modify_object", &SceneManager::modifyObjectCB,this);
    save_scene_srv = nh_.advertiseService("save_scene", &SceneManager::saveSceneCB,this);
    load_scene_srv = nh_.advertiseService("load_scene", &SceneManager::loadSceneCB,this);
    
    // Service client
    planning_scene_diff_client_ = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client_.waitForExistence();



    // Initialize tf listener and buffer ros objects
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    //Reset planning scene monitor
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));      
    
    // update the planning scene monitor with the current state
    bool success = planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
    ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));

    // keep up to date with new changes
    planning_scene_monitor_->startSceneMonitor("move_group/monitored_planning_scene");
    
    // Moveit Initialization 
    move_group_tf2_buffer_.reset(new tf2_ros::Buffer);
    

    try
    {
      move_group_.reset(
          new moveit::planning_interface::MoveGroupInterface(group_name_, move_group_tf2_buffer_, ros::WallDuration(move_group_timeout_)));
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR("Cannot create move group with group name: %s. Is MoveIt running? Group name is correct?", group_name_.c_str());
      ROS_INFO_STREAM("Exception: " << e.what());
    }

    // Retrieve moveit configuration
    robot_base_link_ = move_group_->getPlanningFrame();
    robot_eef_link_ = move_group_->getEndEffectorLink();


    // Visualization publisher
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_base_link_,"moveit_visual_markers"));

    visual_tools_->deleteAllMarkers();

    // Read and store scene description
    loadSceneYaml();

    // Connect to moveit's warehouse mongo db database
    conn_ = moveit_warehouse::loadDatabase();
    conn_->setParams(host_, port_);

    ROS_INFO("Connecting to warehouse on %s:%d", host_.c_str(), port_);

    while (!conn_->connect())
    {
      ROS_ERROR("Failed to connect to DB on %s:%d ", host_.c_str(), port_);
      ros::Duration(2).sleep();
      conn_->setParams(host_, port_);
    }

    scene_storage_.reset(new moveit_warehouse::PlanningSceneStorage(conn_));

    // Visualization timer
    frame_publisher_timer_ = pnh_.createTimer(ros::Duration(1), std::bind(&SceneManager::frameTimerCB, this));
}

SceneManager::~SceneManager(){
    
};

void SceneManager::loadSceneYaml()
{ 
  scene_objects_names_ = std::vector<std::string>();
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

bool SceneManager::addObjects(const std::vector<std::string>& object_names)
{ 
  // Result
  bool result = true; 

  // Initialize vector of collision objects to load into scene
  std::vector <moveit_msgs::CollisionObject> collision_objects;

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

bool SceneManager::removeObjects(const std::vector<std::string>& object_names)
{ 
  // Result
  bool result = true;

  // Initialize vector of collision objects to load into scene
  std::vector <moveit_msgs::CollisionObject> collision_objects;

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



bool SceneManager::attachObjects(const std::vector<std::string>& object_names)
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

bool SceneManager::detachObjects(const std::vector<std::string>& object_names)
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

bool SceneManager::moveRelativeTo(const std::string& object_id, const geometry_msgs::Pose& rel_pose)
{
  bool result;
  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);
  
  move_group_->setEndEffectorLink(robot_eef_link_);

  geometry_msgs::PoseStamped pose;
  Eigen::Isometry3d tf;
  tf2::fromMsg(rel_pose, tf);

  // Check if object exists
  if(planning_scene->knowsFrameTransform(object_id + "/center"))
  {
    /* pose.pose = tf2::toMsg(planning_scene->getFrameTransform(object_id + "/center") * tf); */
    pose.pose = tf2::toMsg(planning_scene->getFrameTransform((object_id)) * tf);
    
  }else if(planning_scene->knowsFrameTransform(object_id))
  {
    pose.pose = tf2::toMsg(planning_scene->getFrameTransform((object_id)) * tf);

  }else
  {
    ROS_WARN_STREAM("Object not present in planning scene, cannot move relative to object: " << object_id);
    return false;
  }

  pose.header.frame_id = planning_scene->getPlanningFrame();
  pose.header.stamp = ros::Time::now();

  // Normalize quaternion and check if is valid
  tf2::Quaternion quat_orientation;
  tf2::fromMsg(pose.pose.orientation, quat_orientation);
  quat_orientation.normalize();
  pose.pose.orientation = tf2::toMsg(quat_orientation);

  move_group_->clearPoseTargets();
  
  bool pose_check = move_group_->setJointValueTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  ROS_INFO_STREAM("move relative to action executing");
  if (move_group_->plan(myplan) && pose_check){
    if(move_group_->execute(myplan)){
      result = true;
    }else{
      result = false;
    }
  }else{
    ROS_INFO_STREAM("move relative to action: " << false);
    result = false;
  }
  return result;
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
 if(req.names.empty() && (req.all == true)){
  std::vector<std::string> objects;
  std::map< std::string,moveit_msgs::CollisionObject > objects_map = getObjects();
  for (auto & [id, object] : objects_map)
  { 
    objects.push_back(id);
  }  
  res.result = attachObjects(objects);
 }else{   
  res.result = attachObjects(req.names);
 }
 if(!res.result){throw std::runtime_error("Could not attach all desired objects.");}
 return res.result;
}

bool SceneManager::detachObjectsCB(scene_manager_msgs::SelectObjects::Request &req, scene_manager_msgs::SelectObjects::Response &res)
{
 if(req.names.empty() && (req.all == true)){
  std::vector<std::string> attached_objects;
  std::map< std::string,moveit_msgs::AttachedCollisionObject > attached_objects_map = getAttachedObjects();
  for (auto & [id, attached_object] : attached_objects_map)
  { 
    attached_objects.push_back(id);
  }  
  res.result = detachObjects(attached_objects);
 }else{   
  res.result = detachObjects(req.names);
 }
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
    
    if(req.pose.pose != default_pose_msg)
    {
      parsed_scene_objects_.at(req.object_id).setPose(req.pose.pose);
      parsed_scene_objects_.at(req.object_id).setFrame(req.pose.header.frame_id);
    }

    if(req.layout != default_layout_msg)
    { 
      removeObjects({req.object_id});
      parsed_scene_objects_.at(req.object_id).setLayout(req.layout.x, req.layout.y, req.layout.z );
    }

    parsed_scene_objects_.at(req.object_id).buildObjects();

    for (auto collision_object: parsed_scene_objects_.at(req.object_id).getObjects()){
      try{
        current_objects_.at(req.object_id);
        collision_object.operation = moveit_msgs::CollisionObject::MOVE;
      }catch (const std::out_of_range& e){
        collision_object.operation = moveit_msgs::CollisionObject::ADD;
      }  
      collision_objects.push_back(collision_object);
      ROS_INFO("Modifying object: %s", collision_object.id.c_str());
    } 

  }catch (const std::out_of_range& e){

    try{
      moveit_msgs::CollisionObject collision_object = current_objects_.at(req.object_id);
      collision_object.operation = moveit_msgs::CollisionObject::MOVE;
      if(req.pose.pose != default_pose_msg)
      {
        collision_object.pose = req.pose.pose;
        collision_object.header.frame_id = req.pose.header.frame_id;
      }
      collision_objects.push_back(collision_object);
      ROS_INFO("Modifying object: %s", req.object_id.c_str());
    }catch (const std::out_of_range& e){
      ROS_WARN("The object: %s is not spawned in scene, cannot be modified.", req.object_id.c_str());
      res.result = false;
      return true;
    }    

  }  

  res.result = applyCollisionObjects(collision_objects); 
  return true;
}

bool SceneManager::moveRelativeToCB(scene_manager_msgs::MoveTo::Request &req, scene_manager_msgs::MoveTo::Response &res)
{
 res.result = moveRelativeTo(req.object, req.pose);
 ROS_INFO_STREAM("move relative to action: " << res.result);
 if(!res.result){throw std::runtime_error("Could not move to goal position defined relative to desired object");}
 return true;
}

bool SceneManager::saveSceneCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
/*   res.success = scene_manager_->initScene();
  res.message = "Initializing planning scene"; */
  moveit_msgs::PlanningScene current_planning_scene_msg;
  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);
  planning_scene->getPlanningSceneMsg(current_planning_scene_msg);
  current_planning_scene_msg.name = "saved_scene";
  res.message = "Stored planning scene";
  res.success = true;
  scene_storage_->addPlanningScene(current_planning_scene_msg);
  return true;
}

bool SceneManager::loadSceneCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
/*   res.success = scene_manager_->initScene();
  res.message = "Initializing planning scene"; */
  moveit_warehouse::PlanningSceneWithMetadata stored_planning_scene_data;
  moveit_msgs::PlanningScene stored_planning_scene_msg;
  std::string scene_name = "saved_scene";
  scene_storage_->getPlanningScene (stored_planning_scene_data, scene_name);
  stored_planning_scene_msg = *stored_planning_scene_data;
  res.message = "Loading stored planning scene";
  res.success = true;
  stored_planning_scene_msg.is_diff = true;
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = stored_planning_scene_msg;
  planning_scene_diff_client_.call(srv);
  return true;
}

bool SceneManager::allowCollision(const std::string& name, const std::vector< std::string >& other_names)
{
  moveit_msgs::PlanningScene current_planning_scene_msg;
  collision_detection::AllowedCollisionMatrix acm_;

  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);
 
  planning_scene->getPlanningSceneMsg(current_planning_scene_msg);
  
  acm_ = planning_scene->getAllowedCollisionMatrix();
  acm_.setEntry(name, other_names, true);
  acm_.getMessage(current_planning_scene_msg.allowed_collision_matrix);

  current_planning_scene_msg.is_diff = true;

  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = current_planning_scene_msg;
  planning_scene_diff_client_.call(srv);

  //applyPlanningScene(current_planning_scene_msg);

  return true;
}

bool SceneManager::restoreCollision(const std::string& name, const std::vector< std::string >& other_names)
{
  moveit_msgs::PlanningScene current_planning_scene_msg;
  collision_detection::AllowedCollisionMatrix acm_;

  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);
 
  planning_scene->getPlanningSceneMsg(current_planning_scene_msg);
  
  acm_ = planning_scene->getAllowedCollisionMatrix();
  acm_.setEntry(name, other_names, false);
  acm_.getMessage(current_planning_scene_msg.allowed_collision_matrix);

  current_planning_scene_msg.is_diff = true;
  applyPlanningScene(current_planning_scene_msg);

  return true;
}


void SceneManager::frameTimerCB()
{ 
  visual_tools_->deleteAllMarkers();
  planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);  

  std::map< std::string,moveit_msgs::CollisionObject > current_objects =  getObjects();
  Eigen::Isometry3d object_transform;
  for (auto & [id, current_object] : current_objects)
  {
    object_transform= planning_scene->getFrameTransform(current_object.id);
    visual_tools_->publishAxisLabeled(planning_scene->getFrameTransform(current_object.id), id, rviz_visual_tools::LARGE);
  

    visual_tools_->publishAxisLabeled(object_transform, id, rviz_visual_tools::LARGE);
    
    if(publish_tf_){
      geometry_msgs::TransformStamped object_transform_msg = tf2::eigenToTransform(object_transform);
      object_transform_msg.header.frame_id = robot_base_link_;
      object_transform_msg.header.stamp = ros::Time::now();
      object_transform_msg.child_frame_id = current_object.id; 
      tf_broadcaster_.sendTransform(object_transform_msg);
    }
  }
  visual_tools_->trigger();

}

moveit_msgs::CollisionObject SceneManager::getObjectMsg(const std::string& object_id)
{
  std::map< std::string, moveit_msgs::CollisionObject > object_map = 	getObjects({object_id});
  return object_map.at(object_id);
}

std::vector<double> SceneManager::getObjectSize(const std::string& object_id)
{
  std::vector<double> dimensions;

  try{
    dimensions = parsed_scene_objects_.at(object_id).getSize();
  }catch (const std::out_of_range& e){
    ROS_WARN("The object: %s is not created using Scene Manager, cannot retrieve size.", object_id.c_str());
  }    

  return dimensions;
}

geometry_msgs::PoseStamped SceneManager::getObjectPose(const std::string& object_id)
{
  geometry_msgs::PoseStamped parent_pose;

  try{
    parent_pose = parsed_scene_objects_.at(object_id).getPose();
  }catch (const std::out_of_range& e){
    ROS_WARN("The object: %s is not created using Scene Manager, cannot retrieve parent pose.", object_id.c_str());
  }    

  return parent_pose;
}

bool SceneManager::paddObject(const std::string& object_name, const double& padding)
{
  // Result
  bool result = true; 

  std::map< std::string,moveit_msgs::AttachedCollisionObject > current_attached_objects_ =  getAttachedObjects();
  
  try
  {
    // Check if object to add is available in node database and proceed with spawn operation 
    moveit_msgs::AttachedCollisionObject collision_object = current_attached_objects_.at(object_name);
    // Padd object
    for(int i = 0; i < collision_object.object.meshes.size(); i++){
      auto mesh = shapes::constructShapeFromMsg(collision_object.object.meshes[i]);
      mesh->padd(padding);
      shape_msgs::Mesh padded_mesh;
      shapes::ShapeMsg mesh_msg;  
      shapes::constructMsgFromShape(mesh, mesh_msg);
      padded_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
      // Fill in moveit collision object geometry
      collision_object.object.meshes[i] = padded_mesh;
    }
    for(int i = 0; i < collision_object.object.primitives.size(); i++){
      auto shape = shapes::constructShapeFromMsg(collision_object.object.primitives[i]);
      shape->padd(padding);
      shapes::ShapeMsg shape_msg;  
      shapes::constructMsgFromShape(shape, shape_msg);
      shape_msgs::SolidPrimitive padded_shape;
      padded_shape = boost::get<shape_msgs::SolidPrimitive>(shape_msg);
      collision_object.object.primitives[i] = padded_shape;
    }
    
    // Modify objects in the scene
    return result && applyAttachedCollisionObject(collision_object);
  }
  catch (const std::out_of_range& e)
  {

  }
  // Initialize vector of collision objects to load into scene
  std::vector <moveit_msgs::CollisionObject> collision_objects;

  // Retrieve objects in scene
  std::map< std::string,moveit_msgs::CollisionObject > current_objects_ = getObjects();

  // Check if object to padd is in scene and proceed with padding
  try{
    // Check if object to add is available in node database and proceed with spawn operation 
    moveit_msgs::CollisionObject collision_object = current_objects_.at(object_name);

    // Padd object
    for(int i = 0; i < collision_object.meshes.size(); i++){
      auto mesh = shapes::constructShapeFromMsg(collision_object.meshes[i]);
      mesh->padd(padding);
      shape_msgs::Mesh padded_mesh;
      shapes::ShapeMsg mesh_msg;  
      shapes::constructMsgFromShape(mesh, mesh_msg);
      padded_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
      // Fill in moveit collision object geometry
      collision_object.meshes[i] = padded_mesh;
    }
    for(int i = 0; i < collision_object.primitives.size(); i++){
      auto shape = shapes::constructShapeFromMsg(collision_object.primitives[i]);
      shape->padd(padding);
      shapes::ShapeMsg shape_msg;  
      shapes::constructMsgFromShape(shape, shape_msg);
      shape_msgs::SolidPrimitive padded_shape;
      padded_shape = boost::get<shape_msgs::SolidPrimitive>(shape_msg);
      collision_object.primitives[i] = padded_shape;
    }
    // Check if object frame exists
    tfBuffer_->lookupTransform(robot_base_link_,collision_object.header.frame_id,ros::Time(0),ros::Duration(1.0));
    // Add collision object
    collision_object.operation = moveit_msgs::CollisionObject::ADD;
    collision_objects.push_back(collision_object);
    ROS_INFO("Adding object: %s", collision_object.id.c_str());
  }
  catch (const std::out_of_range& e)
  {
    result = false;
    ROS_ERROR("The object: %s does not exist in Planning scene, cannot be padded.", object_name.c_str());
  }
  catch(tf2::TransformException ex)
  {
    result = false;
    ROS_ERROR("Error when adding %s object to desired frame. Lookup Transform error: %s", object_name.c_str(), ex.what());
  }   

  // Modify objects in the scene
  return result && applyCollisionObjects(collision_objects);
}

