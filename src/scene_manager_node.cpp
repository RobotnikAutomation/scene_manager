#include <ros/ros.h>
#include <string>
#include "scene_manager/scene_manager.h"


// Define the main ROS node class
class SceneManagerNode
{
public:
    // Constructor
    SceneManagerNode()
    {
        // NodeHandle - private for reading parameters
        ros::NodeHandle nh;

        // Read parameters from parameter server
        nh.param<double>("update_rate", update_rate_, 1.0);

        // Create the SceneManager object
        scene_manager_ = std::make_unique<SceneManager>(nh , wait_);
        scene_manager_->initScene();
    }

private:
    double update_rate_;                           // Update rate parameter
    std::unique_ptr<SceneManager> scene_manager_;  // SceneManager instance
    bool wait_ = true; 
};

// Main function
int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "scene_manager_node");

    // Create the SceneManagerNode object
    SceneManagerNode node;

    // Spin method to run the node
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
