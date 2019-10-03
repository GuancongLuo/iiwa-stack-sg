/* 
owner:  Clarence , 2019.July 
project name:   MotionROS
use iiwa_ros to control iiwa
*/

#include "MotionROS.h"



MotionROS::MotionROS(const  ros::NodeHandle& nh_)
    :   nh(nh_)
    // ??? what is mean :nh(nh_)
{
    nh.param<std::string>("object_frame",object_frame,"object_frame");
    nh.param<std::string>("camera_frame",camera_frame,"camera_frame");
    nh.param<std::string>("camera_ref_frame",camera_ref_frame,"iiwa_link_ee");
    nh.param<std::string>("robot_base",robot_base,"iiwa_link_0");    

    // Eigen is C++ linear temper librery 
    Eigen::Matrix4Xd came_materx;
    came_materx << 1,0,0,0,
                   0.1,0,0,
                   0,0,1,0,
                   0,0,0,1;
    trans_camera_ef = Eigen::Affine3d::Identity();
    trans_camera_ef.matrix() = came_materx;

    // open iiwa subsribes and pulishers
    pose_state.init("iiwa");
    pose_command.init("iiwa");

    // wait a bit 
    ros::Duration(0.5).sleep();

    // services
    set_absolute_srv = nh.advertiseService("absolute_motion",&MotionROS::set_absolute_callback,this);
    set_relative_srv = nh.advertiseService("relative_motion",&MotionROS::set_relative_callback,this);
    ROS_INFO("Ready...");
}// end NodeExample() Here, node is MotionROS


/*--------------------------------------------------------------------
 * ~NodeExample()
 * Destructor.
 *------------------------------------------------------------------*/

MotionROS::~MotionROS()
{
    set_relative_srv.shutdown();
    set_absolute_srv.shutdown();
} // end ~NodeExample()


/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void MotionROS::publishMessage(ros::Publisher *pub_message)
{
  node_example::node_example_data msg;
  msg.message = message;
  msg.a = a;
  msg.b = b;

  pub_message->publish(msg);
} // end publishMessage()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

// publichCallback()
bool MotionROS::set_absolute_callback(motion_msgs::SetAbsolutePose::Request &request,
                                      motion_msgs::SetAbsolutePose::Response &response)
{
  std::string name = motion.motion_name;
  


  // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // Typically, they should be DEBUG.
  ROS_INFO("message is %s", message.c_str());
  ROS_INFO("sum of a + b = %d", a + b);
}

bool MotionROS::set_relative_callback(motion_msgs::SetRelativePose::Request& request,
                                      motion_msgs::SetRelativePose::Response& response)
{

} // end publishCallback()

// define privateCallback()
bool MotionROS::setAbsolutePose(motion_msgs::AbsoluteMotion& motion)
{

}

bool MotionROS::setRelativePose(motion_msgs::AbsoluteMotion &motion )
{

}

void printPoseStamp(geometry_msgs::PoseStamped& poseStamped)
{
    
}

static geometry_msgs::PoseStamped getPoseWrtFrame(ros::NodeHandle &node, const std::string child_frame, const std::string frame)
{

}

static geometry_msgs::PoseStamped transformPoseWrtFrame(ros::NodeHandle& node, const geometry_msgs::PoseStamped& pose, const std::string frame)
{
    
}// end privateCallbacK()



/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

void NodeExample::configCallback(node_example::node_example_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message = config.message.c_str();
  a = config.a;
  b = config.b;  

} // end configCallback()
