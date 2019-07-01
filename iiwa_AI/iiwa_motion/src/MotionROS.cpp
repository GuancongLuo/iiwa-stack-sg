/* ***
 * Cai, 26.06.2019
 */

#include "MotionROS.h"
//#define DEBUG_PRINT


MotionROS::MotionROS(const ros::NodeHandle& nh_)
    : nh(nh_)
{
    nh.param<std::string>("object_frame", object_frame, "object_frame");
    nh.param<std::string>("camera_frame", camera_frame, "camera_frame");
    nh.param<std::string>("camera_ref_frame", camera_ref_frame, "iiwa_link_ee");
    nh.param<std::string>("robot_base", robot_base, "iiwa_link_0");

    //publish the tf between camera wrt to iiwa end-effector, this is fixed
    Eigen::Matrix4d came_matrix;
    came_matrix << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;
    trans_camera_ef = Eigen::Affine3d::Identity();
    trans_camera_ef.matrix() = came_matrix;

    // open iiwa subsribes and pulishers
    pose_state.init("iiwa");
    pose_command.init("iiwa");

    // Wait a bit, so that you can be sure the subscribers are connected.
    ros::Duration(0.5).sleep();

    // services
    set_absolute_srv = nh.advertiseService("absolute_motion", &MotionROS::set_absolute_callback, this);
    set_relative_srv = nh.advertiseService("relative_motion", &MotionROS::set_relative_callback, this);
    ROS_INFO("Ready...");   
}

MotionROS::~MotionROS()
{
    set_absolute_srv.shutdown();
    set_relative_srv.shutdown();
}

bool MotionROS::set_absolute_callback(motion_msgs::SetAbsolutePose::Request& request,
                                      motion_msgs::SetAbsolutePose::Response& response)
{
    response.success = setAbsolutePose(request.motion);
    return response.success;
}

bool MotionROS::set_relative_callback(motion_msgs::SetRelativePose::Request& request,
                                      motion_msgs::SetRelativePose::Response& response)
{
    response.success = setRelativePose(request.motion);
    return response.success;

}

// ////////////////////////////////////////////
bool MotionROS::setAbsolutePose(motion_msgs::AbsoluteMotion& motion)
{
    //get current object pose wrt camera
    geometry_msgs::PoseStamped cur_object_cam;
    cur_object_cam = motion.pose_cur;

    // 1.get the current object pose wrt robot base frame: two ways
    // use tf tree
    geometry_msgs::PoseStamped cur_object_base1 = transformPoseWrtFrame(nh, cur_object_cam, robot_base);
    std::cout << "--------------------------Current object pose wrt base 1: \n";
    printPoseStamp(cur_object_base1);

    // normal multipy
    Eigen::Affine3d ef_base_tr, object_cam_tr, object_base_tr;

    ///current robot state
    iiwa_msgs::CartesianPose car_pose = pose_state.getPose();
    current_ef_pose = car_pose.poseStamped;
    std::cout << "***************************Current robot ef pose : \n";
    printPoseStamp(car_pose.poseStamped);
    tf::poseMsgToEigen(current_ef_pose.pose, ef_base_tr);
    tf::poseMsgToEigen(cur_object_cam.pose, object_cam_tr);
    object_base_tr = ef_base_tr * trans_camera_ef * object_cam_tr;

    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(object_base_tr, pose);
    geometry_msgs::PoseStamped cur_object_base2;
    cur_object_base2.header.frame_id = robot_base;
    cur_object_base2.pose = pose;
    std::cout << "--------------------------Current object pose wrt base 2: \n";
    printPoseStamp(cur_object_base2);

    // 2. compute the desired ef pose wrt base frame,
    // this can not use tf tree since its not current state
    geometry_msgs::PoseStamped des_object_cam;
    des_object_cam = motion.pose_des;

    Eigen::Affine3d des_ef_base_tr, des_object_cam_tr;
    tf::poseMsgToEigen(des_object_cam.pose, des_object_cam_tr);
    des_ef_base_tr = object_base_tr * des_object_cam_tr.inverse() * trans_camera_ef.inverse();

    geometry_msgs::Pose des_ef;
    tf::poseEigenToMsg(des_ef_base_tr, des_ef);
    geometry_msgs::PoseStamped des_ef_base;
    des_ef_base.header.frame_id = robot_base;
    des_ef_base.pose = des_ef;
    std::cout << "***************************Desired robot ef pose : \n";
    printPoseStamp(des_ef_base);

    // 3. move the robot arm
    pose_command.setPose(des_ef_base);
//    ros::Duration(0.5).sleep();

    return true;
}

//
bool MotionROS::setRelativePose(motion_msgs::RelativeMotion& motion)
{
#ifdef DEBUG_PRINT
    std::cout <<"\n=========================================================:";
#endif
//    for(std::vector<actor_msgs::Actor>::iterator it = gpp_boxes.begin(); it != gpp_boxes.end(); ++it)
//    {
//        it->dependency_x.clear();  it->dependency_y.clear();  it->dependency_z.clear();
//        for(std::vector<actor_msgs::Actor>::iterator it_in = gpp_boxes.begin(); it_in != gpp_boxes.end(); ++it_in)
//        {
//            if( (it_in != it) && (xBlocking(*it_in, *it)) )
//            {
//                // if *it blocks box *it_in
//                it->dependency_x.push_back(it_in->uuid);
//            }
//            if( (it_in != it) && (yBlocking(*it_in, *it)) )
//                it->dependency_y.push_back(it_in->uuid);
//            if( (it_in != it) && (zBlocking(*it_in, *it)) )
//                it->dependency_z.push_back(it_in->uuid);
//        }
//    }
    return true;
}

void MotionROS::printPoseStamp(geometry_msgs::PoseStamped &poseStamped)
{
    ROS_INFO_STREAM(
          poseStamped.header.frame_id << " -- " << std::to_string(poseStamped.pose.position.x)
          << " " << std::to_string(poseStamped.pose.position.y) << " " << std::to_string(poseStamped.pose.position.z)
          << " " << std::to_string(poseStamped.pose.orientation.x) << " " << std::to_string(poseStamped.pose.orientation.y)
          << " " << std::to_string(poseStamped.pose.orientation.z) << " " << std::to_string(poseStamped.pose.orientation.w)
          << std::endl;);
    ros::Duration(0.1).sleep();
}

//get the transform between two frames
geometry_msgs::Pose MotionROS::getPoseWrtFrame(ros::NodeHandle &node, const std::string child_frame, const std::string frame)
{
    ros::ServiceClient client = node.serviceClient<motion_msgs::GetTransform>("/tf_server/get_transform");
    motion_msgs::GetTransform srv;
    srv.request.parent_frame = frame;
    srv.request.child_frame = child_frame;

    if (client.call(srv))
    {
        geometry_msgs::Pose pose;
        Eigen::Affine3d pose_tr;
        tf::transformMsgToEigen(srv.response.transform, pose_tr);
        tf::poseEigenToMsg(pose_tr, pose);

        return pose;
    }
    else
    {
        ROS_ERROR("Failed to call service /tf_server/get_transform");
        throw;
    }
}

//get pose wrt target frame
geometry_msgs::PoseStamped MotionROS::transformPoseWrtFrame(ros::NodeHandle &node, const geometry_msgs::PoseStamped &pose, const std::string frame)
{
    ros::ServiceClient client = node.serviceClient<motion_msgs::TransformPose>("/tf_server/transform_pose");
    motion_msgs::TransformPose srv;
    srv.request.pose = pose;
    srv.request.target_frame = frame;
    if (client.call(srv))
    {
        return srv.response.poseWrtTarget;
    }
    else
    {
        ROS_ERROR("Failed to call service /tf_server/transform_pose");
        throw;
    }
}

