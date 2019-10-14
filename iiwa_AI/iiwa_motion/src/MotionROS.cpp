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
    std::string name = motion.motion_name;
    geometry_msgs::PoseStamped cur_object_cam = motion.pose_cur;
    geometry_msgs::PoseStamped des_object_cam = motion.pose_des;

    // 2. There are two ways to get the current object wrt base frame: object_base
    // Method 1: use tf tree
    geometry_msgs::PoseStamped cur_object_base1 = transformPoseWrtFrame(nh, cur_object_cam, robot_base);
    std::cout << "--------------------------Current object pose wrt base 1: \n";
    printPoseStamp(cur_object_base1);
    Eigen::Affine3d object_base_tr;
    tf::poseMsgToEigen(cur_object_base1.pose, object_base_tr);

    // current robot state
    iiwa_msgs::CartesianPose car_pose = pose_state.getPose();
    current_ef_pose = car_pose.poseStamped;
    std::cout << "***************************Current robot ef pose : \n";
    printPoseStamp(current_ef_pose);

    // Method 2: normal multipy. check the frames.pdf
//    Eigen::Affine3d ef_base_tr, object_cam_tr;
//    tf::poseMsgToEigen(current_ef_pose.pose, ef_base_tr);
//    tf::poseMsgToEigen(cur_object_cam.pose, object_cam_tr);
//    object_base_tr = ef_base_tr * trans_camera_ef * object_cam_tr;
//    geometry_msgs::Pose pose;
//    tf::poseEigenToMsg(object_base_tr, pose);
//    geometry_msgs::PoseStamped cur_object_base2;
//    cur_object_base2.header.frame_id = robot_base;
//    cur_object_base2.pose = pose;
//    std::cout << "--------------------------Current object pose wrt base 2: \n";
//    printPoseStamp(cur_object_base2);

    // 2. compute the desired ef pose wrt base frame: des_ef_base
    /// this can not use tf tree since its not current state
    Eigen::Affine3d des_ef_base_tr, des_object_cam_tr;
    tf::poseMsgToEigen(des_object_cam.pose, des_object_cam_tr);
    des_ef_base_tr = object_base_tr * des_object_cam_tr.inverse() * trans_camera_ef.inverse();

    geometry_msgs::Pose des_ef;
    tf::poseEigenToMsg(des_ef_base_tr, des_ef);
    desired_ef_pose.header.frame_id = robot_base;
    desired_ef_pose.pose = des_ef;
    std::cout << "***************************Desired robot ef pose : \n";
    printPoseStamp(desired_ef_pose); //wait for the robot finish movement

    // 3. move the robot arm
    pose_command.setPose(desired_ef_pose);
    ros::Duration(5).sleep();

    return true;
}

//
bool MotionROS::setRelativePose(motion_msgs::RelativeMotion& motion)
{

    // ??? What is mean?
#ifdef DEBUG_PRINT
    std::cout <<"\n=========================================================:";
#endif
    std::string name = motion.motion_name;
    geometry_msgs::PoseStamped cur_object_cam = motion.pose_cur;
    std::float_t percentage = motion.percentage_des;

    //0. get the current camera pose wrt object, use tf tree
    geometry_msgs::PoseStamped cur_cam_object = getPoseWrtFrame(nh, camera_frame, object_frame);
    std::cout << "--------------------------Current camera pose wrt object_frame: \n";
    printPoseStamp(cur_cam_object);

    // ------------------------------------------------------------------------
    //1. get the desired camera pose wrt object
    geometry_msgs::PoseStamped des_cam_object = cur_cam_object;
    std::float_t scale = 1 + percentage/100.0;
    if(name == "distance")
    {
        std::cout << "Desired percetage : " << 1 + percentage/100.0 << std::endl;
        des_cam_object.pose.position.x = cur_cam_object.pose.position.x * scale;
        des_cam_object.pose.position.y = cur_cam_object.pose.position.y * scale;
        des_cam_object.pose.position.z = cur_cam_object.pose.position.z * scale;
    }
    else if (name == "height")
    {
        std::cout << "Desired percetage : " << 1 + percentage/100.0 << std::endl;
        des_cam_object.pose.position.z = cur_cam_object.pose.position.z * scale;
        // orientation change from camere pulist tf_cam_object, don't need to calculation in here. 
    }
    else if (name == "angle")
    {
        double angle = percentage/180.0*M_PI;  //rad
        std::cout << "Desired angle : " << angle << std::endl; //left is -

        // cam_object vector rotate around Z object n degree
        Eigen::Affine3d Mat_new_old = Eigen::Affine3d(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1)));
        // Eigen::Affine3d m = Eigen::Affine3d(Eigen::AngleAxisd(angle/180.0*M_PI, Eigen::Vector3d::UnitZ()));

        Eigen::Affine3d cam_object_tr, cam_object_old;
        tf::poseMsgToEigen(cur_cam_object.pose, cam_object_tr);
        cam_object_old = Mat_new_old * cam_object_tr;

        geometry_msgs::Pose des_cam;
        tf::poseEigenToMsg(cam_object_old, des_cam);
        des_cam_object.pose = des_cam;
    }
    else
    {
        std::cout << "Wrong robot command name!!!!!!!!!!!!!!!!!!!!!!!!! \n";
    }
    std::cout << "--------------------------Desired camera pose wrt object_frame relatived: \n";
    printPoseStamp(des_cam_object);
    // ------------------------------------------------------------------------


    //2. compute the desired ef pose wrt base frame
    /// 1) object_wrt_base, use tf tree
    geometry_msgs::PoseStamped cur_object_base = transformPoseWrtFrame(nh, cur_object_cam, robot_base);
    std::cout << "--------------------------Current object pose wrt base:  \n";
    printPoseStamp(cur_object_base);
    Eigen::Affine3d object_base_tr;
    tf::poseMsgToEigen(cur_object_base.pose, object_base_tr);

    // current robot state
    iiwa_msgs::CartesianPose car_pose = pose_state.getPose();
    current_ef_pose = car_pose.poseStamped;
    std::cout << "***************************Current robot ef pose : \n";
    printPoseStamp(current_ef_pose);

    /// 2) des_cam_object
    Eigen::Affine3d des_ef_base_tr, des_cam_object_tr;
    tf::poseMsgToEigen(des_cam_object.pose, des_cam_object_tr);
    des_ef_base_tr = object_base_tr * des_cam_object_tr * trans_camera_ef.inverse();

    /// 2) des_ef_base
    geometry_msgs::Pose des_ef;
    tf::poseEigenToMsg(des_ef_base_tr, des_ef);
    desired_ef_pose.header.frame_id = robot_base;
    desired_ef_pose.pose = des_ef;
    std::cout << "***************************Desired robot ef pose : \n";
    printPoseStamp(desired_ef_pose); //wait for the robot finish movement

    // 3. move the robot arm
    pose_command.setPose(desired_ef_pose);
    ros::Duration(5).sleep();

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
geometry_msgs::PoseStamped MotionROS::getPoseWrtFrame(ros::NodeHandle &node, const std::string child_frame, const std::string frame)
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

        geometry_msgs::PoseStamped poseStamp;
        poseStamp.header.frame_id = frame;
        poseStamp.pose = pose;
        return poseStamp;
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

