/* Cai 2019.06.26
 * iiwa cartesian movement, check the docs
 */
#include <iostream>
#include <csignal>

#include "MotionROS.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_motion");

    ros::NodeHandle _nh("~");
    MotionROS iiwa_move(_nh);

    ros::spin();
    return EXIT_SUCCESS;
}
