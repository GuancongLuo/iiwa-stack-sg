#include <stdlib.h>
#include <QApplication>
#include "HopROS.h"
#include "PlanningGUI.h"

HopROS *hop_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hop");
    ros::NodeHandle _nh("~");

    hop_ = new HopROS(_nh);
    hop_->problem_generator_client();
    hop_->generate_plan_client();
    ros::spinOnce();
    std::stringstream ss_plan_path;
    ss_plan_path << ros::package::getPath("handover_plan") << "/common/plan.pddl";
    hop_->parse_plan_client(ss_plan_path.str());

    QApplication app(argc, argv);
    PlanningGUI *gui = new PlanningGUI(NULL, hop_);
//    gui->hop_ = ;
    gui->show();

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while(ros::ok()){
      app.processEvents();
      ros::Duration(0.002).sleep();
      ros::spinOnce();
    };

    std::cout << "============== Shutting Down GUI ================= " << std::endl;
    ros::shutdown();
    std::cout << "============== Shutting Down GUI Done ================= " << std::endl;

//    ros::spin();
    return 0;
}
