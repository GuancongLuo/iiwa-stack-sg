#include "PlanningGUI.h"
#include "ui_PlanningGUI.h"
#include <thread>

PlanningGUI::PlanningGUI(QWidget *parent, HopROS *hop) :
    QMainWindow(parent),
    ui(new Ui::PlanningGUI),
    hop_(hop)
{
    ui->setupUi(this);
    if(hop_ != nullptr)
        QObject::connect(hop_, SIGNAL(updateStep(const int&, const QString&)), this, SLOT(slotUpdateStep(const int&, const QString&)));
}

PlanningGUI::~PlanningGUI()
{
    delete ui;
}

std::string get_action_tooltip(rosplan_dispatch_msgs::ActionDispatch &action)
{
    std::stringstream ss_action_tooltip;
    ss_action_tooltip << action.name << "(" << action.parameters.front().value << ",";
    for(size_t param_id = 1;param_id < action.parameters.size();++param_id)
    {
        ss_action_tooltip << action.parameters[param_id].value;
        if(param_id != action.parameters.size()-1)
             ss_action_tooltip << ", ";
    }
    ss_action_tooltip << ")";
    return ss_action_tooltip.str();
}

std::string get_action_string(rosplan_dispatch_msgs::ActionDispatch &action)
{
    std::stringstream ss_action_name;
    ss_action_name << action.action_id << " - " << action.name;
    return ss_action_name.str();
}

void PlanningGUI::slotUpdateStep(const int& action_id, const QString& action_state)
{
    std::cout << action_id << "," << action_state.toStdString() << std::endl;
    if(action_id < hop_->plan_.plan.size())
    {
        if(action_state == "action achieved")
        {
            ui->widget->lastActionAchieved_ = get_action_tooltip(hop_->plan_.plan[action_id]);
        }
        ui->widget->updateNodeStatus(get_action_tooltip(hop_->plan_.plan[action_id]), action_state.toStdString());
    }
}

void PlanningGUI::on_button_getPlan_clicked()
{
//    ui->widget->clearScene();
    ui->widget->grayOldScene();
    for(int i = 0;i < hop_->plan_.plan.size();++i)
    {
        rosplan_dispatch_msgs::ActionDispatch action = hop_->plan_.plan[i];
        ui->widget->addState(get_action_tooltip(action), get_action_tooltip(action), 0, 200*ui->widget->count_plan_);
    }

    if(ui->widget->lastActionAchieved_ != "none")
    {
        if(!ui->widget->addConnection(ui->widget->lastActionAchieved_, get_action_tooltip(hop_->plan_.plan[0])))
        {
            std::stringstream ss_name;
            ss_name << ui->widget->lastActionAchieved_ << "_" << ui->widget->count_plan_;
            ui->widget->addConnection(ss_name.str(), get_action_tooltip(hop_->plan_.plan[0]));
        }
    }

    for(int i = 1;i < hop_->plan_.plan.size();++i)
    {
        rosplan_dispatch_msgs::ActionDispatch action = hop_->plan_.plan[i];
        ui->widget->addConnection(get_action_tooltip(hop_->plan_.plan[i-1]), get_action_tooltip(action));
    }
    ++ui->widget->count_plan_;
}

void PlanningGUI::on_button_runPlan_clicked()
{
    std::thread t1(&HopROS::dispatch_plan_client, hop_);
    t1.detach();
//    hop_->dispatch_plan_client();
}

void PlanningGUI::on_button_stopPlan_clicked()
{
    hop_->cancel_plan_client();
}

void PlanningGUI::on_button_rePlan_clicked()
{
    hop_->problem_generator_client();
    ros::spinOnce();
    hop_->generate_plan_client();
    ros::spinOnce();
    std::stringstream ss_plan_path;
    ss_plan_path << ros::package::getPath("handover_plan") << "/common/plan.pddl";
    hop_->parse_plan_client(ss_plan_path.str());
}

void PlanningGUI::on_button_changeGoal_clicked()
{
    hop_->update_goal("human_l", "human_hand");
}

void PlanningGUI::on_button_changeState_clicked()
{

}
