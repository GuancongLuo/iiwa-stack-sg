#ifndef PLANNINGGUI_H
#define PLANNINGGUI_H

#include <QMainWindow>
#include "HopROS.h"

namespace Ui {
class PlanningGUI;
}

class PlanningGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit PlanningGUI(QWidget *parent = 0, HopROS *hop = nullptr);
    ~PlanningGUI();
    HopROS *hop_;

private slots:

    void on_button_getPlan_clicked();

    void on_button_runPlan_clicked();

    void on_button_stopPlan_clicked();

    void on_button_rePlan_clicked();

    void on_button_changeGoal_clicked();

    void on_button_changeState_clicked();

public slots:
    void slotUpdateStep(const int&, const QString&);
private:
    Ui::PlanningGUI *ui;    
};

#endif // PLANNINGGUI_H
