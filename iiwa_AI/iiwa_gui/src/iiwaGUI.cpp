#include "iiwaGUI.h"
#include "ui_iiwaGUI.h"
// #include "Qmessage"

iiwaGUI::iiwaGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::iiwaGUI)
{
    ui->setupUi(this);
}

iiwaGUI::~iiwaGUI()
{
    delete ui;
}

// void iiwaGUI::on_pushButton_clicked()
// {
//     QmessageBox::worning(this,"My Tiele","This is my custom message");
// }