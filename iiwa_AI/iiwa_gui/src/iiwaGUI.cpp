#include "iiwaGUI.h"
#include "ui_iiwaGUI.h"

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
