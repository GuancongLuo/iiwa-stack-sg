#ifndef IIWAGUI_H
#define IIWAGUI_H

#include <QMainWindow>

namespace Ui {
class iiwaGUI;
}

class iiwaGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit iiwaGUI(QWidget *parent = 0);
    ~iiwaGUI();

private:
    Ui::iiwaGUI *ui;
};

#endif // IIWAGUI_H
