#include <stdlib.h>
#include <QApplication>
#include "iiwaGUI.h"


int main(int argc, char *argv[])
{

    QApplication app(argc, argv);
    iiwaGUI w;
    w.show();

    return app.exec();
}
