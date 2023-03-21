#include <string>
#include <iostream>
#include <QResource>

#include <QApplication>
#include <QTime>

#include "TemplGui.hpp"

using namespace templ::gui;

int main(int argc, char **argv)
{
    // setting up qt application
    QApplication app(argc, argv);
    app.setOrganizationName("Templ");
    app.setApplicationName("Gui");

    Q_INIT_RESOURCE(resources);

    TemplGui w;
    w.setWindowTitle("Templ: Temporal Planning for Reconfigurable Multi-Robot Systems");
    w.show();

    return app.exec();
}
