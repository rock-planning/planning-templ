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
    app.setApplicationName("Templ");

    Q_INIT_RESOURCE(resources);

    // provide seed for force-based layouting in the LayerViewWidget and
    // ComponentEditorWidget
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));

    TemplGui w;
    w.setWindowTitle("Templ: Temporal Planning for Reconfigurable Multi-Robot Systems");
    w.show();

    return app.exec();
}
