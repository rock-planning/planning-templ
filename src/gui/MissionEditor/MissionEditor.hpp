#ifndef TEMPL_GUI_MISSION_EDITOR_HPP
#define TEMPL_GUI_MISSION_EDITOR_HPP

#include <QTreeWidget>
#include <QWidget>
#include <QProcess>
#include <graph_analysis/Graph.hpp>

namespace Ui {
    class MissionEditor;
}

namespace templ {
namespace gui {

class MissionEditor : public QWidget
{
    Q_OBJECT

public:
    MissionEditor(graph_analysis::BaseGraph::Ptr graph,
                    QWidget* parent = NULL);
    ~MissionEditor();

    QString getClassName() const
    {
        return "templ::gui::MissionEditor";
    }

private:
    // GUI Elements
    Ui::MissionEditor* mpUi;

    // Graph
    graph_analysis::BaseGraph::Ptr mpGraph;

    // Adding/Removing Constraints
    void on_addConstraintButton_clicked();
    void on_removeConstraintButton_clicked();

    // Loading/Storing Missions
    void on_loadMissionButton_clicked();
    void on_loadOntologyButton_clicked();

    void on_saveButton_clicked();
    void on_updateButton_clicked();
    void on_clearButton_clicked();

    void updateVisualization();

public slots:
    void on_planMission_clicked();

};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_MISSION_EDITOR_HPP
