#ifndef TEMPL_GUI_MISSION_VIEW_HPP
#define TEMPL_GUI_MISSION_VIEW_HPP

#include <QTreeWidget>
#include <QWidget>
#include <QProcess>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsGridLayout>

#include <graph_analysis/Graph.hpp>
#include <templ/Mission.hpp>

namespace templ {
namespace gui {

class MissionView : public QGraphicsView
{
    Q_OBJECT

public:
    MissionView(QWidget* parent = NULL);
    ~MissionView();

    QString getClassName() const
    {
        return "templ::gui::MissionView";
    }

private:
    Mission::Ptr mpMission;
    QGraphicsGridLayout* mpGraphicsGridLayout;
    QGraphicsScene* mpScene;

    organization_model::OrganizationModel::Ptr mpOrganizationModel;

    // Adding/Removing Constraints
    void on_addConstraintButton_clicked();
    void on_removeConstraintButton_clicked();

    // Loading/Storing Missions
    void on_loadMissionButton_clicked();
    void on_loadOrganizationModelButton_clicked();

    void on_saveButton_clicked();
    void on_updateButton_clicked();
    void on_clearButton_clicked();

    void updateVisualization();

    void refreshView();

protected:
    /// qt mouse wheel spin callback
    void wheelEvent(QWheelEvent *event);
    /// scales scene (zooms into or out of the scene)
    void scaleView(qreal scaleFactor);

    void mousePressEvent(QMouseEvent*);

    void mouseReleaseEvent(QMouseEvent*);

    void contextMenuEvent(QContextMenuEvent* event);

public slots:
    void on_planMission_clicked();

};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_MISSION_VIEW_HPP
