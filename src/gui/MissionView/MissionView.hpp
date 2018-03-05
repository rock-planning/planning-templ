#ifndef TEMPL_GUI_MISSION_VIEW_HPP
#define TEMPL_GUI_MISSION_VIEW_HPP

#include <QTreeWidget>
#include <QWidget>
#include <QProcess>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsGridLayout>
#include <QTreeView>
#include <QStandardItem>
#include <QDomNode>

#include <graph_analysis/Graph.hpp>
#include <templ/Mission.hpp>

namespace templ {
namespace gui {

class MissionView : public QTreeView
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
    QStandardItem* mpItem;
    Mission::Ptr mpMission;


    void updateVisualization();
    void loadXML(const QString& missionSpec);
    void refreshView();

    QStandardItem* createRoot(const QString& name);
    QStandardItem* createChild(QStandardItem* item, const QDomNode& node);
    void setItem(QStandardItemModel* model);

    void preOrder(QDomNode node, QStandardItemModel* model, QStandardItem* item = 0);

public slots:
    void on_planMission_clicked();
    // Adding/Removing Constraints
    void on_addConstraintButton_clicked();
    void on_removeConstraintButton_clicked();

    // Loading/Storing Missions
    void on_loadMissionButton_clicked();

    void on_saveButton_clicked();
    void on_updateButton_clicked();
    void on_clearButton_clicked();

};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_MISSION_VIEW_HPP
