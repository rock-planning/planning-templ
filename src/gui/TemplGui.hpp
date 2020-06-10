#ifndef TEMPL_GUI_TEMPL_GUI_HPP
#define TEMPL_GUI_TEMPL_GUI_HPP

#include <iostream>
#include <QMainWindow>
#include <graph_analysis/gui/QBaseGraph.hpp>
#include <graph_analysis/gui/layouts/GridLayout.hpp>
#include <qxcfg/Configuration.hpp>
#include <QProcess>
#include <moreorg/OrganizationModel.hpp>

namespace Ui {
    class TemplGui;
}

namespace graph_analysis {
namespace gui {
    class BaseGraphView;
}
}

namespace templ {
namespace gui {

class MissionEditor;
class MissionView;
class OntologyView;

class TemplGui : public QMainWindow
{
    Q_OBJECT

public:
    TemplGui();
    ~TemplGui();

    static moreorg::OrganizationModel::Ptr getOrganizationModel() { return mpsOrganizationModel; }

private:
    // gui elements
    Ui::TemplGui* mpUi;

    // the subwigets, present in this window
    std::vector<graph_analysis::gui::BaseGraphView*> mBaseGraphViews;

    /// Store recent file actions list -- allow for additional set of
    QMap<QString, QList<QAction*> > mpRecentFileActionsMap;

    enum { MaxRecentFiles = 5 };

    MissionEditor* mpMissionEditor;
    MissionView* mpMissionView;
    OntologyView* mpOntologyView;

    /// General configuration
    qxcfg::Configuration mConfiguration;

    QProcess* mpProcess;

    static moreorg::OrganizationModel::Ptr mpsOrganizationModel;

    void notifyAll();

    void registerGraphElementTypes();

    /**
     * Create all menus
     */
    void createMenus();

    //void activateEdgeStyle();

    QString strippedName(const QString& fullFileName);

    void activateGraph(graph_analysis::BaseGraph::Ptr& graph, const QString& tabLabel = "Graph");

    static std::string getColumnLabel(const graph_analysis::Vertex::Ptr& vertex);
    static std::string getRowLabel(const graph_analysis::Vertex::Ptr& vertex);

    static void sortRowLabel(const graph_analysis::BaseGraph::Ptr& graph, graph_analysis::gui::layouts::GridLayout::ColumnLabels& labels);

    QMenu* createRecentFilesMenu();

    graph_analysis::gui::BaseGraphView* getCurrentBaseGraphView() const;

private slots:
    void importSolution(const QString& settingsLabel ="IOSolutions",
            const QString& filename = "");
    void editMission(const QString& settingsLabel = "IOMissions",
            const QString& filename = "");
    void importMission(const QString& settingsLabel = "IOMissions",
            const QString& filename = "");
    void importGraph(const QString& settingsLabel = "IOGraphs");
    void importOrganizationModel();

    void exportGraph();
    void selectLayout();
    void clearView();
    void customizeGridLayout();
    void exportScene();

    /**
     * \param settingsLabel Label to separate the recentImportFileList under
     * different namespaces in QSettings
     */
    void importRecentGraph(const QString& settingsLabel = "IOGraphs");
    void importRecentSolution();
    void importRecentMission();
    void importRecentOrganizationModel();
    void importRecentConfiguration();

    void updateRecentFileActions();
    void updateRecentFileActions(const QString& label);

    void clearRecentFileList(const QString& name);
    void clearRecentGraphs() { clearRecentFileList("IOGraphs"); }
    void clearRecentSolutions() { clearRecentFileList("IOSolutions"); }
    void clearRecentMissions() { clearRecentFileList("IOMissions"); }
    void clearRecentConfigurations() { clearRecentFileList("IOConfiguration"); }

    void setChecked(bool check) { std::cout << "CHECKED: " << check; }


    /*Connected to QBaseGraph*/
    void updateVisualization();
    void closeTab(int tabIndex);

    void openMissionEditor(const QString& settingsLabel = "IOMissions", const QString& filename = "",
            bool autoload = true);
    void openMissionView(const QString& settingsLabel = "IOMissions", const QString& filename = "");
    void openOntologyView();

    // Planner
    void runPlanner();
    void stopPlanner();
    void loadConfiguration(const QString& filename = "",
            const QString& settingsLabel = "IOConfiguration");
    void editConfiguration();
    void logOutput();
};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_TEMPL_GUI_HPP
