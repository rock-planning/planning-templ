#ifndef TEMPL_GUI_TEMPL_GUI_HPP
#define TEMPL_GUI_TEMPL_GUI_HPP

#include <iostream>
#include <QMainWindow>
#include <graph_analysis/gui/QBaseGraph.hpp>
#include <graph_analysis/gui/layouts/GridLayout.hpp>

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

private:
    // gui elements
    Ui::TemplGui* mpUi;

    // The actual base graph
    graph_analysis::BaseGraph::Ptr mpBaseGraph;

    // the Qt integration of the basegraph where the subwidgets will work upon
    graph_analysis::gui::QBaseGraph* mpQBaseGraph;

    // the subwigets, present in this window
    graph_analysis::gui::BaseGraphView* mpBaseGraphView;

    QList<QAction*> mpRecentFileActions;
    enum { MaxRecentFiles = 5 };

    MissionEditor* mpMissionEditor;
    MissionView* mpMissionView;
    OntologyView* mpOntologyView;

    void notifyAll();

    void registerGraphElementTypes();

    /**
     * Create all menus
     */
    void createMenus();

    //void activateEdgeStyle();

    QString strippedName(const QString& fullFileName);

    void activateGraph(graph_analysis::BaseGraph::Ptr& graph);

    static std::string getColumnLabel(const graph_analysis::Vertex::Ptr& vertex);
    static std::string getRowLabel(const graph_analysis::Vertex::Ptr& vertex);

    static void sortRowLabel(const graph_analysis::BaseGraph::Ptr& graph, graph_analysis::gui::layouts::GridLayout::ColumnLabels& labels);

private slots:
    void importGraph();
    void exportGraph();
    void selectLayout();
    void importRecentFile();
    void clearView();
    void customizeGridLayout();
    void exportScene();
    void updateRecentFileActions();
    void clearRecentFiles();

    void setChecked(bool check) { std::cout << "CHECKED: " << check; }


    /*Connected to QBaseGraph*/
    void updateVisualization();
    void on_tabWidget_currentChanged(int index);

};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_TEMPL_GUI_HPP
