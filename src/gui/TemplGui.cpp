#include "TemplGui.hpp"

#include "ui_TemplGui.h"

#include <graph_analysis/gui/BaseGraphView/BaseGraphView.hpp>
#include <graph_analysis/gui/VertexItemTypeManager.hpp>
#include <graph_analysis/gui/EdgeItemTypeManager.hpp>

#include <QDebug>
#include <QFileInfo>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QCommonStyle>
#include <QInputDialog>
#include <QGraphicsGridLayout>
#include <QSettings>
#include <QPixmap>
#include <QFileDialog>
#include <QPrinter>
#include <QPrintDialog>
#include <QSvgGenerator>
#include <QPalette>

#include <QDockWidget>
#include <QTableWidget>
#include <QFileSystemModel>

#include <base-logging/Logging.hpp>

#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/gui/GraphWidget.hpp>
#include <graph_analysis/gui/ActionCommander.hpp>
#include <graph_analysis/gui/dialogs/IODialog.hpp>
#include <graph_analysis/gui/GraphLayoutManager.hpp>
#include <graph_analysis/gui/layouts/GridLayout.hpp>
#include <graph_analysis/gui/dialogs/GridLayout.hpp>

#include <templ/gui/MissionEditor/MissionEditor.hpp>
#include <templ/gui/MissionView/MissionView.hpp>
#include <templ/gui/OntologyView/OntologyView.hpp>

#include "../SpaceTime.hpp"
#include "../RoleInfoWeightedEdge.hpp"
#include "../gui/edge_items/RoleInfoItem.hpp"
#include "../gui/edge_items/CapacityLinkItem.hpp"
#include "../gui/vertex_items/RoleInfoItem.hpp"
#include "../solvers/temporal/QualitativeTemporalConstraintNetwork.hpp"
#include "models/AgentStyleModel.hpp"
#include "widgets/PenStyle.hpp"

using namespace graph_analysis;
using namespace graph_analysis::gui;

namespace templ {
namespace gui {

TemplGui::TemplGui()
    : QMainWindow()
    , mpUi(new Ui::TemplGui)
    , mpBaseGraph(graph_analysis::BaseGraph::getInstance())
    , mpQBaseGraph(new graph_analysis::gui::QBaseGraph(mpBaseGraph))
    , mpBaseGraphView(new graph_analysis::gui::BaseGraphView(mpBaseGraph, this))
    , mpMissionEditor(new MissionEditor(this))
    , mpMissionView(new MissionView(this))
    , mpOntologyView(new OntologyView(this))
{
    mpUi->setupUi(this);

    mpUi->tabWidget->clear();
    mpUi->tabWidget->addTab(mpBaseGraphView, "Solution View");

    GraphLayoutManager* layoutManager = GraphLayoutManager::getInstance();
    GraphLayout::Ptr layout = layoutManager->getGraphLayout("grid-layout-default");
    layouts::GridLayout::Ptr gridLayout = dynamic_pointer_cast<layouts::GridLayout>(layout);
    gridLayout->setColumnLabelFunction(bind(&TemplGui::getColumnLabel, std::placeholders::_1));
    gridLayout->setRowLabelFunction(bind(&TemplGui::getRowLabel, std::placeholders::_1));
    gridLayout->setSortRowLabelFunction(bind(&TemplGui::sortRowLabel, std::placeholders::_1, std::placeholders::_2));
    gridLayout->setColumnScalingFactor(10.0);
    gridLayout->setRowScalingFactor(10.0);

    mpUi->tabWidget->addTab(mpMissionEditor,
                            mpMissionEditor->getClassName());
    mpUi->tabWidget->addTab(mpMissionView,
                            "Mission View");
    mpUi->tabWidget->addTab(mpOntologyView,
                            mpOntologyView->getClassName());
    mpUi->tabWidget->setCurrentWidget(mpMissionEditor);

    // and show both' widgets status-messages on the statusbar. this simply
    // assumes that only the one in the front is sending updates. otherwise
    // they would interleave...
    connect(mpBaseGraphView, SIGNAL(currentStatus(QString, int)),
            mpUi->statusbar, SLOT(showMessage(QString, int)));
    connect(mpMissionEditor, SIGNAL(currentStatus(QString, int)),
            mpUi->statusbar, SLOT(showMessage(QString, int)));

    //connect(mpQBaseGraph, SIGNAL(graphChanged()),
    //        this, SLOT(updateVisualization()));

    createMenus();

    QScrollArea* scrollArea = new QScrollArea;
    scrollArea->setWidget( new widgets::PenStyle(mpUi->solutionStyleWidget));
    mpUi->solutionStyleWidget->addTab(scrollArea, "Graph Style");

    //mpUi->solutionStyleWidget->addTab(new widgets::PenStyle(mpUi->solutionStyleWidget), "Graph Style");
    //activateEdgeStyle();

    updateRecentFileActions();

    registerGraphElementTypes();

    // Making sure the bottom docking widget sits between the two outer docking
    // widgets
    setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);
}

void TemplGui::createMenus()
{
    ActionCommander comm(this);
    QStyle* style = new QCommonStyle();

    // BEGIN FILE
    QMenu *fileMenu = new QMenu(QObject::tr("&File"));

    QAction *actionImport = comm.addAction("Import", SLOT(importGraph()), style->standardIcon(QStyle::SP_FileIcon)        , QKeySequence( QKeySequence::Open ), tr("Import graph from file"));
    QAction *actionExport = comm.addAction("Export", SLOT(exportGraph()), style->standardIcon(QStyle::SP_DialogSaveButton), QKeySequence( QKeySequence::SaveAs), tr("Export graph to file"));
    QAction *selectLayout = comm.addAction("Layout", SLOT(selectLayout()), style->standardIcon(QStyle::SP_FileDialogListView), QKeySequence( Qt::ControlModifier & Qt::Key_L), tr("Export graph to file"));
    QAction *saveGraphics = comm.addAction("Export Scene", SLOT(exportScene()), style->standardIcon(QStyle::SP_DialogSaveButton), QKeySequence(Qt::ControlModifier & Qt::Key_I), tr("Export scene"));

    fileMenu->addAction(actionImport);
    fileMenu->addAction(actionExport);
    fileMenu->addAction(selectLayout);
    fileMenu->addAction(saveGraphics);
    fileMenu->addSeparator();

    QMenu* recentFilesMenu = new QMenu(QObject::tr("&Recent Files"));
    connect(recentFilesMenu, SIGNAL(aboutToShow()), this, SLOT(updateRecentFileActions()));

    fileMenu->addMenu(recentFilesMenu);
    // Populate the recent files list
    for(int i = 0; i < MaxRecentFiles; ++i)
    {
        QAction* action = new QAction(this);
        action->setVisible(false);
        mpRecentFileActions.push_back(action);
        connect(action, SIGNAL(triggered()), this, SLOT(importRecentFile()));
    }

    // account only for the existing files
    int existingRecentFiles = qMin(mpRecentFileActions.size(), (int) MaxRecentFiles);
    for(int i = 0; i < existingRecentFiles; ++i)
    {
        recentFilesMenu->addAction(mpRecentFileActions[i]);
    }
    QAction* clearRecentFiles = comm.addAction("Clear list", SLOT(clearRecentFiles()), style->standardIcon(QStyle::SP_TrashIcon), QKeySequence(Qt::ControlModifier & Qt::Key_C), tr("Clear recent files"));
    recentFilesMenu->addAction(clearRecentFiles);
    fileMenu->addSeparator();
    // END FILE

    // BEGIN View
    // https://joekuan.wordpress.com/2015/09/23/list-of-qt-icons/ for list of
    // standard icons
    QMenu *viewMenu = new QMenu(QObject::tr("&View"));
    QAction *clearView = comm.addAction("Clear", SLOT(clearView()), style->standardIcon(QStyle::SP_TrashIcon), QKeySequence( Qt::ControlModifier & Qt::Key_X), tr("Clear view"));
    viewMenu->addAction(clearView);
    QAction *gridLayoutAction = comm.addAction("Customize GridLayout", SLOT(customizeGridLayout()), style->standardIcon(QStyle::SP_FileDialogListView), QKeySequence(Qt::ControlModifier & Qt::Key_G), tr("Customize grid layout"));
    viewMenu->addAction(gridLayoutAction);
    // END View

    // BEGIN Windows
    // BEGIN Windows->Dockable Dialogs
    QMenu* windowsMenu = new QMenu(QObject::tr("&Windows"));
    QMenu* viewDockWidgetMenu = windowsMenu->addMenu("&Dockable Dialogs");
    viewDockWidgetMenu->addAction( mpUi->dockWidgetLeft->toggleViewAction() );
    viewDockWidgetMenu->addAction( mpUi->dockWidgetRight->toggleViewAction() );
    viewDockWidgetMenu->addAction( mpUi->dockWidgetBottom->toggleViewAction() );


    QFileSystemModel* model = new QFileSystemModel;
    model->setRootPath(QDir::currentPath());
    mpUi->filesystemTreeView->setModel(model);
    mpUi->filesystemTreeView->setRootIndex(model->index(QDir::currentPath()));
    mpUi->filesystemTab->setEnabled(true);

    organization_model::ModelPool modelPool;
    modelPool["http://test/sherpa#model"] = 5;
    QAbstractTableModel* agentStyleModel = new models::AgentStyleModel(modelPool);
    mpUi->agentStyleModelView->setModel(agentStyleModel);
    connect(mpUi->agentStyleModelView, SIGNAL(doubleClicked(QModelIndex)), agentStyleModel, SLOT(setColor(QModelIndex)));
    //models::ColorChooserDelegate* colorChooserDelegate = new models::ColorChooserDelegate(this);
    //mpUi->agentStyleModelView->setItemDelegate(colorChooserDelegate);
    mpUi->solutionStyleWidget->setEnabled(true);
    // END Windows->Dockable Dialogs
    // END Windows

    QMenuBar *bar = menuBar();
    bar->addMenu(fileMenu);
    bar->addMenu(viewMenu);
    bar->addMenu(windowsMenu);

    QToolBar* toolBar = new QToolBar("Toolbar");
    toolBar->addAction(actionImport);
    toolBar->addAction(actionExport);
    toolBar->addAction(selectLayout);
    toolBar->setFloatable(true);
    addToolBar(toolBar);

}

//void TemplGui::activateEdgeStyle()
//{
//    QPalette* palette = new QPalette();
//    palette->setColor(QPalette::Button, Qt::black);
//    mpUi->colorPushButton->setPalette(*palette);
//    mpUi->colorPushButton->setText("OK");
//    QString s = "background-color: red";
//    mpUi->colorPushButton->setStyleSheet(s);
//}

TemplGui::~TemplGui()
{
    delete mpUi;
}

void TemplGui::registerGraphElementTypes()
{
    using namespace templ;
    using namespace graph_analysis;
    graph_analysis::gui::EdgeItemTypeManager* eManager = graph_analysis::gui::EdgeItemTypeManager::getInstance();

    // Edges
    {
        {
            RoleInfoWeightedEdge::Ptr e(new RoleInfoWeightedEdge());
            eManager->registerVisualization(e->getClassName(), new edge_items::RoleInfoItem());
        }
        {
            CapacityLink::Ptr e(new CapacityLink());
            eManager->registerVisualization(e->getClassName(), new edge_items::CapacityLinkItem());
        }
    }


    SpaceTime::Network::tuple_t::Ptr v(new SpaceTime::Network::tuple_t());

    graph_analysis::gui::VertexItemTypeManager* vManager = graph_analysis::gui::VertexItemTypeManager::getInstance();
    vManager->registerVisualization(v->getClassName(), new vertex_items::RoleInfoItem());

}

void TemplGui::updateRecentFileActions()
{
    QSettings settings(QCoreApplication::organizationName(), "IO");
    QStringList files = settings.value("recentImportFileList").toStringList();

    int numRecentFiles = qMin(files.size(), (int) MaxRecentFiles);
    for(int i = 0; i < numRecentFiles; ++i)
    {
        QString text = tr("&%1 %2").arg(i + 1).arg(files[i]);
        mpRecentFileActions[i]->setText(text);
        mpRecentFileActions[i]->setData(files[i]);
        mpRecentFileActions[i]->setVisible(true);
    }
    for(int j = numRecentFiles; j < MaxRecentFiles; ++j)
    {
        mpRecentFileActions[j]->setVisible(false);
    }
}

QString TemplGui::strippedName(const QString& fullFileName)
{
    return QFileInfo(fullFileName).fileName();
}

void TemplGui::importGraph()
{
    graph_analysis::BaseGraph::Ptr graph = graph_analysis::gui::dialogs::IODialog::importGraph(this);
    activateGraph(graph);
}

void TemplGui::activateGraph(graph_analysis::BaseGraph::Ptr& graph)
{
    if(graph)
    {
        //updateVisualization();
        mpBaseGraphView->clearVisualization();
        mpBaseGraph = graph;

        delete mpQBaseGraph;
        mpQBaseGraph = new QBaseGraph(mpBaseGraph);
        mpBaseGraphView->setGraph(mpBaseGraph);
        mpBaseGraphView->refresh();
        mpBaseGraphView->updateVisualization();
        mpBaseGraphView->applyLayout("dot");
        mpBaseGraphView->updateVisualization();
    } else {
        qDebug() << "Failed to activate graph";
    }
}

void TemplGui::exportGraph()
{
    graph_analysis::gui::dialogs::IODialog::exportGraph(mpBaseGraphView->graph());
}

void TemplGui::importRecentFile()
{
    QAction *action = qobject_cast<QAction*>(sender());
    if(action)
    {
        qDebug() << "Importing file from: " << action->data().toString();
        graph_analysis::BaseGraph::Ptr graph = graph_analysis::gui::dialogs::IODialog::importGraph(this, action->data().toString());

        activateGraph(graph);
    }
}

void TemplGui::clearView()
{
    mpBaseGraphView->clearVisualization();
}

void TemplGui::selectLayout()
{
    if(mpUi->tabWidget->currentWidget() == mpBaseGraphView)
    {
        bool ok;
        QString desiredLayout = QInputDialog::getItem(this, tr("Select Layout"),
                                    tr("select a layout:"), mpBaseGraphView->getSupportedLayouts(),
                                    0, false, &ok);
        if(ok)
        {
            mpBaseGraphView->applyLayout(desiredLayout.toStdString());
        }
    }
    updateVisualization();
}

void TemplGui::customizeGridLayout()
{
    graph_analysis::gui::dialogs::GridLayout::execute();
}

void TemplGui::on_tabWidget_currentChanged(int index)
{
    // When the tab changed, we want to update the widget
    this->updateVisualization();
}

void TemplGui::updateVisualization()
{
    // Call the current tab widget's update function
    if (mpUi->tabWidget->currentWidget() == mpBaseGraphView)
    {
        assert(mpBaseGraphView);
        mpBaseGraphView->updateVisualization();
    } else if (mpUi->tabWidget->currentWidget() == mpMissionEditor)
    {
        mpMissionEditor->update();
    }
}

std::string TemplGui::getColumnLabel(const graph_analysis::Vertex::Ptr& vertex)
{
    std::string columnLabel;
    SpaceTime::SpaceTimeTuple::Ptr tuple = dynamic_pointer_cast<SpaceTime::SpaceTimeTuple>(vertex);
    if(tuple)
    {
        columnLabel = tuple->first()->toString();
    }
    return columnLabel;
}

std::string TemplGui::getRowLabel(const graph_analysis::Vertex::Ptr& vertex)
{
    std::string rowLabel;
    SpaceTime::SpaceTimeTuple::Ptr tuple = dynamic_pointer_cast<SpaceTime::SpaceTimeTuple>(vertex);
    if(tuple)
    {
        rowLabel = tuple->second()->getLabel();
    }
    return rowLabel;
}

void TemplGui::sortRowLabel(const graph_analysis::BaseGraph::Ptr& graph, graph_analysis::gui::layouts::GridLayout::RowLabels& labels)
{
    using namespace graph_analysis;
    using namespace templ::solvers;
    using namespace templ::solvers::temporal;

    QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());

    // Get the start vertices (which have no incoming vertices)
    std::vector<Vertex::Ptr> vertices;
    VertexIterator::Ptr vertexIt = graph->getVertexIterator();
    while(vertexIt->next())
    {
        Vertex::Ptr vertex = vertexIt->current();
        if(graph->getInEdges(vertex).empty())
        {
            vertices.push_back(vertex);
        }
    }

    // In the temporal network a forward edge can only be between this and the
    // next timepoint
    // Thus, trying to infer the timepoint network from the existing
    // relationship to construct a QualitativeTemporalConstraintNetwork
    while(!vertices.empty())
    {
        Vertex::Ptr vertex = vertices.back();
        vertices.pop_back();

        SpaceTime::SpaceTimeTuple::Ptr sourceTuple = dynamic_pointer_cast<SpaceTime::SpaceTimeTuple>(vertex);
        if(sourceTuple)
        {
            // Get the outgoing edges
            EdgeIterator::Ptr edgeIt = graph->getOutEdgeIterator(vertex);
            while(edgeIt->next())
            {
                Vertex::Ptr targetVertex = edgeIt->current()->getTargetVertex();
                vertices.push_back(targetVertex);

                // Add the temporal constraint
                SpaceTime::SpaceTimeTuple::Ptr targetTuple = dynamic_pointer_cast<SpaceTime::SpaceTimeTuple>(targetVertex);
                qtcn->addQualitativeConstraint(sourceTuple->second(),
                        targetTuple->second(), point_algebra::QualitativeTimePointConstraint::Less);
            }
        }
    }
    assert(qtcn->isConsistent());
    point_algebra::TimePointComparator comparator(qtcn);

    // Now that we have the temporal constraint network, we can sort the labels,
    // using the temporal relationships
    std::sort( labels.begin(), labels.end(), [comparator](const std::string& t0, const std::string& t1)
            {
                point_algebra::TimePoint::Ptr tp0 = comparator.getTemporalConstraintNetwork()->getTimePoint(t0);
                point_algebra::TimePoint::Ptr tp1 = comparator.getTemporalConstraintNetwork()->getTimePoint(t1);

                return comparator.lessThan(tp0, tp1);
            });
}

void TemplGui::exportScene()
{
    graph_analysis::gui::dialogs::IODialog::exportScene(this->mpBaseGraphView->scene(), this);
}

void TemplGui::clearRecentFiles()
{
    QSettings settings(QCoreApplication::organizationName(), "IO");
    QStringList files;
    settings.setValue("recentImportFileList", files);
    updateRecentFileActions();
}

} // end namespace gui
} // end namespace templ
