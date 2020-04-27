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
#include <QScrollBar>

#include <QDockWidget>
#include <QTableWidget>
#include <QFileSystemModel>
#include <QProcess>

#include <base-logging/Logging.hpp>

#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/gui/GraphWidget.hpp>
#include <graph_analysis/gui/ActionCommander.hpp>
#include <graph_analysis/gui/dialogs/IODialog.hpp>
#include <graph_analysis/gui/GraphLayoutManager.hpp>
#include <graph_analysis/gui/layouts/GridLayout.hpp>
#include <graph_analysis/gui/dialogs/GridLayout.hpp>

#include "MissionEditor/MissionEditor.hpp"
#include "MissionView/MissionView.hpp"
#include "OntologyView/OntologyView.hpp"

#include "../SpaceTime.hpp"
#include "../RoleInfoWeightedEdge.hpp"
#include "../gui/edge_items/RoleInfoItem.hpp"
#include "../gui/edge_items/CapacityLinkItem.hpp"
#include "../gui/vertex_items/RoleInfoItem.hpp"
#include "../solvers/temporal/QualitativeTemporalConstraintNetwork.hpp"
#include "../solvers/csp/TemporalConstraintNetwork.hpp"
#include "../solvers/SolutionAnalysis.hpp"
#include "models/AgentStyleModel.hpp"
#include "widgets/PenStyle.hpp"

using namespace graph_analysis;
using namespace graph_analysis::gui;

namespace templ {
namespace gui {

organization_model::OrganizationModel::Ptr TemplGui::mpsOrganizationModel;

TemplGui::TemplGui()
    : QMainWindow()
    , mpUi(new Ui::TemplGui)
    , mpMissionEditor(0)
    , mpMissionView(0)
    , mpOntologyView(0)
    , mConfiguration()
    , mpProcess(0)
{
    mpUi->setupUi(this);
    mpUi->tabWidget->clear();
    mpUi->tabWidget->setTabsClosable(true);
    mpUi->tabWidget->setMovable(true);

    mConfiguration.setValue("templ-gui/min_solutions","2");
    // Load any recent configuration file if available
    QSettings settings(QCoreApplication::organizationName());
    QVariant variant = settings.value("recentConfigurationFile");
    if(!variant.isNull())
    {
        loadConfiguration(variant.toString());
    }

    connect(mpUi->tabWidget,SIGNAL(tabCloseRequested(int)), this, SLOT(closeTab(int)));

    openOntologyView(); //MissionEditor();
    openMissionEditor();

    GraphLayoutManager* layoutManager = GraphLayoutManager::getInstance();
    GraphLayout::Ptr layout = layoutManager->getGraphLayout("grid-layout-default");
    layouts::GridLayout::Ptr gridLayout = dynamic_pointer_cast<layouts::GridLayout>(layout);
    gridLayout->setColumnLabelFunction(bind(&TemplGui::getColumnLabel, std::placeholders::_1));
    gridLayout->setRowLabelFunction(bind(&TemplGui::getRowLabel, std::placeholders::_1));
    gridLayout->setSortRowLabelFunction(bind(&TemplGui::sortRowLabel, std::placeholders::_1, std::placeholders::_2));
    gridLayout->setColumnScalingFactor(7.0);
    gridLayout->setRowScalingFactor(7.0);

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
    QMenu* fileMenu = new QMenu(QObject::tr("&File"));
    QMenu* newMenu = new QMenu(QObject::tr("&New"));
    QAction* actionMissionEditor = comm.addAction("Mission Editor",
                                                    SLOT(openMissionEditor()),
                                                    style->standardIcon(QStyle::SP_FileIcon),
                                                    QKeySequence( QKeySequence::Open & Qt::Key_E),
                                                    tr("Open mission editor"));
    //QAction* actionMissionView = comm.addAction("Mission View",
    //                                                SLOT(openMissionView()),
    //                                                style->standardIcon(QStyle::SP_FileIcon),
    //                                                QKeySequence( QKeySequence::Open & Qt::Key_M),
    //                                                tr("Open mission view"));
    QAction* actionOntologyView = comm.addAction("Ontology View",
                                                    SLOT(openOntologyView()),
                                                    style->standardIcon(QStyle::SP_FileIcon),
                                                    QKeySequence( QKeySequence::Open & Qt::Key_O),
                                                    tr("Open ontology view"));
    newMenu->addAction(actionMissionEditor);
    //newMenu->addAction(actionMissionView);
    newMenu->addAction(actionOntologyView);
    fileMenu->addMenu(newMenu);

    QMenu* importMenu = new QMenu(QObject::tr("&Import"));
    QAction *actionImport = comm.addAction("Import Graph", SLOT(importGraph()), style->standardIcon(QStyle::SP_FileIcon)        , QKeySequence( QKeySequence::Open ), tr("Import graph from file"));
    QAction *actionImportSolution = comm.addAction("Import Solution", SLOT(importSolution()), style->standardIcon(QStyle::SP_FileIcon)        , QKeySequence( QKeySequence::Open & Qt::Key_S ), tr("Import solution from file"));
    QAction *actionImportMission = comm.addAction("Import Mission", SLOT(importMission()), style->standardIcon(QStyle::SP_FileIcon)        , QKeySequence( QKeySequence::Open & Qt::Key_M ), tr("Import solution from file"));
    QAction *actionImportOrganizationModel = comm.addAction("Import Organization Model", SLOT(importOrganizationModel()), style->standardIcon(QStyle::SP_FileIcon)        , QKeySequence( QKeySequence::Open & Qt::Key_O ), tr("Import organization model from file"));

    importMenu->addAction(actionImport);
    importMenu->addAction(actionImportSolution);
    importMenu->addAction(actionImportMission);
    importMenu->addAction(actionImportOrganizationModel);
    fileMenu->addMenu(importMenu);

    QAction *actionExport = comm.addAction("Export", SLOT(exportGraph()), style->standardIcon(QStyle::SP_DialogSaveButton), QKeySequence( QKeySequence::SaveAs), tr("Export graph to file"));
    QAction *selectLayout = comm.addAction("Layout", SLOT(selectLayout()), style->standardIcon(QStyle::SP_FileDialogListView), QKeySequence( Qt::ControlModifier & Qt::Key_L), tr("Export graph to file"));
    QAction *saveGraphics = comm.addAction("Export Scene", SLOT(exportScene()), style->standardIcon(QStyle::SP_DialogSaveButton), QKeySequence(Qt::ControlModifier & Qt::Key_I), tr("Export scene"));

    fileMenu->addAction(actionExport);
    fileMenu->addAction(selectLayout);
    fileMenu->addAction(saveGraphics);
    fileMenu->addSeparator();


    // BEGIN View
    // https://joekuan.wordpress.com/2015/09/23/list-of-qt-icons/ for list of
    // standard icons
    QMenu *viewMenu = new QMenu(QObject::tr("&View"));
    QAction *clearView = comm.addAction("Clear", SLOT(clearView()), style->standardIcon(QStyle::SP_TrashIcon), QKeySequence( Qt::ControlModifier & Qt::Key_X), tr("Clear view"));
    viewMenu->addAction(clearView);
    QAction *gridLayoutAction = comm.addAction("Customize GridLayout", SLOT(customizeGridLayout()), style->standardIcon(QStyle::SP_FileDialogListView), QKeySequence(Qt::ControlModifier & Qt::Key_G), tr("Customize grid layout"));
    viewMenu->addAction(gridLayoutAction);
    // END View
    //

    // BEGIN Planning
    QMenu* planningMenu = new QMenu(QObject::tr("&Planning"));
    QAction* runPlanner = comm.addAction("Run planner", SLOT(runPlanner()), style->standardIcon(QStyle::SP_MediaPlay), QKeySequence(Qt::ControlModifier + Qt::Key_P), tr("Run planner"));
    QAction* stopPlanner = comm.addAction("Stop planner", SLOT(stopPlanner()), style->standardIcon(QStyle::SP_MediaStop), QKeySequence(Qt::ControlModifier + Qt::Key_H), tr("Stop planner"));
    QAction* loadConfiguration = comm.addAction("Load configuration", SLOT(loadConfiguration()), style->standardIcon(QStyle::SP_FileDialogDetailedView), QKeySequence(Qt::ControlModifier & Qt::Key_L & Qt::Key_C));
    QAction* editConfiguration = comm.addAction("Edit configuration", SLOT(editConfiguration()), style->standardIcon(QStyle::SP_FileDialogDetailedView), QKeySequence(Qt::ControlModifier & Qt::Key_E & Qt::Key_C));


    planningMenu->addAction(runPlanner);
    planningMenu->addAction(stopPlanner);
    planningMenu->addAction(loadConfiguration);
    planningMenu->addAction(editConfiguration);

    // END Planning

    // BEGIN Windows
    // BEGIN Windows->Dockable Dialogs
    QMenu* windowsMenu = new QMenu(QObject::tr("&Windows"));
    QMenu* viewDockWidgetMenu = windowsMenu->addMenu("&Dockable Dialogs");

    QAction* widgetLeft = mpUi->dockWidgetLeft->toggleViewAction();
    viewDockWidgetMenu->addAction(widgetLeft);
    mpUi->dockWidgetLeft->hide();

    QAction* widgetRight = mpUi->dockWidgetRight->toggleViewAction();
    viewDockWidgetMenu->addAction(widgetRight);
    mpUi->dockWidgetRight->hide();

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
    bar->addMenu(planningMenu);
    bar->addMenu(windowsMenu);

    QToolBar* toolBar = new QToolBar("Toolbar");
    toolBar->addAction(actionImport);
    toolBar->addAction(actionExport);
    toolBar->addAction(selectLayout);
    toolBar->addAction(runPlanner);
    toolBar->addAction(stopPlanner);
    toolBar->setFloatable(true);
    addToolBar(toolBar);

    QMenu* recentFilesMenu = createRecentFilesMenu();
    fileMenu->addMenu(recentFilesMenu);
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
            eManager->registerVisualization(e->getClassName(), new edge_items::CapacityLinkItem()); //RoleInfoItem());
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
    updateRecentFileActions("IOSolutions");
    updateRecentFileActions("IOMissions");
    updateRecentFileActions("IOGraphs");
    updateRecentFileActions("IOConfiguration");
}

void TemplGui::updateRecentFileActions(const QString& label)
{
    QSettings settings(QCoreApplication::organizationName(), label);
    if( !mpRecentFileActionsMap.count(label) )
    {
        QMessageBox::warning(this, "Templ", "Update recent file actions failing for label " + label + " please inform developer");
        return;
    }
    QList<QAction*> recentFileActions = mpRecentFileActionsMap[label];

    // The subproperty: recentImportFileList will be dynamically interpreted to
    // populate the recent files menu
    QStringList files = settings.value("recentImportFileList").toStringList();
    int numRecentFiles = qMin(files.size(), (int) MaxRecentFiles);
    for(int i = 0; i < numRecentFiles; ++i)
    {
        QString text = tr("&%1 %2").arg(i + 1).arg(files[i]);
        recentFileActions[i]->setText(text);
        recentFileActions[i]->setData(files[i]);
        recentFileActions[i]->setVisible(true);
    }

    for(int j = numRecentFiles; j < MaxRecentFiles; ++j)
    {
        recentFileActions[j]->setVisible(false);
    }
}

QString TemplGui::strippedName(const QString& fullFileName)
{
    return QFileInfo(fullFileName).fileName();
}

void TemplGui::importGraph(const QString& settingsLabel)
{
    graph_analysis::BaseGraph::Ptr graph = graph_analysis::gui::dialogs::IODialog::importGraph(this,
            QString(),
            settingsLabel);
    activateGraph(graph);
}

void TemplGui::importSolution(const QString& settingsLabel, const QString& filename)
{
    qDebug() << "Import solution from " << filename;
    graph_analysis::BaseGraph::Ptr graph = graph_analysis::gui::dialogs::IODialog::importGraph(this,
            filename,
            settingsLabel);
    if(!graph)
    {
        qDebug() << "Import solution failed for" << filename;
        return;
    }

    MissionView* missionView = qobject_cast<templ::gui::MissionView*>(mpUi->tabWidget->currentWidget());
    if(missionView && missionView->getMission())
    {
        try {
            Mission::Ptr mission = missionView->getMission();
            SpaceTime::Network network = SpaceTime::Network::fromGraph(graph,
                    mission->getLocations(), mission->getTimepoints());

            // FIXME: mission loading should account for configuration setting
            organization_model::OrganizationModelAsk ask(mission->getOrganizationModel()
                    , mission->getAvailableResources()
                    , true
                    , mConfiguration.getValueAs<double>("TransportNetwork/search/options/connectivity/timeout_in_s",
                        20)*1000
                    , mConfiguration.getValue("TransportNetwork/search/options/connectivity/interface-type"
                         , organization_model::vocabulary::OM::ElectroMechanicalInterface().toString())
                    );
            mission->setOrganizationModelAsk(ask);
            solvers::SolutionAnalysis sa(mission, network);
            sa.analyse();
            graph = sa.getSolutionNetwork().getGraph();
        } catch(const std::exception& e)
        {
            QMessageBox::warning(this, "Templ", QString(e.what()));
            return;
        }
    } else {
        QMessageBox::warning(this, "Templ", QString("No active mission view, will import solution as graph without annotations"));
    }

    if(!mpsOrganizationModel)
    {
        QMessageBox::warning(this, "Templ", QString("No active organization model -- please select one"));
        importOrganizationModel();
    }

    activateGraph(graph);
}

void TemplGui::importMission(const QString& settingsLabel, const QString& filename)
{
    MissionEditor* missionEditor = qobject_cast<templ::gui::MissionEditor*>(mpUi->tabWidget->currentWidget());
    if(missionEditor)
    {
        if(!missionEditor->loadMission(settingsLabel, filename))
        {
            return;
        }

        int tabIndex = mpUi->tabWidget->indexOf(missionEditor);
        mpUi->tabWidget->setTabToolTip(tabIndex, missionEditor->getFilename());
        mpUi->tabWidget->setTabWhatsThis(tabIndex, missionEditor->getFilename());

        QFileInfo fileInfo(missionEditor->getFilename());
        mpUi->tabWidget->setTabText(tabIndex, QString("MissionEditor: ") + fileInfo.baseName());

        // Update organization model upon import
        // TODO: use global session setup/active mission to import solution?
        mpsOrganizationModel = missionEditor->getMission()->getOrganizationModel();
    } else {
        openMissionEditor(settingsLabel, filename);
    }
}


//void TemplGui::importMission(const QString& settingsLabel, const QString& filename)
//{
//    MissionView* missionView = qobject_cast<templ::gui::MissionView*>(mpUi->tabWidget->currentWidget());
//    if(missionView)
//    {
//        if(!missionView->loadMission(settingsLabel, filename))
//        {
//            return;
//        }
//
//        int tabIndex = mpUi->tabWidget->indexOf(missionView);
//        mpUi->tabWidget->setTabToolTip(tabIndex, missionView->getFilename());
//        mpUi->tabWidget->setTabWhatsThis(tabIndex, missionView->getFilename());
//
//        QFileInfo fileInfo(missionView->getFilename());
//        mpUi->tabWidget->setTabText(tabIndex, QString("MissionView: ") + fileInfo.baseName());
//
//        // Update organization model upon import
//        // TODO: use global session setup/active mission to import solution?
//        mpsOrganizationModel = missionView->getMission()->getOrganizationModel();
//    } else {
//        openMissionView(settingsLabel, filename);
//    }
//
//}

void TemplGui::importOrganizationModel()
{
    QSettings settings(QCoreApplication::organizationName(), "organization_model");
    QString iriSetting = settings.value("iri").toString();

    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load organization model"),
        QDir::currentPath(),
        tr("Organization Model File (*.xml *.owl)"));

    mpsOrganizationModel = organization_model::OrganizationModel::getInstance(filename.toStdString());

    settings.setValue("iri", mpsOrganizationModel->ontology()->getIRI().toString().c_str());
}

void TemplGui::importRecentOrganizationModel()
{
    QSettings settings(QCoreApplication::organizationName(), "organization_model");
    QString iriSetting = settings.value("iri").toString();
    if(iriSetting.isEmpty())
    {
        QMessageBox::warning(this, "Templ", QString("OrganizationModel: not recent model detected"));
    } else {
        owlapi::model::IRI iri(iriSetting.toStdString());
        mpsOrganizationModel = organization_model::OrganizationModel::getInstance(iri);
    }
}

void TemplGui::activateGraph(graph_analysis::BaseGraph::Ptr& graph, const QString& tabLabel)
{
    graph_analysis::gui::BaseGraphView* view = new graph_analysis::gui::BaseGraphView(graph, this);
    if(graph)
    {
        view->refresh();
        view->updateVisualization();
        view->applyLayout("dot");
        view->updateVisualization();

        connect(view, SIGNAL(currentStatus(QString, int)),
                mpUi->statusbar, SLOT(showMessage(QString, int)));

        mpUi->tabWidget->addTab(view, tabLabel);
        mBaseGraphViews.push_back(view);
    } else {
        qDebug() << "Failed to activate graph";
    }


}

void TemplGui::exportGraph()
{
    using namespace graph_analysis::gui;
    BaseGraphView* view = getCurrentBaseGraphView();
    if(view)
    {
        graph_analysis::gui::dialogs::IODialog::exportGraph(view->graph());
    } else {
        QMessageBox::warning(this, tr("Templ"), tr("No active tab with a graph"));
    }
}

void TemplGui::importRecentGraph(const QString& settingsLabel)
{
    QAction *action = qobject_cast<QAction*>(sender());
    if(action)
    {
        qDebug() << "Importing file from: " << action->data().toString();
        graph_analysis::BaseGraph::Ptr graph = graph_analysis::gui::dialogs::IODialog::importGraph(this, action->data().toString(), settingsLabel);

        activateGraph(graph, "Graph");
    }
}

void TemplGui::importRecentMission()
{
    QAction *action = qobject_cast<QAction*>(sender());
    if(action)
    {
        QString filename = action->data().toString();
        qDebug() << "Importing mission file from: " << filename;
        importMission("IOMissions", filename);
    }
}

void TemplGui::importRecentSolution()
{
    QAction *action = qobject_cast<QAction*>(sender());
    if(action)
    {
        QString filename = action->data().toString();
        qDebug() << "Importing solution from: " << filename;
        importSolution("IOSolutions", filename);
    }
}

void TemplGui::importRecentConfiguration()
{
    QAction *action = qobject_cast<QAction*>(sender());
    if(action)
    {
        QString filename = action->data().toString();
        qDebug() << "Importing configuration file from: " << filename;
        loadConfiguration(filename, "IOConfiguration");
    }
}

void TemplGui::clearView()
{
    using namespace graph_analysis::gui;
    BaseGraphView* view = getCurrentBaseGraphView();
    if(view)
    {
        view->clearVisualization();
    } else {
        QMessageBox::warning(this, tr("Templ"), tr("No active tab with a graph"));
    }
}

void TemplGui::selectLayout()
{
    using namespace graph_analysis::gui;
    BaseGraphView* view = getCurrentBaseGraphView();
    if(view)
    {
        bool ok;
        QString desiredLayout = QInputDialog::getItem(this, tr("Select Layout"),
                                    tr("select a layout:"), view->getSupportedLayouts(),
                                    0, false, &ok);
        if(ok)
        {
            view->applyLayout(desiredLayout.toStdString());
        }
        updateVisualization();
    } else {
        QMessageBox::warning(this, tr("Templ"), tr("No active tab with a graph"));
    }
}

void TemplGui::customizeGridLayout()
{
    graph_analysis::gui::dialogs::GridLayout::execute();
}

void TemplGui::updateVisualization()
{
    // Call the current tab widget's update function
    graph_analysis::gui::BaseGraphView* view = getCurrentBaseGraphView();
    if(view)
    {
        view->updateVisualization();
    } else if (mpUi->tabWidget->currentWidget() == mpMissionEditor)
    {
        mpMissionEditor->update();
    }
}

void TemplGui::closeTab(int tabIndex)
{
    using namespace graph_analysis::gui;
    QWidget* widget = mpUi->tabWidget->widget(tabIndex);
    BaseGraphView* view = qobject_cast<BaseGraphView*>(widget);
    if(view)
    {
        std::vector<BaseGraphView*>::iterator it = std::find(mBaseGraphViews.begin(), mBaseGraphViews.end(), view);
        mBaseGraphViews.erase(it);
    }
    if(qobject_cast<MissionView*>(widget))
    {
        mpMissionView = NULL;
    } else if(qobject_cast<MissionEditor*>(widget))
    {
        mpMissionEditor = NULL;
    } else if(qobject_cast<OntologyView*>(widget))
    {
        mpOntologyView = NULL;
    }

    delete widget;
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

    // We can assume that edges always point forward in time -- thus the
    // network implicitly defined the temporal network
    QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());

    size_t edgeCount = graph->size();
    EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
    while(edgeIt->next())
    {
        Edge::Ptr edge = edgeIt->current();

        // Add the temporal constraint
        SpaceTime::SpaceTimeTuple::Ptr sourceTuple =
            dynamic_pointer_cast<SpaceTime::SpaceTimeTuple>(edge->getSourceVertex());
        SpaceTime::SpaceTimeTuple::Ptr targetTuple =
            dynamic_pointer_cast<SpaceTime::SpaceTimeTuple>(edge->getTargetVertex());
        qtcn->addQualitativeConstraint(sourceTuple->second(),
                targetTuple->second(), point_algebra::QualitativeTimePointConstraint::Less);
    }
    assert(csp::TemporalConstraintNetworkBase::isConsistent(*qtcn));

    point_algebra::TimePoint::PtrList sortedTimepoints =
        csp::TemporalConstraintNetworkBase::getSortedList(*qtcn);

    graph_analysis::gui::layouts::GridLayout::RowLabels sortedLabels;
    for(point_algebra::TimePoint::Ptr& tp : sortedTimepoints)
    {
        sortedLabels.push_back(tp->getLabel());
    }
    labels = sortedLabels;
}

void TemplGui::exportScene()
{
    graph_analysis::gui::BaseGraphView* view = getCurrentBaseGraphView();
    if(view)
    {
        graph_analysis::gui::dialogs::IODialog::exportScene(view->scene(), this);
    } else {
        QMessageBox::warning(this, tr("Templ"), tr("No active tab with a graph"));
    }
}

void TemplGui::clearRecentFileList(const QString& name)
{
    QSettings settings(QCoreApplication::organizationName(), name);
    QStringList files;
    settings.setValue("recentImportFileList", files);
    updateRecentFileActions();
}

QMenu* TemplGui::createRecentFilesMenu()
{
    ActionCommander comm(this);
    QCommonStyle style;

    QMenu* recentFilesMenu = new QMenu(QObject::tr("&Recent Files"));
    QList<QString> recentFileList;
    recentFileList.append("Graphs");
    recentFileList.append("Missions");
    recentFileList.append("Solutions");
    recentFileList.append("Configuration");

    foreach(const QString& name, recentFileList)
    {
        QMenu* recentFilesSubMenu = new QMenu("&" + name);
        QList<QAction*>& subMenuActionList = mpRecentFileActionsMap["IO" + name];

        // Populate the recent files list
        for(int i = 0; i < MaxRecentFiles; ++i)
        {
            QAction* action = new QAction(this);
            action->setVisible(false);
            subMenuActionList.push_back(action);
            if(name == "Missions")
            {
                connect(action, SIGNAL(triggered()), this, SLOT(importRecentMission()));
            } else if(name == "Solutions")
            {
                connect(action, SIGNAL(triggered()), this, SLOT(importRecentSolution()));
            } else if(name == "Configuration")
            {
                connect(action, SIGNAL(triggered()), this, SLOT(importRecentConfiguration()));
            } else {
                connect(action, SIGNAL(triggered()), this, SLOT(importRecentGraph()));
            }
        }

        QAction* clearRecentFiles = NULL;
        if(name == "Missions")
        {
            clearRecentFiles = comm.addAction("Clear list", SLOT(clearRecentMissions()), style.standardIcon(QStyle::SP_TrashIcon), QKeySequence(Qt::ControlModifier & Qt::Key_C), tr("Clear recent files"));
        } else if(name == "Solutions")
        {
            clearRecentFiles = comm.addAction("Clear list", SLOT(clearRecentSolutions()), style.standardIcon(QStyle::SP_TrashIcon), QKeySequence(Qt::ControlModifier & Qt::Key_C), tr("Clear recent files"));
        } else if(name == "Configuration")
        {
            clearRecentFiles = comm.addAction("Clear list", SLOT(clearRecentConfigurations()), style.standardIcon(QStyle::SP_TrashIcon), QKeySequence(Qt::ControlModifier & Qt::Key_C), tr("Clear recent files"));
        } else {
            clearRecentFiles = comm.addAction("Clear list", SLOT(clearRecentGraphs()), style.standardIcon(QStyle::SP_TrashIcon), QKeySequence(Qt::ControlModifier & Qt::Key_C), tr("Clear recent files"));
        }

        recentFilesSubMenu->addSeparator();
        recentFilesSubMenu->addAction(clearRecentFiles);

        int existingRecentFiles = qMin(subMenuActionList.size(), (int) MaxRecentFiles);
        for(int i = 0; i < existingRecentFiles; ++i)
        {
            recentFilesSubMenu->addAction(subMenuActionList[i]);
        }
        recentFilesMenu->addMenu(recentFilesSubMenu);
    }

    connect(recentFilesMenu, SIGNAL(aboutToShow()), this, SLOT(updateRecentFileActions()));
    return recentFilesMenu;
}

graph_analysis::gui::BaseGraphView* TemplGui::getCurrentBaseGraphView() const
{
    return qobject_cast<graph_analysis::gui::BaseGraphView*>(mpUi->tabWidget->currentWidget());
}

void TemplGui::openMissionView(const QString& settingsLabel, const QString& filename)
{
    MissionView* missionView = new MissionView;
    mpUi->tabWidget->addTab(missionView,
                            "Mission View");
    mpUi->tabWidget->setCurrentWidget(missionView);
    // and show both' widgets status-messages on the statusbar. this simply
    // assumes that only the one in the front is sending updates. otherwise
    // they would interleave...
    connect(missionView, SIGNAL(currentStatus(QString, int)),
            mpUi->statusbar, SLOT(showMessage(QString, int)));

    importMission(settingsLabel, filename);
}

void TemplGui::openMissionEditor(const QString& settingsLabel, const QString& filename)
{
    MissionEditor* missionEditor = new MissionEditor;
    mpUi->tabWidget->addTab(missionEditor,
                            "Mission Editor");
    mpUi->tabWidget->setCurrentWidget(missionEditor);
    // and show both' widgets status-messages on the statusbar. this simply
    // assumes that only the one in the front is sending updates. otherwise
    // they would interleave...
    connect(missionEditor, SIGNAL(currentStatus(QString, int)),
            mpUi->statusbar, SLOT(showMessage(QString, int)));

    importMission(settingsLabel, filename);
}

void TemplGui::openOntologyView()
{
    if(mpOntologyView)
    {
        QMessageBox::warning(this, "Templ", "Ontology view is already opened");
    } else {
        mpOntologyView = new OntologyView(this);
        mpUi->tabWidget->addTab(mpOntologyView,
                                mpOntologyView->getClassName());
        mpUi->tabWidget->setCurrentWidget(mpOntologyView);
    }
}

void TemplGui::runPlanner()
{
    QString program = "templ-transport_network_planner";
    QStringList arguments;

    // Get current configuration
    arguments << "--configuration";
    std::string path = mConfiguration.saveTemp("templ-gui-config");
    arguments << QString(path.c_str());

    MissionView* missionView = qobject_cast<templ::gui::MissionView*>(mpUi->tabWidget->currentWidget());
    if(!missionView)
    {
        QMessageBox::warning(this, "Templ", "No active mission view selected for plan execution");
        return;
    }

    // Get current mission
    arguments << "--mission";
    arguments << missionView->getFilename();

    arguments << "--min_solutions";
    arguments << QString( mConfiguration.getValue("templ-gui/min_solutions","2").c_str() );

    arguments << "--interactive";
    arguments << "false";

    qDebug() << "Arguments: " << arguments;

    // Kill any already running process before starting a new one
    if(mpProcess && mpProcess->state() == QProcess::Running)
    {
        qDebug() << "Killing running process before starting a new planning process";
        mpProcess->kill();
    }

    mpProcess = new QProcess(this);
    connect(mpProcess, SIGNAL(readyReadStandardOutput()),
            this, SLOT(logOutput()));

    mpProcess->start(program, arguments);
    if(!mpProcess->waitForStarted())
    {
        QMessageBox::warning(this, "Templ", "Failed to start planner");
    } else {
        qDebug() << "Planner started";
    }
}

void TemplGui::stopPlanner()
{
    if(mpProcess && mpProcess->state() == QProcess::Running)
    {
        qDebug() << "Killing current planning process";
        mpProcess->kill();
    } else {
        qDebug() << "No running planning process";
    }
}

void TemplGui::loadConfiguration(const QString& _filename,
        const QString& settingsLabel)
{
    QSettings settings(QCoreApplication::organizationName(), settingsLabel);
    QString filename = _filename;
    if(filename.isEmpty() || !QFileInfo(filename).exists())
    {
        QString dir = QDir::currentPath();

        QString dirValue = settings.value("recentImportFileList").toString();
        if(!dirValue.isEmpty())
        {
            dir = dirValue;
        }

        filename = QFileDialog::getOpenFileName(
            this, tr("Load configuration"),
            dir,
            tr("Configuration File (*.qxcfg *.xml)"));
    }

    if(!filename.isEmpty())
    {
        QFileInfo fileinfo(filename);
        // update recent files list
        QStringList files = settings.value("recentImportFileList").toStringList();
        files.removeAll(filename);
        files.prepend(filename);
        while(files.size() > 10)
        {
            files.removeLast();
        }
        settings.setValue("recentImportFileList", files);

        try {
            mConfiguration = qxcfg::Configuration(filename.toStdString());
            qDebug() << "Loaded configuration: " << mConfiguration.toString().c_str();
        } catch(const std::exception& e)
        {
            QMessageBox::warning(this, "Templ", QString("Configuration: failed to load file --") + QString(e.what()));
        }
    }
}

void TemplGui::editConfiguration()
{
    QMessageBox::warning(this, "Templ", QString("Configuration: editing the configuration is not yet supported"));
}

void TemplGui::logOutput()
{
    QString output(mpProcess->readAll());
    mpUi->textBrowser->append(output);
}

} // end namespace gui
} // end namespace templ
