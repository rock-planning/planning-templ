#include "TemplGui.hpp"

#include "ui_TemplGui.h"

#include <graph_analysis/gui/BaseGraphView/BaseGraphView.hpp>
#include <graph_analysis/gui/VertexItemTypeManager.hpp>
#include <graph_analysis/gui/EdgeItemTypeManager.hpp>

#include <QDebug>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QCommonStyle>
#include <QInputDialog>
#include <QGraphicsGridLayout>

#include <base-logging/Logging.hpp>

#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/io/YamlWriter.hpp>
#include <graph_analysis/io/GexfWriter.hpp>
#include <graph_analysis/io/GexfReader.hpp>
#include <graph_analysis/io/YamlReader.hpp>
#include <graph_analysis/io/GraphvizWriter.hpp>
#include <graph_analysis/gui/GraphWidget.hpp>
#include <graph_analysis/gui/dialogs/ExportFile.hpp>
#include <graph_analysis/gui/ActionCommander.hpp>

#include <templ/gui/MissionEditor/MissionEditor.hpp>
#include <templ/gui/MissionView/MissionView.hpp>
#include <templ/gui/OntologyView/OntologyView.hpp>

#include <templ/SpaceTime.hpp>
#include <templ/RoleInfoWeightedEdge.hpp>
#include <templ/gui/edge_items/RoleInfoItem.hpp>
#include <templ/gui/vertex_items/RoleInfoItem.hpp>

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
    mpUi->tabWidget->addTab(mpBaseGraphView, mpBaseGraphView->getClassName());
    mpUi->tabWidget->addTab(mpMissionEditor,
                            mpMissionEditor->getClassName());
    mpUi->tabWidget->addTab(mpMissionView,
                            "Mission view");
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


    QMenuBar *bar = menuBar();

    ActionCommander comm(this);

    QMenu *fileMenu = new QMenu(QObject::tr("&File"));
    QStyle* style = new QCommonStyle();
    QAction *actionImport = comm.addAction("Import", SLOT(importGraph()), style->standardIcon(QStyle::SP_FileIcon)        , QKeySequence( QKeySequence::Open ), tr("Import graph from file"));
    QAction *actionExport = comm.addAction("Export", SLOT(exportGraph()), style->standardIcon(QStyle::SP_DialogSaveButton), QKeySequence( QKeySequence::SaveAs), tr("Export graph to file"));
    QAction *selectLayout = comm.addAction("Layout", SLOT(selectLayout()), style->standardIcon(QStyle::SP_FileDialogListView), QKeySequence( Qt::ControlModifier & Qt::Key_L), tr("Export graph to file"));

    fileMenu->addAction(actionImport);
    fileMenu->addAction(actionExport);
    fileMenu->addAction(selectLayout);
    fileMenu->addSeparator();

    QToolBar* toolBar = new QToolBar("Toolbar");
    toolBar->addAction(actionImport);
    toolBar->addAction(actionExport);
    toolBar->addAction(selectLayout);
    toolBar->setFloatable(true);
    addToolBar(toolBar);

    registerGraphElementTypes();
}

TemplGui::~TemplGui()
{
    delete mpUi;
}

void TemplGui::registerGraphElementTypes()
{
    using namespace templ;
    using namespace graph_analysis;
    // Edges
    RoleInfoWeightedEdge::Ptr e(new RoleInfoWeightedEdge());

    graph_analysis::gui::EdgeItemTypeManager* eManager = graph_analysis::gui::EdgeItemTypeManager::getInstance();
    eManager->registerVisualization(e->getClassName(), new edge_items::RoleInfoItem());


    SpaceTime::Network::tuple_t::Ptr v(new SpaceTime::Network::tuple_t());

    graph_analysis::gui::VertexItemTypeManager* vManager = graph_analysis::gui::VertexItemTypeManager::getInstance();
    vManager->registerVisualization(v->getClassName(), new vertex_items::RoleInfoItem());

}

void TemplGui::importGraph()
{
    QString selectedFilter;
    // Constructing the writer suffix filter
    graph_analysis::io::GraphIO::SuffixMap suffixMap = graph_analysis::io::GraphIO::getSuffixMap();
    graph_analysis::io::GraphIO::ReaderMap readerMap = graph_analysis::io::GraphIO::getReaderMap();
    graph_analysis::io::GraphIO::ReaderMap::const_iterator rit = readerMap.begin();

    std::stringstream ss;
    for(;;)
    {
        ss << representation::TypeTxt[rit->first] << " (";
        graph_analysis::io::GraphIO::SuffixMap::const_iterator sit = suffixMap.begin();
        for(; sit != suffixMap.end(); ++sit)
        {
            if(sit->second == rit->first)
            {
                ss << "*." << sit->first << " ";
            }
        }
        ss << ")";

        ++rit;
        if(rit != readerMap.end())
        {
            ss << ";;";
        }
        else
        {
            break;
        }
    }
    // End constructing the writer suffix filter

    QString filename = QFileDialog::getOpenFileName(
        this, tr("Choose input file"), QDir::currentPath(),
        tr(ss.str().c_str()), &selectedFilter);

    if(!filename.isEmpty())
    {
        fromFile(filename.toStdString());
        //updateVisualization();
    }
    else
    {
        /* updateStatus("Failed to import graph: aborted by user - an empty
         * input filename was provided"); */
    }
}

void TemplGui::exportGraph()
{
    using namespace graph_analysis;

    //if(mpQBaseGraph->getBaseGraph()->empty())
    //{
    //    QMessageBox::critical(this, tr("Graph Export Failed"),
    //                          "Graph is empty");
    //    return;
    //}

    //QString selectedFilter;

    //// Constructing the writer suffix filter
    //io::GraphIO::SuffixMap suffixMap = io::GraphIO::getSuffixMap();
    //io::GraphIO::WriterMap writerMap = io::GraphIO::getWriterMap();
    //io::GraphIO::WriterMap::const_iterator wit = writerMap.begin();

    //std::stringstream ss;
    //for(;;)
    //{
    //    ss << representation::TypeTxt[wit->first] << " (";
    //    io::GraphIO::SuffixMap::const_iterator sit = suffixMap.begin();
    //    for(; sit != suffixMap.end(); ++sit)
    //    {
    //        if(sit->second == wit->first)
    //        {
    //            ss << "*." << sit->first << " ";
    //        }
    //    }
    //    ss << ")";

    //    ++wit;
    //    if(wit != writerMap.end())
    //    {
    //        ss << ";;";
    //    }
    //    else
    //    {
    //        break;
    //    }
    //}
    //// End constructing the writer suffix filter

    //dialogs::ExportFile dialog(ss.str().c_str());
    //if(dialog.exec() == QFileDialog::Accepted)
    //{
    //    try
    //    {
    //        io::GraphIO::write(dialog.getFilename().toStdString(), mpQBaseGraph->getBaseGraph(),
    //                           dialog.getTypeName());
    //    }
    //    catch(const std::exception& e)
    //    {
    //        std::string msg = "Export of graph to '" +
    //                          dialog.getFilename().toStdString() + "' failed " +
    //                          e.what();
    //        QMessageBox::critical(this, tr("Graph Export Failed"), msg.c_str());
    //        return;
    //    }
    //}
    //else
    //{
    //    /* updateStatus("Exporting graph aborted by user"); */
    //}
}

void TemplGui::fromFile(const std::string& filename)
{
    graph_analysis::BaseGraph::Ptr graph = graph_analysis::BaseGraph::getInstance();
    try
    {
        graph_analysis::io::GraphIO::read(filename, graph);
    }
    catch(const std::exception& e)
    {
        std::string msg = "Failed to import '" + filename + "': " + e.what();
        QMessageBox::critical(this, tr("Graph Import Failed"), msg.c_str());
        return;
    }

    mpBaseGraph = graph;

    delete mpQBaseGraph;
    mpQBaseGraph = new QBaseGraph(mpBaseGraph);

    mpBaseGraphView->setGraph(mpBaseGraph);
    mpBaseGraphView->clearVisualization();
    mpBaseGraphView->refresh();
    mpBaseGraphView->updateVisualization();
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

} // end namespace gui
} // end namespace templ
