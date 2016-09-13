#include "TemplGui.hpp"

#include "ui_TemplGui.h"

#include <graph_analysis/gui/BaseGraphView/BaseGraphView.hpp>
#include <graph_analysis/VertexTypeManager.hpp>

#include <QDebug>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QCommonStyle>

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
    , mpOntologyView(new OntologyView(this))
{
    mpUi->setupUi(this);
    mpUi->tabWidget->clear();
    mpUi->tabWidget->addTab(mpBaseGraphView, mpBaseGraphView->getClassName());
    mpUi->tabWidget->addTab(mpMissionEditor,
                            mpMissionEditor->getClassName());
    mpUi->tabWidget->addTab(mpOntologyView,
                            mpOntologyView->getClassName());
    mpUi->tabWidget->setCurrentWidget(mpMissionEditor);

    // and show both' widgets status-messages on the statusbar. this simply
    // assumes that only the one in the front is sending updates. otherwise
    // they would interleave...

    //connect(mpQBaseGraph, SIGNAL(graphChanged()),
    //        this, SLOT(updateVisualization()));


    QMenuBar *bar = menuBar();

    ActionCommander comm(this);

    QMenu *fileMenu = new QMenu(QObject::tr("&File"));
    QStyle* style = new QCommonStyle();
    QAction *actionImport = comm.addAction("Import", SLOT(importGraph()), style->standardIcon(QStyle::SP_FileIcon)        , QKeySequence( QKeySequence::Open ), tr("Import graph from file"));
    QAction *actionExport = comm.addAction("Export", SLOT(exportGraph()), style->standardIcon(QStyle::SP_DialogSaveButton), QKeySequence( QKeySequence::SaveAs), tr("Export graph to file"));

    fileMenu->addAction(actionImport);
    fileMenu->addAction(actionExport);
    fileMenu->addSeparator();

    QToolBar* toolBar = new QToolBar("Toolbar");
    toolBar->addAction(actionImport);
    toolBar->addAction(actionExport);
    toolBar->setFloatable(true);
    addToolBar(toolBar);
}

TemplGui::~TemplGui()
{
    delete mpUi;
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
        io::GraphIO::SuffixMap::const_iterator sit = suffixMap.begin();
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
        updateVisualization();
    }
    else
    {
        /* updateStatus("Failed to import graph: aborted by user - an empty
         * input filename was provided"); */
    }
}

void TemplGui::exportGraph()
{
    if(mpQBaseGraph->getBaseGraph()->empty())
    {
        QMessageBox::critical(this, tr("Graph Export Failed"),
                              "Graph is empty");
        return;
    }

    QString selectedFilter;

    // Constructing the writer suffix filter
    io::GraphIO::SuffixMap suffixMap = io::GraphIO::getSuffixMap();
    io::GraphIO::WriterMap writerMap = io::GraphIO::getWriterMap();
    io::GraphIO::WriterMap::const_iterator wit = writerMap.begin();

    std::stringstream ss;
    for(;;)
    {
        ss << representation::TypeTxt[wit->first] << " (";
        io::GraphIO::SuffixMap::const_iterator sit = suffixMap.begin();
        for(; sit != suffixMap.end(); ++sit)
        {
            if(sit->second == wit->first)
            {
                ss << "*." << sit->first << " ";
            }
        }
        ss << ")";

        ++wit;
        if(wit != writerMap.end())
        {
            ss << ";;";
        }
        else
        {
            break;
        }
    }
    // End constructing the writer suffix filter

    dialogs::ExportFile dialog(ss.str().c_str());
    if(dialog.exec() == QFileDialog::Accepted)
    {
        try
        {
            io::GraphIO::write(dialog.getFilename().toStdString(), mpQBaseGraph->getBaseGraph(),
                               dialog.getTypeName());
        }
        catch(const std::exception& e)
        {
            std::string msg = "Export of graph to '" +
                              dialog.getFilename().toStdString() + "' failed " +
                              e.what();
            QMessageBox::critical(this, tr("Graph Export Failed"), msg.c_str());
            return;
        }
    }
    else
    {
        /* updateStatus("Exporting graph aborted by user"); */
    }
}

void TemplGui::fromFile(const std::string& filename)
{
    graph_analysis::BaseGraph::Ptr graph = graph_analysis::BaseGraph::getInstance();
    try
    {
        io::GraphIO::read(filename, graph);
    }
    catch(const std::exception& e)
    {
        std::string msg = "Failed to import '" + filename + "': " + e.what();
        QMessageBox::critical(this, tr("Graph Import Failed"), msg.c_str());
        return;
    }
    delete mpQBaseGraph;
    delete mpBaseGraphView;

    mpBaseGraph = graph;
    mpQBaseGraph = new QBaseGraph(mpBaseGraph);
    mpBaseGraphView = new graph_analysis::gui::BaseGraphView(mpBaseGraph, this);
    assert(mpQBaseGraph);
    assert(mpBaseGraphView);
}

void TemplGui::on_tabWidget_currentChanged(int index)
{
    // When the tab changed, we want to update the widget
    this->updateVisualization();
}

void TemplGui::updateVisualization()
{
    // Call the current tab widget's update function
    //if (mpUi->tabWidget->currentWidget() == mpMissionEditor)
    //{
    //    mpMissionEditor->updateVisualization();
    //} else {
    LOG_WARN_S << "Populate canvas";
    assert(mpBaseGraphView);
    mpBaseGraphView->update();
    //}
}

} // end namespace gui
} // end namespace templ
