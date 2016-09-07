#include "MissionEditor.hpp"
#include <templ/io/MissionReader.hpp>

// QT specific includes
#include "ui_MissionEditor.h"
#include <QDirIterator>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <QtDebug>

// Rock specific includes
#include <base-logging/Logging.hpp>
#include <graph_analysis/gui/RegisterQtMetatypes.hpp>

namespace templ {
namespace gui {
MissionEditor::MissionEditor(graph_analysis::BaseGraph::Ptr graph,
                                 QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::MissionEditor)
    , mpGraph(graph)
{
    mpUi->setupUi(this);

    //mpTaskGraphViewer = new TaskGraphViewer(mpGraph);
    //mpUi->placeHolder->addWidget(mpTaskGraphViewer);
    //mpUi->splitter->setSizes(QList<int>() << 10 << 1000);
    //mpUi->taskTemplateTree->sortByColumn(0, Qt::AscendingOrder);

    //connect(mpTaskGraphViewer, SIGNAL(currentStatus(QString, int)), this,
    //        SLOT(currentStatus_internal(QString, int)));

    //connect(&mLauncher, SIGNAL(finished()), this,
    //        SLOT(launcher_execution_finished()));
    //connect(&mLauncher, SIGNAL(started()), this,
    //        SLOT(launcher_execution_started()));

    //connect(mpUi->taskTemplateTree,
    //        SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
    //        mpUi->taskPreview,
    //        SLOT(updatePreview(QTreeWidgetItem*, QTreeWidgetItem*)));
    //
    connect(mpUi->planMission,
            SIGNAL(clicked()),
            this,
            SLOT(on_planMission_clicked()));

    //// Get all system environment variables
    //QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

    //// Check whether the environment variable to the INSTALL directory exists,
    //// i.e. whether the env.sh has been sourced or not
    //if(env.contains("AUTOPROJ_CURRENT_ROOT"))
    //{
    //    // Expand the environment variable and add the folders to the yml models
    //    QString PathToYmlModels =
    //        qgetenv("AUTOPROJ_CURRENT_ROOT") + "/install/share/orogen/models";

    //    // Log the anomaly
    //    LOG_INFO("Checking for orogen yml files in: %s\n",
    //             PathToYmlModels.toUtf8().constData());

    //    // Check if the folder to the yml model files exists
    //    if(QDir(PathToYmlModels).exists())
    //    {
    //        // Get an iterator over all elements in model folder
    //        QDirIterator it(PathToYmlModels);
    //        while(it.hasNext())
    //        {
    //            it.next();
    //            if(it.fileInfo().suffix() == "yml")
    //            {
    //                addFile(it.fileInfo().absoluteFilePath());
    //            }
    //        }
    //    }
    //    else
    //    {
    //        // Log the anomaly
    //        LOG_WARN("The orogen yml model folder does not exist: %s\n",
    //                 PathToYmlModels.toUtf8().constData());

    //        // Notify the user
    //        QMessageBox* msgBox = new QMessageBox(this);
    //        msgBox->setAttribute(Qt::WA_DeleteOnClose);
    //        msgBox->setWindowTitle(tr("Folder not found"));
    //        msgBox->setText(tr("The orogen yml model folder does not exist!"));
    //        msgBox->setIcon(QMessageBox::Warning);
    //        msgBox->show();
    //    }
    //}
    //else
    //{
    //    // Log the anomaly
    //    LOG_WARN_S << "The 'AUTOPROJ_CURRENT_ROOT' has not been found - check "
    //                  "if 'env.sh' has been sourced.";

    //    // Notify the user
    //    QMessageBox* msgBox = new QMessageBox(this);
    //    msgBox->setAttribute(Qt::WA_DeleteOnClose);
    //    msgBox->setWindowTitle(tr("Missing Environment Variable"));
    //    msgBox->setText(
    //        tr("The 'AUTOPROJ_CURRENT_ROOT' has not been found - check "
    //           "if 'env.sh' has been sourced."));
    //    msgBox->setIcon(QMessageBox::Warning);
    //    msgBox->show();
    //}

    //// disable the "Start Launcher" button at first. will be initially enabled by
    //// pressing "save"
    //mpUi->updateButton->setDisabled(true);
    //mpUi->updateButton->setToolTip("Save and Submit to Launcher");
}

MissionEditor::~MissionEditor()
{
    delete mpUi;
}

void MissionEditor::updateVisualization()
{
}

void MissionEditor::on_loadMissionButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load mission description"),
        QDir::currentPath(),
        tr("Mission Description File (*.mdf *.xml)"));

    if(!filename.isEmpty())
    {
        templ::io::MissionReader reader;
        //reader.read(filename.toStdString(), mpGraph);
    }
}

void MissionEditor::on_loadOntologyButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load ontology"),
        QDir::currentPath(),
        tr("Mission Description File (*.xml)"));

    if(!filename.isEmpty())
    {
        // owlapi::MissionReader reader;
        //reader.read(filename.toStdString(), mpGraph);
    }
}

void MissionEditor::on_saveButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(
        this, tr("Store mission description"),
        QDir::currentPath(),
        tr("Component Network Definition File (*.mdf *.xml)"));

    if(!filename.isEmpty())
    {
        //templ::io::MissionWriter writer;
        // do write
    }
}

void MissionEditor::on_clearButton_clicked()
{
    // When this button is clicked we want to remove
    // every mission element
    std::vector<graph_analysis::Vertex::Ptr> vertices = mpGraph->getAllVertices();
    std::vector<graph_analysis::Vertex::Ptr>::const_iterator it;
    for (it = vertices.begin(); it != vertices.end(); ++it)
    {
        mpGraph->removeVertex(*it);
    }
}

void MissionEditor::on_addConstraintButton_clicked()
{
    LOG_INFO_S << "addConstraintButton clicked";
}

void MissionEditor::on_removeConstraintButton_clicked()
{
    LOG_INFO_S << "removeConstraintButton clicked";
}

void MissionEditor::on_planMission_clicked()
{
    LOG_DEBUG_S << "planMission clicked";
}

void MissionEditor::on_updateButton_clicked()
{
    LOG_DEBUG_S << "updateButton clicked";
}

} // end namespace gui
} // end namespace templ
