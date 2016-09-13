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
MissionEditor::MissionEditor(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::MissionEditor)
{
    mpUi->setupUi(this);

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
