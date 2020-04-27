#include "MissionEditor.hpp"
#include "../../io/MissionReader.hpp"

// QT specific includes
#include "ui_MissionEditor.h"
#include <QDirIterator>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <QSettings>
#include <QtDebug>
#include <QSpacerItem>
#include <QCheckBox>

// Rock specific includes
#include <base-logging/Logging.hpp>
#include <graph_analysis/gui/RegisterQtMetatypes.hpp>

namespace templ {
namespace gui {
MissionEditor::MissionEditor(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::MissionEditor)
    , mLayoutResources(new QVBoxLayout)
{
    mpMission = make_shared<Mission>();

    mpUi->setupUi(this);
    mpUi->verticalLayoutResources->addLayout(mLayoutResources);

    connect(mpUi->pushButtonSelectOrganizationModel, SIGNAL(clicked()),
            this, SLOT(loadOrganizationModel()));
    connect(mpUi->pushButtonSave, SIGNAL(clicked()),
            this, SLOT(on_saveButton_clicked));
    connect(mpUi->pushButtonAddResource, SIGNAL(clicked()),
            this, SLOT(addResourceCardinality()));
    connect(mpUi->pushButtonDeleteResources, SIGNAL(clicked()),
            this, SLOT(removeResourceCardinalities()));
}

MissionEditor::~MissionEditor()
{
    delete mpUi;
}

bool MissionEditor::loadMission(const QString& settingsLabel, const QString& _filename)
{
    QSettings settings(QCoreApplication::organizationName(), settingsLabel);
    QString dir = QDir::currentPath();

    QString dirValue = settings.value("recentImportDir").toString();
    if(!dirValue.isEmpty())
    {
        dir = dirValue;
    }

    QString filename;
    qDebug() << "MissionEditor: loadMission with: " << _filename;

    if(_filename.isEmpty() || !QFileInfo(_filename).exists())
    {
        filename = QFileDialog::getOpenFileName(
        this, tr("Load mission description"),
        dir,
        tr("Mission Description File (*.mdf *.xml)"));
        if(filename.isEmpty())
        {
            return false;
        }
    } else {
        filename = _filename;
    }

    if(!filename.isEmpty())
    {
        // update recent files list
        QStringList files = settings.value("recentImportFileList").toStringList();
        files.removeAll(filename);
        files.prepend(filename);
        while(files.size() > 10)
        {
            files.removeLast();
        }
        settings.setValue("recentImportFileList", files);
        // end update recent files list

        mMissionFilename = filename;
        QFileInfo fileinfo(filename);
        settings.setValue("recentImportDir", fileinfo.absolutePath());

        try {
            Mission mission = templ::io::MissionReader::fromFile(filename.toStdString());
            mpMission = make_shared<Mission>(mission);
            mpMission->prepareTimeIntervals();

            updateVisualization();
            show();

            return true;
        } catch(const std::exception& e)
        {
            QMessageBox::warning(this, "Templ", QString("MissionView: failed to load file --") + QString(e.what()));
        }
    }
    return false;
}

bool MissionEditor::loadOrganizationModel(const QString& settingsLabel, const QString& _filename)
{
    QSettings settings(QCoreApplication::organizationName(), settingsLabel);
    QString dir = QDir::currentPath();

    QString dirValue = settings.value("recentImportOrganizationModelDir").toString();
    if(!dirValue.isEmpty())
    {
        dir = dirValue;
    }

    QString filename;
    qDebug() << "MissionEditor: loadOrganizationModel with: " << _filename;

    if(_filename.isEmpty() || !QFileInfo(_filename).exists())
    {
        filename = QFileDialog::getOpenFileName(
        this, tr("Load organization model"),
        dir,
        tr("Organization Model / Ontology (*.owl *.ttl)"));
        if(filename.isEmpty())
        {
            return false;
        }
    } else {
        filename = _filename;
    }

    if(!filename.isEmpty())
    {
        // update recent files list
        QStringList files = settings.value("recentImportOrganizationModelList").toStringList();
        files.removeAll(filename);
        files.prepend(filename);
        while(files.size() > 10)
        {
            files.removeLast();
        }
        settings.setValue("recentImportOrganizationModelList", files);
        // end update recent files list

        QFileInfo fileinfo(filename);
        settings.setValue("recentImportOrganizationModelDir", fileinfo.absolutePath());

        try {
            organization_model::OrganizationModel::Ptr om =
                make_shared<organization_model::OrganizationModel>(filename.toStdString());
            if(mpMission)
            {
                mpMission->setOrganizationModel(om);
            }

            updateVisualization();
            show();

            return true;
        } catch(const std::exception& e)
        {
            QMessageBox::warning(this, "Templ", QString("MissionView: failed to load file --") + QString(e.what()));
        }
    }
    return false;
}

void MissionEditor::updateVisualization()
{
    qDebug() << "Loading mission";
    QString name = QString::fromStdString( mpMission->getName());
    qDebug() << "    name: " << name;
    mpUi->lineEditName->setText(name);

    QString description = QString::fromStdString( mpMission->getDescription() );
    qDebug() << "    description: " << description;
    mpUi->textEditDescription->setText(description);

    std::string organizationModelIri = mpMission->getOrganizationModel()->ontology()->getIRI().toString();
    QString organizationModel = QString::fromStdString( organizationModelIri );
    qDebug() << "    organization Model: " << organizationModelIri.c_str();
    mpUi->lineEditOrganizationModel->setText(organizationModel);


    // Resources
    {
        organization_model::OrganizationModelAsk ask =
            mpMission->getOrganizationModelAsk();

        owlapi::model::IRIList agentModels = ask.getAgentModels();
        for(const owlapi::model::IRI iri : agentModels)
        {
            mpUi->comboBoxResourceModels->addItem(QString::fromStdString( iri.toString() ) );
        }

        organization_model::ModelPool availableResources = mpMission->getAvailableResources();
        for(const organization_model::ModelPool::value_type pair : availableResources)
        {
            const owlapi::model::IRI& iri = pair.first;
            const size_t cardinality = pair.second;

            qDebug() << "    resource: " << iri.toString().c_str() << ", max: " << cardinality;
        }

    }

    // Constants
    {
        using namespace templ::symbols;
        for(size_t c = static_cast<size_t>(Constant::UNKNOWN) + 1;
                c < static_cast<size_t>(Constant::END); ++c)
        {
            mpUi->comboBoxConstantsType->addItem(
                    QString::fromStdString(
                        Constant::TypeTxt[static_cast<Constant::Type>(c)]
                    )
            );
        }
        for(const Constant::Ptr& c : mpMission->getConstants())
        {
            qDebug() << "    constant: " << QString::fromStdString(c->toString());
        }
    }

    // Constraints
    {
        for(const Constraint::Ptr& c : mpMission->getConstraints())
        {
            qDebug() << "    constraint: " << QString::fromStdString(c->toString());
        }
    }
}

void MissionEditor::save(const QString& filename)
{
    qDebug() << "save mission to:" << filename;

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
        save(filename);
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

void MissionEditor::addResourceCardinality()
{
    QHBoxLayout* rowLayout = new QHBoxLayout;

    QString resourceModel = mpUi->comboBoxResourceModels->currentText();
    int value = mpUi->spinBoxMaxCardinality->value();

    qDebug() << "Add resource: " << resourceModel << " <= " << value;

    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);
    QLabel* labelModel = new QLabel( resourceModel );
    rowLayout->addWidget(labelModel);
    QLabel* labelValue = new QLabel( "<= " + QString::number(value));
    rowLayout->addWidget(labelValue);
    QSpacerItem* spacer = new QSpacerItem(50,0);
    rowLayout->addItem(spacer);

    mLayoutResources->addLayout(rowLayout);
    for(int i = 0; i < rowLayout->count(); ++i)
    {
        qDebug() << "#" << i << " in layout";
    }
}

void MissionEditor::removeResourceCardinalities()
{
    QList<QHBoxLayout*> removeLayouts;
    for(int i = 0; i < mLayoutResources->count();++i)
    {
        QHBoxLayout* rowLayout = qobject_cast<QHBoxLayout*>(mLayoutResources->itemAt(i)->layout());
        if(rowLayout)
        {
            QCheckBox* checkbox =
                qobject_cast<QCheckBox*>(rowLayout->itemAt(0)->widget());

            if(checkbox && checkbox->checkState() == Qt::Checked)
            {
                removeLayouts.append(rowLayout);
            }
        }
    }

    for(int i = 0; i < removeLayouts.size(); ++i)
    {
        removeResourceCardinality(removeLayouts[i]);
    }
}

void MissionEditor::removeResourceCardinality(QHBoxLayout* rowLayout)
{
    while(rowLayout->count() != 0)
    {
        QLayoutItem* item = rowLayout->itemAt(0);
        if(!item)
        {
            break;
        }

        QWidget* widget = item->widget();
        if(widget)
        {
            rowLayout->removeWidget(widget);
            delete widget;
        } else {
            rowLayout->removeItem(item);
            delete item;
        }
    }

    mLayoutResources->removeItem(rowLayout);
    delete rowLayout;
}

} // end namespace gui
} // end namespace templ
