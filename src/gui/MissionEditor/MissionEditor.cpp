#include "MissionEditor.hpp"

#include "../../io/MissionReader.hpp"
#include "../dialogs/AddLocation.hpp"
#include "../widgets/Location.hpp"
#include "../dialogs/AddModelConstraint.hpp"

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
{
    mpMission = make_shared<Mission>();

    mpUi->setupUi(this);

    connect(mpUi->pushButtonSelectOrganizationModel, SIGNAL(clicked()),
            this, SLOT(loadOrganizationModel()));
    connect(mpUi->pushButtonSave, SIGNAL(clicked()),
            this, SLOT(on_saveButton_clicked));
    connect(mpUi->pushButtonAddResource, SIGNAL(clicked()),
            this, SLOT(addResourceCardinality()));
    connect(mpUi->pushButtonRemoveResources, SIGNAL(clicked()),
            this, SLOT(removeResourceCardinalities()));

    connect(mpUi->pushButtonAddConstant, SIGNAL(clicked()),
            this, SLOT(addConstant()));
    connect(mpUi->pushButtonRemoveConstants, SIGNAL(clicked()),
            this, SLOT(removeConstants()));

    connect(mpUi->pushButtonAddConstraint, SIGNAL(clicked()),
            this, SLOT(addConstraint()));
    connect(mpUi->pushButtonRemoveConstraints, SIGNAL(clicked()),
            this, SLOT(removeConstraints()));
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
            mpUi->comboBoxConstantTypes->addItem(
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
        using namespace templ::constraints;
        for(size_t c = static_cast<size_t>(Constraint::UNKNOWN) +1;
                c < static_cast<size_t>(Constraint::END); ++c)
        {
            mpUi->comboBoxConstraintTypes->addItem(
                    QString::fromStdString(
                        Constraint::CategoryTxt[static_cast<Constraint::Category>(c)]

                    )
            );
        }

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

// Adding/Removing Constraints
void MissionEditor::addConstant()
{
    qDebug() << "Add constant";

    QString constantsType = mpUi->comboBoxConstantTypes->currentText();
    using namespace templ::symbols;
    if(constantsType.toStdString() == Constant::TypeTxt[Constant::LOCATION] )
    {
        dialogs::AddLocation dialog;
        dialog.exec();
        if(dialog.result() == QDialog::Accepted)
        {
            addLocation(dialog.getLocation());
        }
    }
}

void MissionEditor::addConstraint()
{
    qDebug() << "Add constraint";
    QString constraintType = mpUi->comboBoxConstraintTypes->currentText();

    using namespace templ::constraints;
    if(constraintType.toStdString() == Constraint::CategoryTxt[Constraint::MODEL])
    {
        dialogs::AddModelConstraint dialog(mpMission->getOrganizationModelAsk());
        dialog.exec();
        if(dialog.result() == QDialog::Accepted)
        {
            addModelConstraint(dialog.getConstraint());
        }
    }
    if(constraintType.toStdString() ==
            Constraint::CategoryTxt[Constraint::TEMPORAL_QUALITATIVE])
    {
    }
    if(constraintType.toStdString() ==
            Constraint::CategoryTxt[Constraint::TEMPORAL_QUANTITATIVE])
    {
    }
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

    //QStandardItemModel* comboBoxModel = qobject_cast<QStandardItemModel*>(mpUi->comboBoxResourceModel->model());
    //int rowIdx = comboBoxModel->currentIndex();
    //QStandardItem* item = comboBoxModel->item(rowIdx, 0);
    //item->setEnabled(false);

    mpUi->verticalLayoutResources->addLayout(rowLayout);
    for(int i = 0; i < rowLayout->count(); ++i)
    {
        qDebug() << "#" << i << " in layout";
    }
}

void MissionEditor::removeResourceCardinalities()
{
    removeCheckedRows(mpUi->verticalLayoutResources);
}

void MissionEditor::removeConstants()
{
    removeCheckedRows(mpUi->verticalLayoutConstants);
}

void MissionEditor::removeConstraints()
{
    removeCheckedRows(mpUi->verticalLayoutConstraintsTemporalQualitative);
    removeCheckedRows(mpUi->verticalLayoutConstraintsTemporalQuantitative);
    removeCheckedRows(mpUi->verticalLayoutConstraintsModel);
}

void MissionEditor::removeCheckedRows(QLayout* parentLayout)
{
    QList<QHBoxLayout*> removeLayouts;
    for(int i = 0; i < parentLayout->count();++i)
    {
        QHBoxLayout* rowLayout = qobject_cast<QHBoxLayout*>(parentLayout->itemAt(i)->layout());
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
        removeRow(parentLayout, removeLayouts[i]);
    }
}

void MissionEditor::removeRow(QLayout* parent, QHBoxLayout* rowLayout)
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

    parent->removeItem(rowLayout);
    delete rowLayout;
}

void MissionEditor::addLocation(const symbols::constants::Location::Ptr&
        location)
{
    QHBoxLayout* rowLayout = new QHBoxLayout;
    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::Location* locationWidget = new widgets::Location;
    locationWidget->setValue( location );
    rowLayout->addWidget(locationWidget);

    mpUi->verticalLayoutConstants->addLayout(rowLayout);
}

void MissionEditor::addModelConstraint(const constraints::ModelConstraint::Ptr&
        constraint)
{
    qDebug() << "Adding model constraint";

    QHBoxLayout* rowLayout = new QHBoxLayout;
    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::ModelConstraint* constraintWidget = new widgets::ModelConstraint;
    constraintWidget->prepare(mpMission->getOrganizationModelAsk());
    constraintWidget->setValue( constraint );
    rowLayout->addWidget(constraintWidget);

    mpUi->verticalLayoutConstraintsModel->addLayout(rowLayout);

}

} // end namespace gui
} // end namespace templ
