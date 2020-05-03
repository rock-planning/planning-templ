#include "MissionEditor.hpp"

#include "../../io/MissionReader.hpp"
#include "../dialogs/AddLocation.hpp"
#include "../widgets/Location.hpp"
#include "../dialogs/AddModelConstraint.hpp"
#include "../widgets/ModelConstraint.hpp"
#include "../dialogs/AddTemporalConstraintQualitative.hpp"
#include "../widgets/TemporalConstraintQualitative.hpp"
#include "../dialogs/AddTemporalConstraintQuantitative.hpp"
#include "../widgets/TemporalConstraintQuantitative.hpp"
#include "../widgets/SpatioTemporalRequirement.hpp"
#include "../Utils.hpp"
#include "../../symbols/object_variables/LocationCardinality.hpp"
#include "../../SpaceTime.hpp"

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
#include <tuple>
#include <algorithm>

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

    connect(mpUi->pushButtonAddRequirement, SIGNAL(clicked()),
                this, SLOT(addRequirement()));
    connect(mpUi->pushButtonRemoveRequirements, SIGNAL(clicked()),
                this, SLOT(removeRequirements()));
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
    QString name = QString::fromStdString( mpMission->getName());
    mpUi->lineEditName->setText(name);

    QString description = QString::fromStdString( mpMission->getDescription() );
    qDebug() << "    description: " << description;
    mpUi->textEditDescription->setText(description);

    std::string organizationModelIri = mpMission->getOrganizationModel()->ontology()->getIRI().toString();
    QString organizationModel = QString::fromStdString( organizationModelIri );
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
            io::ResourceRequirement r;
            r.model = pair.first;
            r.minCardinality = 0;
            r.maxCardinality = pair.second;

            widgets::ModelCardinality* cardinality = addResourceCardinality();
            cardinality->setRequirement(r);
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
            switch(c->getConstantType())
            {
                case Constant::LOCATION:
                    addLocation(dynamic_pointer_cast<symbols::constants::Location>(c));
                    break;
                default:
                    break;
            }
        }
    }

    // Requirements
    {
        std::map< std::tuple< std::string, std::string, std::string>, widgets::SpatioTemporalRequirement*> requirementWidgets;
        std::vector<solvers::temporal::PersistenceCondition::Ptr> pcs =
            mpMission->getPersistenceConditions();

        for(const solvers::temporal::PersistenceCondition::Ptr& pc : pcs)
        {
            // pc:
            //  statevariable --> rloc : location cardinality + resourceModel(as string)
            //  objectvariable: location, cardinality, restriction type
            //  from
            //  to
            owlapi::model::IRI resourceModel(pc->getStateVariable().getResource());
            symbols::object_variables::LocationCardinality::Ptr lc =
                dynamic_pointer_cast<symbols::object_variables::LocationCardinality>(pc->getValue());
            symbols::constants::Location::Ptr location = lc->getLocation();
            solvers::temporal::point_algebra::TimePoint::Ptr from = pc->getFromTimePoint();
            solvers::temporal::point_algebra::TimePoint::Ptr to = pc->getToTimePoint();

            widgets::SpatioTemporalRequirement* str;

            auto widgetKey = std::make_tuple(location->getInstanceName(),
                    from->getLabel(), to->getLabel());
            auto widgetsIt = requirementWidgets.find(widgetKey);
            if(widgetsIt == requirementWidgets.end())
            {
                qDebug() << "Add new requirement";
                str = addRequirement();
                requirementWidgets[widgetKey] = str;
            } else {
                qDebug() << "Found existing";
                str = widgetsIt->second;
            }

            str->updateRequirement(*pc.get());
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
            if(mpMission->isImplicitConstraint(c))
            {
                continue;
            }

            qDebug() << "    constraint: " << QString::fromStdString(c->toString());
            switch(c->getCategory())
            {
                case Constraint::MODEL:
                    addModelConstraint(dynamic_pointer_cast<constraints::ModelConstraint>(c));
                    break;
                case Constraint::TEMPORAL_QUALITATIVE:
                {
                    namespace pa = solvers::temporal::point_algebra;
                    pa::QualitativeTimePointConstraint::Ptr qtpc = dynamic_pointer_cast<pa::QualitativeTimePointConstraint>(c);

                    io::TemporalConstraint tc;
                    tc.rval = qtpc->getRVal()->getLabel();
                    tc.lval = qtpc->getLVal()->getLabel();
                    tc.type = qtpc->getType();
                    addTemporalConstraintQualitative(tc);
                    break;
                }
                case Constraint::TEMPORAL_QUANTITATIVE:
                {
                    namespace tp = solvers::temporal;
                    tp::IntervalConstraint::Ptr ic = dynamic_pointer_cast<tp::IntervalConstraint>(c);

                    io::TemporalConstraint tc;
                    tc.rval = ic->getSourceTimePoint()->getLabel();
                    tc.lval = ic->getTargetTimePoint()->getLabel();
                    tc.minDuration = ic->getLowerBound();
                    tc.maxDuration = ic->getUpperBound();
                    addTemporalConstraintQualitative(tc);
                    break;
                }
                default:
                    break;
            }
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

    requirementsUpdated();
}

widgets::SpatioTemporalRequirement* MissionEditor::addRequirement()
{
    QHBoxLayout* rowLayout = new QHBoxLayout;

    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::SpatioTemporalRequirement* str = new
        widgets::SpatioTemporalRequirement( mpMission->getOrganizationModelAsk() );

    connect(str, SIGNAL(keyChanged(QString)),
            this, SLOT(requirementsUpdated()));

    symbols::constants::Location::PtrList locations = getLocations();
    QList<QString> locationNames;
    for(const symbols::constants::Location::Ptr& l : locations)
    {
        locationNames.append( QString::fromStdString(l->getInstanceName()) );
    }
    str->prepareLocations(locationNames);
    //str->prepareTimepoints(mTimepoints);
    rowLayout->addWidget(str);
    mpUi->verticalLayoutRequirements->addLayout(rowLayout);

    return str;
}

void MissionEditor::removeRequirements()
{
    Utils::removeCheckedRows( mpUi->verticalLayoutRequirements );
    requirementsUpdated();
}

QList<QString> MissionEditor::getRequirementsKeys() const
{
    QList<QString> keys;

    QList<widgets::Location*> locations =
        Utils::getWidgets<widgets::Location>(mpUi->verticalLayoutConstants);
    for(const widgets::Location* l : locations)
    {
        keys.append( QString::fromStdString(l->getValue()->getInstanceName()) );
    }


    QList<widgets::SpatioTemporalRequirement*> strs = Utils::getWidgets<widgets::SpatioTemporalRequirement>(mpUi->verticalLayoutRequirements);
    for(const widgets::SpatioTemporalRequirement* str : strs)
    {
        keys.append(str->getKey());
    }

    std::sort(keys.begin(), keys.end());

    return keys;
}

void MissionEditor::requirementsUpdated()
{
    QList<QString> keys = getRequirementsKeys();
    qDebug() << "Requirements updated" << keys;
    QList<widgets::ModelConstraint*> widgets =
        Utils::getWidgets<widgets::ModelConstraint>(mpUi->verticalLayoutConstraintsModel);
    for(widgets::ModelConstraint* c : widgets)
    {
        c->updateRequirements(keys);
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
        dialogs::AddTemporalConstraintQualitative dialog;
        dialog.exec();
        if(dialog.result() == QDialog::Accepted)
        {
            addTemporalConstraintQualitative(dialog.getConstraint());
        }
    }
    if(constraintType.toStdString() ==
            Constraint::CategoryTxt[Constraint::TEMPORAL_QUANTITATIVE])
    {
        dialogs::AddTemporalConstraintQuantitative dialog;
        dialog.exec();
        if(dialog.result() == QDialog::Accepted)
        {
            addTemporalConstraintQuantitative(dialog.getConstraint());
        }
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

widgets::ModelCardinality* MissionEditor::addResourceCardinality()
{
    QHBoxLayout* rowLayout = new QHBoxLayout;

    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::ModelCardinality* cardinality = new widgets::ModelCardinality;
    cardinality->prepare(mpMission->getOrganizationModelAsk());
    cardinality->setMinVisible(false);
    rowLayout->addWidget(cardinality);

    mpUi->verticalLayoutResources->addLayout(rowLayout);
    return cardinality;
}

void MissionEditor::removeResourceCardinalities()
{
    Utils::removeCheckedRows(mpUi->verticalLayoutResources);
}

void MissionEditor::removeConstants()
{
    Utils::removeCheckedRows(mpUi->verticalLayoutConstants);
}

void MissionEditor::removeConstraints()
{
    Utils::removeCheckedRows(mpUi->verticalLayoutConstraintsTemporalQualitative);
    Utils::removeCheckedRows(mpUi->verticalLayoutConstraintsTemporalQuantitative);
    Utils::removeCheckedRows(mpUi->verticalLayoutConstraintsModel);
}

widgets::Location* MissionEditor::addLocation(const symbols::constants::Location::Ptr&
        location)
{
    QHBoxLayout* rowLayout = new QHBoxLayout;
    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::Location* locationWidget = new widgets::Location;
    qDebug() << "Found location: " << location->getInstanceName().c_str() << " " <<
        location->getPosition().x() << "/" << location->getPosition().y();

    locationWidget->setValue( location );
    rowLayout->addWidget(locationWidget);

    mpUi->verticalLayoutConstants->addLayout(rowLayout);
    return locationWidget;
}

symbols::constants::Location::PtrList MissionEditor::getLocations() const
{
    symbols::constants::Location::PtrList locations;
    QList<widgets::Location*> widgets = Utils::getWidgets<widgets::Location>(mpUi->verticalLayoutConstants, 1);
    for(const widgets::Location* widget : widgets)
    {
        locations.push_back( widget->getValue() );
    }
    return locations;
}

widgets::ModelConstraint* MissionEditor::addModelConstraint(const constraints::ModelConstraint::Ptr&
        constraint)
{
    qDebug() << "Adding model constraint";

    QHBoxLayout* rowLayout = new QHBoxLayout;
    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::ModelConstraint* constraintWidget = new widgets::ModelConstraint;
    constraintWidget->prepare(mpMission->getOrganizationModelAsk());
    QList<QString> keys = getRequirementsKeys();
    qDebug() << "Requirement keys " << keys;
    constraintWidget->updateRequirements(keys);
    constraintWidget->setValue( constraint );
    rowLayout->addWidget(constraintWidget);

    mpUi->verticalLayoutConstraintsModel->addLayout(rowLayout);
    return constraintWidget;
}

widgets::TemporalConstraintQualitative* MissionEditor::addTemporalConstraintQualitative(const io::TemporalConstraint& constraint)
{
    qDebug() << "Adding temporal constraint (qualitative)";

    QHBoxLayout* rowLayout = new QHBoxLayout;
    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::TemporalConstraintQualitative* constraintWidget = new widgets::TemporalConstraintQualitative;
    //constraintWidget->prepare();
    constraintWidget->setConstraint( constraint );
    rowLayout->addWidget(constraintWidget);

    mpUi->verticalLayoutConstraintsTemporalQualitative->addLayout(rowLayout);
    return constraintWidget;
}

widgets::TemporalConstraintQuantitative* MissionEditor::addTemporalConstraintQuantitative(const io::TemporalConstraint& constraint)
{
    qDebug() << "Adding temporal constraint (quantitative)";

    QHBoxLayout* rowLayout = new QHBoxLayout;
    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::TemporalConstraintQuantitative* constraintWidget = new widgets::TemporalConstraintQuantitative;
    //constraintWidget->prepare();
    constraintWidget->setConstraint( constraint );
    rowLayout->addWidget(constraintWidget);

    mpUi->verticalLayoutConstraintsTemporalQuantitative->addLayout(rowLayout);
    return constraintWidget;
}

solvers::temporal::point_algebra::TimePoint::PtrList MissionEditor::getTimepoints()
    const
{
    solvers::temporal::point_algebra::TimePoint::PtrList timepoints;
    return timepoints;
}

} // end namespace gui
} // end namespace templ
