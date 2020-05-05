#include "MissionEditor.hpp"

#include "../../io/MissionReader.hpp"
#include "../../io/MissionWriter.hpp"
#include "../dialogs/AddLocation.hpp"
#include "../widgets/Location.hpp"
#include "../dialogs/AddModelConstraint.hpp"
#include "../widgets/ModelConstraint.hpp"
#include "../dialogs/AddTemporalConstraintQualitative.hpp"
#include "../widgets/TemporalConstraintQualitative.hpp"
#include "../dialogs/AddTemporalConstraintQuantitative.hpp"
#include "../widgets/TemporalConstraintQuantitative.hpp"
#include "../widgets/SpatioTemporalRequirement.hpp"
#include "../widgets/PropertyOverride.hpp"
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
    connect(mpUi->pushButtonLoad, SIGNAL(clicked()),
            this, SLOT(on_loadMissionButton_clicked()));
    connect(mpUi->pushButtonClear, SIGNAL(clicked()),
            this, SLOT(on_clearButton_clicked()));
    connect(mpUi->pushButtonSave, SIGNAL(clicked()),
            this, SLOT(on_saveButton_clicked()));
    connect(mpUi->pushButtonAddResource, SIGNAL(clicked()),
            this, SLOT(addResourceCardinality()));
    connect(mpUi->pushButtonRemoveResources, SIGNAL(clicked()),
            this, SLOT(removeResourceCardinalities()));

    connect(mpUi->pushButtonAddOverride, SIGNAL(clicked()),
            this, SLOT(addOverride()));
    connect(mpUi->pushButtonRemoveOverrides, SIGNAL(clicked()),
            this, SLOT(removeOverrides()));

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

    deactivateOrganizationModelDependants();
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

            clear();
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
    activateOrganizationModelDependants();

    // Resources
    {
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
                str = addRequirement();
                requirementWidgets[widgetKey] = str;
            } else {
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

Mission::Ptr MissionEditor::currentMission() const
{
    Mission::Ptr m = make_shared<Mission>();

    m->setName(mpUi->lineEditName->text().toStdString());
    m->setDescription(mpUi->textEditDescription->toPlainText().toStdString());
    QString organizationModel = mpUi->lineEditOrganizationModel->text();
    m->setOrganizationModel(organizationModel.toStdString());

    qDebug() << "Set organization model: " << organizationModel;

    QList<widgets::Location*> locationWidgets = Utils::getWidgets<widgets::Location>(mpUi->verticalLayoutConstants);
    for(widgets::Location* w : locationWidgets)
    {
        symbols::constants::Location::Ptr location = w->getValue();
        qDebug() << "Add constant: " << QString::fromStdString( location->getInstanceName() );
        m->addConstant(location);
    }

    QList<widgets::ModelCardinality*> resourceWidgets = Utils::getWidgets<widgets::ModelCardinality>(mpUi->verticalLayoutResources);
    organization_model::ModelPool availableResources;
    for(widgets::ModelCardinality* w : resourceWidgets)
    {
        io::ResourceRequirement r = w->getRequirement();
        availableResources[r.model] = r.maxCardinality;
    }
    qDebug() << "Set available resources: " << QString::fromStdString( availableResources.toString() );
    m->setAvailableResources(availableResources);

    // Overrides
    {
        QList<widgets::PropertyOverride*> overrideWidgets =
            Utils::getWidgets<widgets::PropertyOverride>(mpUi->verticalLayoutOverrides);
        for(widgets::PropertyOverride* w : overrideWidgets)
        {
            DataPropertyAssignment dpa = w->getDataPropertyAssignment();
            m->addDataPropertyAssignment(dpa);
        }
    }

    // Requirements
    {
        namespace pa = solvers::temporal::point_algebra;
        QList<widgets::SpatioTemporalRequirement*> strWidgets =
            Utils::getWidgets<widgets::SpatioTemporalRequirement>(mpUi->verticalLayoutRequirements);
        for(widgets::SpatioTemporalRequirement* w : strWidgets)
        {
            io::SpatioTemporalRequirement::Ptr str = w->getRequirement();

            // Retrieve constant from mission object
            std::string locationId = str->spatial.location.id;
            symbols::Constant::Ptr constant = m->getConstant(locationId, symbols::Constant::LOCATION);
            symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(constant);

            pa::TimePoint::Ptr from = pa::TimePoint::create(str->temporal.from);
            pa::TimePoint::Ptr to = pa::TimePoint::create(str->temporal.to);

            for(const io::ResourceRequirement& resource : str->resources)
            {
                // setting the min cardinality by default
                m->addResourceLocationCardinalityConstraint(location, from, to,
                    resource.model,
                    resource.minCardinality,
                    owlapi::model::OWLCardinalityRestriction::MIN);

                //// setting the max cardinality by default
                //m->addResourceLocationCardinalityConstraint(location, from, to,
                //    resource.model,
                //    resource.maxCardinality,
                //    owlapi::model::OWLCardinalityRestriction::MAX);
            }
        }
    }

    // Constraints
    {
        QList<widgets::ModelConstraint*> modelConstraintWidgets = Utils::getWidgets<widgets::ModelConstraint>(mpUi->verticalLayoutConstraintsModel);
        for(widgets::ModelConstraint* w : modelConstraintWidgets)
        {
            constraints::ModelConstraint::Ptr c = w->getConstraint();
            if(!c)
            {
                throw
                    std::runtime_error("templ::gui::MissionEditor::currentMission:"
                            " failed to get model constraint");
            } else {
                m->addConstraint(c);
            }
        }
        namespace pa = solvers::temporal::point_algebra;
        QList<widgets::TemporalConstraintQualitative*> qualitativeConstraintWidgets = Utils::getWidgets<widgets::TemporalConstraintQualitative>(mpUi->verticalLayoutConstraintsTemporalQualitative);
        for(widgets::TemporalConstraintQualitative* w : qualitativeConstraintWidgets)
        {
            pa::QualitativeTimePointConstraint::Ptr c =
                w->getQualitativeTemporalConstraint();
            if(!c)
            {
                throw
                    std::runtime_error("templ::gui::MissionEditor::currentMission:"
                            " failed to get temporal (qualitative) constraint");
            } else {
                m->addConstraint(c);
            }
        }

        QList<widgets::TemporalConstraintQuantitative*> quantitativeConstraintWidgets = Utils::getWidgets<widgets::TemporalConstraintQuantitative>(mpUi->verticalLayoutConstraintsTemporalQuantitative);
        for(widgets::TemporalConstraintQuantitative* w : quantitativeConstraintWidgets)
        {
            solvers::temporal::IntervalConstraint::Ptr c = w->getIntervalConstraint();
            if(!c)
            {
                throw
                    std::runtime_error("templ::gui::MissionEditor::currentMission:"
                            " failed to get temporal (quantitative) constraint");
            } else {
                m->addConstraint(c);
            }
        }
    }
    return m;
}

void MissionEditor::save(const QString& filename)
{
    qDebug() << "Save mission to:" << filename;
    Mission::Ptr m = currentMission();

    io::MissionWriter::write(filename.toStdString(), *m.get());
}

void MissionEditor::clear()
{
   mpUi->lineEditName->clear();
   mpUi->textEditDescription->clear();
   mpUi->lineEditOrganizationModel->clear();

   Utils::removeRows( mpUi->verticalLayoutResources );
   Utils::removeRows( mpUi->verticalLayoutConstants );
   Utils::removeRows( mpUi->verticalLayoutRequirements );

   Utils::removeRows( mpUi->verticalLayoutConstraintsModel );
   Utils::removeRows( mpUi->verticalLayoutConstraintsTemporalQuantitative );
   Utils::removeRows( mpUi->verticalLayoutConstraintsTemporalQualitative );

   deactivateOrganizationModelDependants();
}

void MissionEditor::on_loadMissionButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load mission description"),
        QDir::currentPath(),
        tr("Mission Description File (*.mdf *.xml)"));

    if(!filename.isEmpty())
    {
        loadMission("IOMission", filename);
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
    clear();
}

void MissionEditor::addOverride()
{
    qDebug() << "Add override";

    QHBoxLayout* rowLayout = new QHBoxLayout;

    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::PropertyOverride* po = new
        widgets::PropertyOverride(mpMission->getOrganizationModelAsk());
    rowLayout->addWidget(po);

    mpUi->verticalLayoutOverrides->addLayout(rowLayout);
}

void MissionEditor::removeOverrides()
{
    Utils::removeCheckedRows(mpUi->verticalLayoutOverrides);
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

void MissionEditor::activateOrganizationModelDependants()
{
    mpUi->groupBoxResources->setEnabled(true);
    mpUi->groupBoxRequirements->setEnabled(true);
    mpUi->groupBoxConstraints->setEnabled(true);
    mpUi->groupBoxOverrides->setEnabled(true);
}

void MissionEditor::deactivateOrganizationModelDependants()
{
    mpUi->groupBoxResources->setEnabled(false);
    mpUi->groupBoxRequirements->setEnabled(false);
    mpUi->groupBoxConstraints->setEnabled(false);
    mpUi->groupBoxOverrides->setEnabled(false);
}

} // end namespace gui
} // end namespace templ
