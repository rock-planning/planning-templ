#include "SpatioTemporalRequirement.hpp"
#include "ui_SpatioTemporalRequirement.h"
#include "ModelCardinality.hpp"
#include "../Utils.hpp"
#include "../../symbols/object_variables/LocationCardinality.hpp"
#include <QCheckBox>
#include <QDebug>
#include <stdexcept>

namespace templ {
namespace gui {
namespace widgets {

SpatioTemporalRequirement::SpatioTemporalRequirement(
        const moreorg::OrganizationModelAsk& ask,
        QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::SpatioTemporalRequirement)
    , mAsk(ask)
{
    mpUi->setupUi(this);

    connect(mpUi->pushButtonAdd, SIGNAL(clicked()),
            this, SLOT(addModelCardinality()));
    connect(mpUi->pushButtonRemove, SIGNAL(clicked()),
            this, SLOT(removeModelCardinalities()));

    connect(mpUi->comboBoxLocation, SIGNAL(currentIndexChanged(QString)),
            this, SLOT(updateKey()));
    connect(mpUi->comboBoxFrom, SIGNAL(currentIndexChanged(QString)),
            this, SLOT(updateKey()));
    connect(mpUi->comboBoxTo, SIGNAL(currentIndexChanged(QString)),
            this, SLOT(updateKey()));
}

SpatioTemporalRequirement::~SpatioTemporalRequirement()
{
    Utils::removeRows(mpUi->verticalLayoutModelCardinalities);
    delete mpUi;
}

void SpatioTemporalRequirement::prepareLocations(const QList<QString>& locations)
{
    mpUi->comboBoxLocation->clear();
    for(const QString& l : locations)
    {
        mpUi->comboBoxLocation->addItem(l);
    }
    mpUi->comboBoxLocation->setCurrentIndex(-1);
}

void SpatioTemporalRequirement::prepareTimepoints(const QList<QString>& timepoints)
{
    mpUi->comboBoxFrom->clear();
    mpUi->comboBoxTo->clear();

    for(const QString& t : timepoints)
    {
        mpUi->comboBoxFrom->addItem(t);
        mpUi->comboBoxTo->addItem(t);
    }
    mpUi->comboBoxFrom->setCurrentIndex(-1);
    mpUi->comboBoxTo->setCurrentIndex(-1);
}

void SpatioTemporalRequirement::setLocation(const std::string& locationId)
{
    QString location = QString::fromStdString(locationId);
    int index = mpUi->comboBoxLocation->findText(location);
    if(index == -1)
    {
        mpUi->comboBoxLocation->addItem(location);
        index = mpUi->comboBoxLocation->findText(location);
    }
    mpUi->comboBoxLocation->setCurrentIndex(index);
}

void SpatioTemporalRequirement::setTimepointFrom(const std::string& from)
{
    QString timepoint = QString::fromStdString(from);
    int index = mpUi->comboBoxFrom->findText(timepoint);
    if(index == -1)
    {
        mpUi->comboBoxFrom->addItem(timepoint);
        index = mpUi->comboBoxFrom->findText(timepoint);
    }
    mpUi->comboBoxFrom->setCurrentIndex(index);
}

void SpatioTemporalRequirement::setTimepointTo(const std::string& to)
{
    QString timepoint = QString::fromStdString(to);
    int index = mpUi->comboBoxTo->findText(timepoint);
    if(index == -1)
    {
        mpUi->comboBoxTo->addItem(timepoint);
        index = mpUi->comboBoxTo->findText(timepoint);
    }
    mpUi->comboBoxTo->setCurrentIndex(index);
}

void SpatioTemporalRequirement::setRequirement(const
        io::SpatioTemporalRequirement::Ptr& str)
{
    if(!str)
    {
        return;
    }

    for(const io::ResourceRequirement& r : str->resources)
    {
        widgets::ModelCardinality* cardinality = addModelCardinality();
        cardinality->setRequirement(r);
    }

    setLocation(str->spatial.location.id);
    setTimepointFrom(str->temporal.from);
    setTimepointTo(str->temporal.to);
}

void SpatioTemporalRequirement::updateRequirement(const solvers::temporal::PersistenceCondition& pc)
{
    owlapi::model::IRI resourceModel(pc.getStateVariable().getResource());
    symbols::object_variables::LocationCardinality::Ptr lc = dynamic_pointer_cast<symbols::object_variables::LocationCardinality>(pc.getValue());
    symbols::constants::Location::Ptr location = lc->getLocation();

    if(mpUi->comboBoxLocation->currentIndex() == -1)
    {
        setLocation( location->getInstanceName() );
        setTimepointFrom( pc.getFromTimePoint()->getLabel() );
        setTimepointTo( pc.getToTimePoint()->getLabel() );
    } else if(mpUi->comboBoxLocation->currentText() != QString::fromStdString(location->getInstanceName()) )
    {
        // invalid to update with this persistence condition
        throw std::runtime_error("templ::gui::widgets::SpatialRequirement::updateRequirement: "
                "update tried for location '" + location->getInstanceName() + "', but widgets location "
                " is '" + mpUi->comboBoxLocation->currentText().toStdString());
    }

    QList<widgets::ModelCardinality*> rows = Utils::getWidgets<ModelCardinality>(mpUi->verticalLayoutModelCardinalities);
    for(widgets::ModelCardinality* row : rows)
    {
        io::ResourceRequirement r = row->getRequirement();
        if(r.model == resourceModel)
        {
            size_t cardinality = lc->getCardinality();
            switch(lc->getCardinalityRestrictionType())
            {
                case owlapi::model::OWLCardinalityRestriction::MIN:
                    r.minCardinality = cardinality;
                    break;
                case owlapi::model::OWLCardinalityRestriction::MAX:
                    r.maxCardinality = cardinality;
                    break;
                default:
                    break;
            }
            row->setRequirement(r);
            return;
        }
    }

    // Now existing definition, so add new
    widgets::ModelCardinality* widget = addModelCardinality();

    io::ResourceRequirement r;
    r.model = resourceModel;
    size_t cardinality = lc->getCardinality();
    switch(lc->getCardinalityRestrictionType())
    {
        case owlapi::model::OWLCardinalityRestriction::MIN:
            r.minCardinality = cardinality;
            break;
        case owlapi::model::OWLCardinalityRestriction::MAX:
            r.maxCardinality = cardinality;
            break;
        default:
            break;
    }
    widget->setRequirement(r);
}

io::SpatioTemporalRequirement::Ptr SpatioTemporalRequirement::getRequirement() const
{
    io::SpatioTemporalRequirement::Ptr str =
        make_shared<io::SpatioTemporalRequirement>();

    QList<widgets::ModelCardinality*> widgets = Utils::getWidgets<widgets::ModelCardinality>(mpUi->verticalLayoutModelCardinalities, 1);
    for(const widgets::ModelCardinality* widget : widgets)
    {
        qDebug() << "Found model cardinality";
        io::ResourceRequirement r = widget->getRequirement();
        str->resources.push_back(r);
    }

    str->spatial.location.id = mpUi->comboBoxLocation->currentText().toStdString();
    str->temporal.from = mpUi->comboBoxFrom->currentText().toStdString();
    str->temporal.to = mpUi->comboBoxTo->currentText().toStdString();
    return str;
}

widgets::ModelCardinality* SpatioTemporalRequirement::addModelCardinality()
{
    QHBoxLayout* rowLayout = new QHBoxLayout;
    rowLayout->setSizeConstraint(QLayout::SetMaximumSize);
    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    widgets::ModelCardinality* cardinality = new widgets::ModelCardinality;
    cardinality->prepare(mAsk);
    rowLayout->addWidget(cardinality);

    mpUi->verticalLayoutModelCardinalities->addLayout(rowLayout);
    return cardinality;
}

void SpatioTemporalRequirement::removeModelCardinalities()
{
    Utils::removeCheckedRows(mpUi->verticalLayoutModelCardinalities);
}

void SpatioTemporalRequirement::updateKey()
{
    QString key = getKey();
    emit keyChanged(key);
}

QString SpatioTemporalRequirement::getKey() const
{
    QString location = mpUi->comboBoxLocation->currentText();
    QString from = mpUi->comboBoxFrom->currentText();
    QString to = mpUi->comboBoxTo->currentText();

    return location + ",[" + from + "," + to + "]";
}

} // namespace widgets
} // namespace gui
} // namespace templ
