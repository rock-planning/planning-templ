#include "SpatioTemporalRequirement.hpp"
#include "ui_SpatioTemporalRequirement.h"
#include "ModelCardinality.hpp"
#include "../Utils.hpp"
#include <QCheckBox>
#include <QDebug>

namespace templ {
namespace gui {
namespace widgets {

SpatioTemporalRequirement::SpatioTemporalRequirement(
        const organization_model::OrganizationModelAsk& ask,
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

    {
        QString location = QString::fromStdString(str->spatial.location.id);
        int index = mpUi->comboBoxLocation->findText(location);
        if(index == -1)
        {
            mpUi->comboBoxLocation->addItem(location);
            index = mpUi->comboBoxLocation->findText(location);
        }
        mpUi->comboBoxLocation->setCurrentIndex(index);
    }

    {
        QString timepoint = QString::fromStdString( str->temporal.from );
        int index = mpUi->comboBoxLocation->findText(timepoint);
        if(index == -1)
        {
            mpUi->comboBoxFrom->addItem(timepoint);
            index = mpUi->comboBoxFrom->findText(timepoint);
        }
        mpUi->comboBoxFrom->setCurrentIndex(index);
    }

    {
        QString timepoint = QString::fromStdString( str->temporal.to );
        int index = mpUi->comboBoxLocation->findText(timepoint);
        if(index == -1)
        {
            mpUi->comboBoxTo->addItem(timepoint);
            index = mpUi->comboBoxTo->findText(timepoint);
        }
        mpUi->comboBoxTo->setCurrentIndex(index);
    }
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

} // namespace widgets
} // namespace gui
} // namespace templ
