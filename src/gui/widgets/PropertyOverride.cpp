#include "PropertyOverride.hpp"
#include "ui_PropertyOverride.h"

#include "../MouseWheelGuard.hpp"

namespace templ {
namespace gui {
namespace widgets {

PropertyOverride::PropertyOverride(
        const moreorg::OrganizationModelAsk& ask,
        QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::PropertyOverride)
    , mAsk(ask)
{
    mpUi->setupUi(this);
    connect(mpUi->comboBoxSubject, SIGNAL(currentIndexChanged(QString)),
            this, SLOT(updateProperties()));

    mpUi->doubleSpinBoxValue->installEventFilter(new
            MouseWheelGuard(mpUi->doubleSpinBoxValue));

    prepare();
}

PropertyOverride::~PropertyOverride()
{
    delete mpUi;
}

void PropertyOverride::setOrganizationModelAsk(const moreorg::OrganizationModelAsk& ask)
{
    mAsk = ask;
    prepare();
}

void PropertyOverride::clear()
{
    mpUi->comboBoxSubject->clear();
    mpUi->comboBoxProperty->clear();
    mpUi->doubleSpinBoxValue->clear();
}

void PropertyOverride::prepare()
{
    clear();

    owlapi::model::IRIList agentModels = mAsk.getAgentModels();
    for(const owlapi::model::IRI& iri : agentModels)
    {
        mpUi->comboBoxSubject->addItem(QString::fromStdString( iri.toString() ) );
    }
    owlapi::model::IRIList serviceModels = mAsk.getServiceModels();
    for(const owlapi::model::IRI& iri : serviceModels)
    {
        mpUi->comboBoxSubject->addItem(QString::fromStdString( iri.toString() ) );
    }
}

void PropertyOverride::updateProperties()
{
    mpUi->comboBoxProperty->clear();
    owlapi::model::IRI
        subject(mpUi->comboBoxSubject->currentText().toStdString());
    if(!subject.empty())
    {
        owlapi::model::IRIList propertyModels = mAsk.getAgentProperties(subject);
        for(const owlapi::model::IRI& iri : propertyModels)
        {
            mpUi->comboBoxProperty->addItem(QString::fromStdString(iri.toString()));
        }
    }
}

DataPropertyAssignment PropertyOverride::getDataPropertyAssignment() const
{
    owlapi::model::IRI subject(mpUi->comboBoxSubject->currentText().toStdString());
    owlapi::model::IRI property(mpUi->comboBoxProperty->currentText().toStdString());
    double value = mpUi->doubleSpinBoxValue->value();

    return DataPropertyAssignment(subject, property, value);
}

} // end namespace widgets
} // end namespace gui
} // end namespace templ
