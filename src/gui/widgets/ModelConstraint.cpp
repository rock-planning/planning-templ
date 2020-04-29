#include "ModelConstraint.hpp"
#include "ui_ModelConstraint.h"

#include <owlapi/model/IRI.hpp>

#include <QString>
#include <QDebug>

namespace templ {
namespace gui {
namespace widgets {

ModelConstraint::ModelConstraint(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::ModelConstraint)
{
    mpUi->setupUi(this);
}

ModelConstraint::~ModelConstraint()
{
    delete mpUi;
}

void ModelConstraint::prepare(const organization_model::OrganizationModelAsk& ask)
{
    for(size_t c = static_cast<size_t>(constraints::ModelConstraint::UNKNOWN) + 1;
            c < static_cast<size_t>(constraints::ModelConstraint::END_TYPE); ++c)
    {
        mpUi->comboBoxTypes->addItem(
                QString::fromStdString(
                    constraints::ModelConstraint::TypeTxt[ static_cast<constraints::ModelConstraint::Type>(c) ]
                )
        );
    }
    connect(mpUi->comboBoxTypes, SLOT(currentIndexChanged(QString)),
                this, SIGNAL(typeChanged(QString)));

    owlapi::model::IRIList agentModels = ask.getAgentModels();
    for(const owlapi::model::IRI iri : agentModels)
    {
        mpUi->comboBoxModels->addItem(QString::fromStdString( iri.toString() ) );
    }

    connect(mpUi->comboBoxModels, SLOT(currentIndexChanged(QString)),
                this, SIGNAL(modelChanged(QString)));

    owlapi::model::IRIList agentProperties = ask.getAgentProperties();
    for(const owlapi::model::IRI iri : agentProperties)
    {
        mpUi->comboBoxProperties->addItem(QString::fromStdString( iri.toString() ) );
    }

}

void ModelConstraint::setValue(const constraints::ModelConstraint::Ptr& constraint)
{
    qDebug() << "Setting model value";
    if(!constraint)
    {
        return;
    }

    constraints::ModelConstraint::Type type = constraint->getModelConstraintType();
    {
        QString typeTxt = QString::fromStdString( constraints::ModelConstraint::TypeTxt[type] );

        int index = mpUi->comboBoxTypes->findText(typeTxt);
        if(index != -1)
        {
            mpUi->comboBoxTypes->setCurrentIndex(index);
        } else {
            throw std::runtime_error("templ::gui::widgets::ModelConstraint::setValue:"
                    " no type " + typeTxt.toStdString() + " known - (internal error: did you call prepare?)");
        }
    }

    {
        owlapi::model::IRI iri = constraint->getModel();
        int index = mpUi->comboBoxModels->findText(QString::fromStdString( iri.toString()) );
        if(index != -1)
        {
            mpUi->comboBoxModels->setCurrentIndex(index);
        } else {
            throw std::runtime_error("templ::gui::widgets::ModelConstraint::setValue:"
                    " no model " + iri.toString() + " known");
        }
    }

    {
        double value = constraint->getValue();
        mpUi->spinBoxValue->setValue( value );
    }

    {
        owlapi::model::IRI property = constraint->getProperty();
        if(!property.empty())
        {
            int index = mpUi->comboBoxProperties->findText( QString::fromStdString( property.toString() ));
            if(index != -1)
            {
                mpUi->comboBoxProperties->setCurrentIndex(index);
            } else {
                throw std::runtime_error("templ::gui::widgets::ModelConstraint::setValue:"
                        " no property " + property.toString() + " known - pls verify the ontology");
            }
        }
    }
}

constraints::ModelConstraint::Ptr ModelConstraint::getConstraint() const
{
    qDebug() << "Getting Model Constraint";

    QString currentType = mpUi->comboBoxTypes->currentText();
    constraints::ModelConstraint::Type type = constraints::ModelConstraint::getTypeFromTxt(currentType.toStdString());

    QString model = mpUi->comboBoxModels->currentText();
    owlapi::model::IRI modelIRI( model.toStdString() );

    owlapi::model::IRI propertyIRI;
    if(mpUi->comboBoxProperties->isEnabled())
    {
        QString property = mpUi->comboBoxProperties->currentText();
        propertyIRI = property.toStdString();
    }

    int value = 0;
    if(mpUi->spinBoxValue->isEnabled())
    {
        value = mpUi->spinBoxValue->value();
    }
    std::vector<SpaceTime::SpaceIntervalTuple> affectedIntervals = getIntervals();

    return make_shared<constraints::ModelConstraint>(type,
            modelIRI,
            affectedIntervals,
            value,
            propertyIRI);
}

std::vector<SpaceTime::SpaceIntervalTuple> ModelConstraint::getIntervals() const
{
    qDebug() << "Get intervals;";
    std::vector<SpaceTime::SpaceIntervalTuple> affectedIntervals;
    return affectedIntervals;
}

void ModelConstraint::typeChanged(const QString& type)
{
    qDebug() << "Type changed to : " << type;
}

void ModelConstraint::modelChanged(const QString& model)
{
    qDebug() << "Model changed to : " << model;
}

} // namespace widgets
} // namespace gui
} // namespace templ
