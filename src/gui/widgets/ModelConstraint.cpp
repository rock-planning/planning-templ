#include "ModelConstraint.hpp"
#include "ui_ModelConstraint.h"
#include "../Utils.hpp"

#include <owlapi/model/IRI.hpp>

#include <QString>
#include <QDebug>
#include <QCheckBox>

namespace templ {
namespace gui {
namespace widgets {

ModelConstraint::ModelConstraint(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::ModelConstraint)
{
    mpUi->setupUi(this);
    connect(mpUi->pushButtonAddRequirement, SIGNAL(clicked()),
            this, SLOT(addRequirement()));
    connect(mpUi->pushButtonRemoveRequirements, SIGNAL(clicked()),
            this, SLOT(removeRequirements()));

    mpUi->labelProperty->setVisible(false);
    mpUi->comboBoxProperties->setVisible(false);
    mpUi->labelValue->setVisible(false);
    mpUi->spinBoxValue->setVisible(false);
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
    connect(mpUi->comboBoxTypes, SIGNAL(currentIndexChanged(QString)),
                this, SLOT(typeChanged(QString)));

    owlapi::model::IRIList agentModels = ask.getAgentModels();
    for(const owlapi::model::IRI iri : agentModels)
    {
        mpUi->comboBoxModels->addItem(QString::fromStdString( iri.toString() ) );
    }

    connect(mpUi->comboBoxModels, SIGNAL(currentIndexChanged(QString)),
                this, SLOT(modelChanged(QString)));

    owlapi::model::IRIList agentProperties = ask.getAgentProperties();
    for(const owlapi::model::IRI iri : agentProperties)
    {
        mpUi->comboBoxProperties->addItem(QString::fromStdString( iri.toString() ) );
    }

}

void ModelConstraint::setButtonVisibility(bool visible)
{
    mpUi->pushButtonAddRequirement->setVisible(visible);
    mpUi->pushButtonRemoveRequirements->setVisible(visible);
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

void ModelConstraint::typeChanged(const QString& typeTxt)
{
    qDebug() << "Type changed to : " << typeTxt;
    constraints::ModelConstraint::Type type =
        constraints::ModelConstraint::getTypeFromTxt(typeTxt.toStdString());

    mpUi->labelProperty->setVisible(false);
    mpUi->comboBoxProperties->setVisible(false);
    mpUi->labelValue->setVisible(false);
    mpUi->spinBoxValue->setVisible(false);

    switch(type)
    {
        case constraints::ModelConstraint::MIN_PROPERTY:
        case constraints::ModelConstraint::MAX_PROPERTY:
            mpUi->labelProperty->setVisible(true);
            mpUi->comboBoxProperties->setVisible(true);
        case constraints::ModelConstraint::MIN_FUNCTION:
        case constraints::ModelConstraint::MAX_FUNCTION:
        case constraints::ModelConstraint::MIN_EQUAL:
        case constraints::ModelConstraint::MAX_EQUAL:
        case constraints::ModelConstraint::MIN_DISTINCT:
        case constraints::ModelConstraint::MAX_DISTINCT:
        case constraints::ModelConstraint::MAX_ACCESS:
        case constraints::ModelConstraint::MIN_ACCESS:
        case constraints::ModelConstraint::MAX:
        case constraints::ModelConstraint::MIN:
            mpUi->labelValue->setVisible(true);
            mpUi->spinBoxValue->setVisible(true);
            break;
        case constraints::ModelConstraint::ALL_EQUAL:
        case constraints::ModelConstraint::ALL_DISTINCT:
            break;
        default:
            break;
    }
}

void ModelConstraint::modelChanged(const QString& model)
{
    qDebug() << "Model changed to : " << model;
}

QComboBox* ModelConstraint::addRequirement()
{
    QHBoxLayout* rowLayout = new QHBoxLayout;
    QCheckBox* checkbox = new QCheckBox;
    rowLayout->addWidget(checkbox);

    QComboBox* dropbox = new QComboBox;
    for(const QString& option : mRequirements)
    {
        dropbox->addItem(option);
    }

    rowLayout->addWidget(dropbox);

    mpUi->verticalLayoutRequirements->addLayout(rowLayout);
    return dropbox;
}

void ModelConstraint::updateRequirements(const QList<QString>& options)
{
    // cache latest update for new additions
    mRequirements = options;
    mRequirements.insert(0, QString());
    QList<QComboBox*> requirements = Utils::getWidgets<QComboBox>(mpUi->verticalLayoutRequirements);
    for(QComboBox* dropbox : requirements)
    {
        QString currentText = dropbox->currentText();
        dropbox->clear();

        for(const QString& option : mRequirements)
        {
            dropbox->addItem(option);
        }

        if(!currentText.isEmpty())
        {
            int index = dropbox->findText(currentText);
            if(index == -1)
            {
                qDebug() << "debug cleanup requirement -- requirement " << currentText <<
                    " does not exist anymore";
                dropbox->setCurrentIndex(0);
            } else {
                dropbox->setCurrentIndex(index);
            }
        }
    }

}

void ModelConstraint::removeRequirements()
{
    Utils::removeCheckedRows(mpUi->verticalLayoutRequirements);
}

} // namespace widgets
} // namespace gui
} // namespace templ
