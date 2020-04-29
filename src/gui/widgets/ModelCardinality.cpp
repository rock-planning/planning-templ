#include "ModelCardinality.hpp"
#include "ui_ModelCardinality.h"

namespace templ {
namespace gui {
namespace widgets {

ModelCardinality::ModelCardinality(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::ModelCardinality)
{
    mpUi->setupUi(this);
}

ModelCardinality::~ModelCardinality()
{
    delete mpUi;
}

void ModelCardinality::prepare(const organization_model::OrganizationModelAsk& ask)
{
    owlapi::model::IRIList agentModels = ask.getAgentModels();
    for(const owlapi::model::IRI iri : agentModels)
    {
        mpUi->comboBoxModels->addItem(QString::fromStdString( iri.toString() ) );
    }
}

void ModelCardinality::setRequirement(const io::ResourceRequirement& requirement)
{
    {
        QString model = QString::fromStdString( requirement.model.toString() );
        int index = mpUi->comboBoxModels->findText(model);
        if(index != -1)
        {
            mpUi->comboBoxModels->setCurrentIndex(index);
        } else {
            throw std::runtime_error("templ::gui::widgets::ModelCardinality::setRequirement:"
                    " no model " + model.toStdString() + " known - (internal error: did you call prepare?)");
        }
    }
    mpUi->spinBoxMin->setValue(requirement.minCardinality);
    mpUi->spinBoxMax->setValue(requirement.maxCardinality);
}

io::ResourceRequirement ModelCardinality::getRequirement() const
{
    io::ResourceRequirement requirement;
    requirement.model = mpUi->comboBoxModels->currentText().toStdString();
    requirement.minCardinality = mpUi->spinBoxMin->value();
    requirement.maxCardinality = mpUi->spinBoxMax->value();
    return requirement;
}

} // namespace widgets
} // namespace gui
} // namespace templ
