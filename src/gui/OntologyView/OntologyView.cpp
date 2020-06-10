#include "OntologyView.hpp"
#include "ui_OntologyView.h"

#include <QFileDialog>
#include <moreorg/vocabularies/OM.hpp>

namespace templ {
namespace gui {

OntologyView::OntologyView(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::OntologyView)
    , mpFunctionalitiesTreeModel(NULL)
    , mpAgentsTreeModel(NULL)
{
    mpUi->setupUi(this);

    //connect(mpUi->loadOntologyButton,
    //        SIGNAL(clicked()),
    //        this,
    //        SLOT(on_loadOntologyButton_clicked()));
}

OntologyView::~OntologyView()
{
    delete mpUi;
}

void OntologyView::updateVisualization()
{
}

void OntologyView::on_loadOntologyButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load ontology"),
        QDir::currentPath(),
        tr("Ontology File (*.owl)"));

    if(!filename.isEmpty())
    {
        mpOntology = owlapi::model::OWLOntology::fromFile(filename.toStdString());
        OntologyTreeModel* oldFunctionalitiesTreeModel = mpFunctionalitiesTreeModel;
        OntologyTreeModel* oldAgentsTreeModel = mpAgentsTreeModel;
        {
            mpFunctionalitiesTreeModel = new OntologyTreeModel(mpOntology, moreorg::vocabulary::OM::Functionality(), this);
            mpUi->functionalitiesTreeView->setModel(mpFunctionalitiesTreeModel);

            mpAgentsTreeModel = new OntologyTreeModel(mpOntology, moreorg::vocabulary::OM::Actor());
            mpUi->agentsTreeView->setModel(mpAgentsTreeModel);
        }
        updateVisualization();

        delete oldFunctionalitiesTreeModel;
        delete oldAgentsTreeModel;
    } else {
        LOG_WARN_S << "Filename is empty";
    }
}


} // end namespace gui
} // end namespace templ
