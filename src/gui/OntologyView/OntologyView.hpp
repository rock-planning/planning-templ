#ifndef TEMPL_GUI_ONTOLOGY_VIEW_ONTOLOGY_VIEW_HPP
#define TEMPL_GUI_ONTOLOGY_VIEW_ONTOLOGY_VIEW_HPP

#include <owlapi/OWLApi.hpp>
#include <QWidget>

#include <templ/gui/OntologyView/OntologyTreeModel.hpp>

namespace Ui {
    class OntologyView;
}

namespace templ {
namespace gui {

class OntologyView : public QWidget
{
    Q_OBJECT

public:
    OntologyView(QWidget* parent = NULL);
    ~OntologyView();

    QString getClassName() const
    {
        return "templ::gui::OntologyView";
    }

    void updateVisualization();

private:
    Ui::OntologyView* mpUi;

private slots:
    void on_loadOntologyButton_clicked();

protected:

    owlapi::model::OWLOntology::Ptr mpOntology;

    OntologyTreeModel* mpFunctionalitiesTreeModel;
    OntologyTreeModel* mpAgentsTreeModel;
};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_ONTOLOGY_VIEW_ONTOLOGY_VIEW_HPP
