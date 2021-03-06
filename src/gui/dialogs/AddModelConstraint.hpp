#ifndef TEMPL_GUI_DIALOGS_ADD_MODEL_CONSTRAINT_HPP
#define TEMPL_GUI_DIALOGS_ADD_MODEL_CONSTRAINT_HPP

#include "../../constraints/ModelConstraint.hpp"
#include "../widgets/ModelConstraint.hpp"

#include <QDialog>
#include <moreorg/OrganizationModelAsk.hpp>

namespace Ui
{
    class EmptyDialog;
}

namespace templ {
namespace gui {
namespace dialogs {

class AddModelConstraint : public QDialog
{
    Q_OBJECT

public:
    AddModelConstraint(const moreorg::OrganizationModelAsk& ask,
            QWidget* parent = NULL
    );
    ~AddModelConstraint();

    constraints::ModelConstraint::Ptr getConstraint() const;

private:
    Ui::EmptyDialog* mpUi;
    widgets::ModelConstraint* mpModelConstraint;

    moreorg::OrganizationModelAsk mAsk;
};

} // end namespace dialogs
} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_DIALOGS_ADD_LOCATION_HPP
