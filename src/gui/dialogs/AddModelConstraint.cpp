#include "AddModelConstraint.hpp"
#include <QDialog>
#include "ui_AddModelConstraint.h"

namespace templ {
namespace gui {
namespace dialogs {

AddModelConstraint::AddModelConstraint(
        const organization_model::OrganizationModelAsk& ask,
        QWidget* parent
        )
    : QDialog(parent)
    , mpUi(new Ui::AddModelConstraint)
    , mpModelConstraint(new widgets::ModelConstraint)
    , mAsk(ask)
{
    mpUi->setupUi(this);
    mpModelConstraint->prepare(mAsk);
    mpUi->gridLayoutModelConstraint->addWidget(mpModelConstraint);
}

AddModelConstraint::~AddModelConstraint()
{
    delete mpUi;
}

constraints::ModelConstraint::Ptr AddModelConstraint::getConstraint() const
{
    return mpModelConstraint->getConstraint();
}

} // end namespace dialogs
} // end namespace gui
} // end namespace templ
