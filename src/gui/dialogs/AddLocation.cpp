#include "AddLocation.hpp"
#include <QDialog>
#include "ui_AddLocation.h"

namespace templ {
namespace gui {
namespace dialogs {

AddLocation::AddLocation(QWidget* parent)
    : QDialog(parent)
    , mpUi(new Ui::Dialog)
{
    mpUi->setupUi(this);
}

AddLocation::~AddLocation()
{
    delete mpUi;
}

symbols::constants::Location::Ptr AddLocation::getLocation() const
{
    QString name = mpUi->lineEditName->text();
    base::Point position( mpUi->doubleSpinBoxX->value(),
            mpUi->doubleSpinBoxY->value(),
            mpUi->doubleSpinBoxZ->value()
    );

    return make_shared<symbols::constants::Location>(name.toStdString(),
            position);
}

} // end namespace dialogs
} // end namespace gui
} // end namespace templ
