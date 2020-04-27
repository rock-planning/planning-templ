#include "Location.hpp"
#include "ui_Location.h"

namespace templ {
namespace gui {
namespace widgets {

Location::Location(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::Location)
{
    mpUi->setupUi(this);
}

Location::~Location()
{
    delete mpUi;
}

void Location::setValue(const symbols::constants::Location::Ptr& location)
{
    if(!location)
    {
        return;
    }

    mpUi->lineEditName->setText(QString::fromStdString(location->getInstanceName()));
    mpUi->doubleSpinBoxX->setValue(location->getPosition().x());
    mpUi->doubleSpinBoxY->setValue(location->getPosition().y());
    mpUi->doubleSpinBoxZ->setValue(location->getPosition().z());
}

symbols::constants::Location::Ptr Location::getValue() const
{
    QString name = mpUi->lineEditName->text();
    base::Point position( mpUi->doubleSpinBoxX->value(),
            mpUi->doubleSpinBoxY->value(),
            mpUi->doubleSpinBoxZ->value()
    );
    return make_shared<symbols::constants::Location>(name.toStdString(),
            position);
}

} // namespace widgets
} // namespace gui
} // namespace templ
