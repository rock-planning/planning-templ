#ifndef TEMPL_GUI_DIALOGS_ADD_LOCATIONS_HPP
#define TEMPL_GUI_DIALOGS_ADD_LOCATIONS_HPP

#include <QDialog>
#include "../../symbols/constants/Location.hpp"

namespace Ui
{
    class Dialog;
}

namespace templ {
namespace gui {
namespace dialogs {

class AddLocation : public QDialog
{
    Q_OBJECT

public:
    AddLocation(QWidget* parent = NULL);
    ~AddLocation();

    symbols::constants::Location::Ptr getLocation() const;

private:
    Ui::Dialog* mpUi;
};

} // end namespace dialogs
} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_DIALOGS_ADD_LOCATION_HPP
