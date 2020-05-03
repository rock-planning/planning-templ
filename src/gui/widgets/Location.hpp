#ifndef TEMPL_GUI_WIDGETS_LOCATION_HPP
#define TEMPL_GUI_WIDGETS_LOCATION_HPP

#include "../../symbols/constants/Location.hpp"

#include <QWidget>

namespace Ui
{
    class Location;
}

namespace templ {
namespace gui {
namespace widgets {

class Location : public QWidget
{
    Q_OBJECT

public:
    Location(QWidget* parent = NULL);
    ~Location();

    void setValue(const symbols::constants::Location::Ptr& location);

    symbols::constants::Location::Ptr getValue() const;

    QString getLocationName() const;
private:
    Ui::Location* mpUi;
};

} // namespace widgets
} // namespace gui
} // namespace templ

#endif // TEMPL_GUI_WIDGETS_LOCATION_HPP
