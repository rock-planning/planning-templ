#ifndef TEMPL_GUI_MOUSE_WHEEL_GUARD_HPP
#define TEMPL_GUI_MOUSE_WHEEL_GUARD_HPP

#include <QEvent>
#include <QWidget>

namespace templ {
namespace gui {

class MouseWheelGuard : public QObject
{
public:
    explicit MouseWheelGuard(QObject* parent);

protected:
    bool eventFilter(QObject* o, QEvent* e) override;
};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_MOUSE_WHEEL_GUARD_HPP
