#include "MouseWheelGuard.hpp"

namespace templ {
namespace gui {

MouseWheelGuard::MouseWheelGuard(QObject* parent)
{
}

bool MouseWheelGuard::eventFilter(QObject* o, QEvent* e)
{
    const QWidget* widget = static_cast<QWidget*>(o);
    if(widget && !widget->hasFocus() && e->type() == QEvent::Wheel)
    {
        e->ignore();
        return true;
    }

    return QObject::eventFilter(o, e);
}


} // end namespace gui
} // end namespace templ
