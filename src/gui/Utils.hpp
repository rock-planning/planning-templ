#ifndef TEMPL_GUI_UTILS_HPP
#define TEMPL_GUI_UTILS_HPP

#include <QLayout>
#include <QHBoxLayout>

namespace templ {
namespace gui {

class Utils
{
public:
    static void removeCheckedRows(QLayout* parentLayout);
    static void removeRows(QLayout* parent);
    static void removeRow(QLayout* parent, QHBoxLayout* rowLayout);
    template<typename T>
    static QList<T*> getWidgets(QLayout* parentLayout, int col = 1)
    {
        QList<T*> widgets;
        for(int i = 0; i < parentLayout->count(); ++i)
        {
            QHBoxLayout* rowLayout =
                qobject_cast<QHBoxLayout*>(parentLayout->itemAt(i)->layout());

            if(rowLayout)
            {
                T* customWidget = qobject_cast<T*>(rowLayout->itemAt(col)->widget());
                if(customWidget)
                {
                    widgets.append(customWidget);
                }
            }
        }
        return widgets;
    }

};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_UTILS_HPP
