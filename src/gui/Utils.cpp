#include "Utils.hpp"
#include <QList>
#include <QCheckBox>
#include <QDebug>

namespace templ {
namespace gui {

void Utils::removeCheckedRows(QLayout* parentLayout)
{
    QList<QHBoxLayout*> removeLayouts;
    for(int i = 0; i < parentLayout->count();++i)
    {
        QHBoxLayout* rowLayout = qobject_cast<QHBoxLayout*>(parentLayout->itemAt(i)->layout());
        if(rowLayout)
        {
            QCheckBox* checkbox =
                qobject_cast<QCheckBox*>(rowLayout->itemAt(0)->widget());

            if(checkbox && checkbox->checkState() == Qt::Checked)
            {
                removeLayouts.append(rowLayout);
            }
        }
    }

    for(int i = 0; i < removeLayouts.size(); ++i)
    {
        removeRow(parentLayout, removeLayouts[i]);
    }
}

void Utils::removeRows(QLayout* parentLayout)
{
    QList<QHBoxLayout*> removeLayouts;
    for(int i = 0; i < parentLayout->count();++i)
    {
        QHBoxLayout* rowLayout = qobject_cast<QHBoxLayout*>(parentLayout->itemAt(i)->layout());
        if(rowLayout)
        {
            removeLayouts.append(rowLayout);
        }
    }

    for(int i = 0; i < removeLayouts.size(); ++i)
    {
        removeRow(parentLayout, removeLayouts[i]);
    }
}

void Utils::removeRow(QLayout* parent, QHBoxLayout* rowLayout)
{
    while(rowLayout->count() != 0)
    {
        QLayoutItem* item = rowLayout->itemAt(0);
        if(!item)
        {
            break;
        }

        QWidget* widget = item->widget();
        if(widget)
        {
            rowLayout->removeWidget(widget);
            delete widget;
        } else {
            rowLayout->removeItem(item);
            delete item;
        }
    }

    parent->removeItem(rowLayout);
    delete rowLayout;
}


} // end namespace gui
} // end namespace templ
