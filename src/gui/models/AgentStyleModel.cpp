#include "AgentStyleModel.hpp"
#include <iostream>
#include <QColor>
#include <QColorDialog>

#include <moreorg/ModelPool.hpp>
#include "../../Role.hpp"

using namespace moreorg;

namespace templ {
namespace gui {
namespace models {

// http://doc.qt.io/qt-5/qt.html#ItemDataRole-enum
// Qt::DecorationRole
// Qt::EditRole
// Qt::CheckedStateRole (Qt::CheckState)
//

AgentStyleModel::AgentStyleModel(const moreorg::ModelPool& modelPool)
    : QAbstractTableModel()
    , mModelPool(modelPool)
{
    int row = 0;
    for(const ModelPool::value_type& p : mModelPool)
    {
        for(size_t i = 0; i < p.second; ++i, ++row)
        {
            Role role(row, p.first);
            mRoles.push_back(role);
            {
                QModelIndex idx = createIndex(row, 0);
                QVariant value = QString(role.getName().c_str());
                setData(idx, value, Qt::DisplayRole);
                QVariant checkState = Qt::Checked;
                setData(idx, checkState, Qt::CheckStateRole);
            }

            {
                QModelIndex idx = createIndex(row, 1);
                QVariant value = (QColor)Qt::black;
                setData(idx, value, Qt::BackgroundColorRole);
            }

            {
                QModelIndex idx = createIndex(row, 2);
                QVariant value = QString(role.getModel().toString().c_str());
                setData(idx, value, Qt::DisplayRole);
            }
        }
    }

}

int AgentStyleModel::rowCount(const QModelIndex& parent) const
{
    return mRoles.size();
}

int AgentStyleModel::columnCount(const QModelIndex& parent) const
{
    return 3;
}

QVariant AgentStyleModel::data(const QModelIndex& index, int role) const
{

    //switch(role)
    //{
    //    //case Qt::DisplayRole:
    //    //case Qt::ForegroundRole:
    //    //    //return Qt::blue;
    //    //case Qt::BackgroundColorRole:
    //    //    return Qt::red;
    //    //case Qt::CheckStateRole:
    //    //    return Qt::Checked;
    //}
    //return QVariant();
    return mModelData[index][role];
}

QVariant AgentStyleModel::headerData(int section,
        Qt::Orientation orientation,
        int role) const
{
    switch(role)
    {
        case Qt::DisplayRole:
            if(orientation == Qt::Horizontal)
            {
                switch(section)
                {
                    case 0:
                        return QString("Name");
                    case 1:
                        return QString("Color");
                    case 2:
                        return QString("Model");
                }
            }
            if(orientation == Qt::Vertical)
            {
                return QString::number(section);
            }
        //case Qt::ForegroundRole:
        //    return Qt::yellow;
        //case Qt::BackgroundColorRole:
        //    return Qt::blue;
    }
    return QVariant();
}

bool AgentStyleModel::setData(const QModelIndex& index,
        const QVariant& value,
        int role)
{
    QMap<int, QVariant> itemDataMap = mModelData[index];
    switch(role)
    {
        case Qt::EditRole:
        case Qt::DisplayRole:
        case Qt::BackgroundColorRole:
        case Qt::ForegroundRole:
        case Qt::CheckStateRole:
            itemDataMap[role] = value;
            mModelData[index] = itemDataMap;
            break;
    }
    return true;
}

Qt::ItemFlags AgentStyleModel::flags(const QModelIndex& index) const
{
    return Qt::ItemIsUserCheckable | QAbstractTableModel::flags(index);
}

void AgentStyleModel::setColor(const QModelIndex& modelIndex)
{
    if(modelIndex.column() == 1)
    {
        QColor currentColor = data(modelIndex, Qt::BackgroundColorRole).value<QColor>();
        QColor color = QColorDialog::getColor(currentColor);
        setData(modelIndex, color, Qt::BackgroundColorRole);
    }
}

} // end namespace models
} // end namespace gui
} // end namespace templ
