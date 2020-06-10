#ifndef TEMPL_GUI_MODELS_AGENT_STYLE_MODEL_HPP
#define TEMPL_GUI_MODELS_AGENT_STYLE_MODEL_HPP

#include <QAbstractTableModel>
#include <moreorg/ModelPool.hpp>
#include "../../Role.hpp"

namespace templ {
namespace gui {
namespace models {
class AgentStyleModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    AgentStyleModel(const moreorg::ModelPool& modelPool);

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;

    int columnCount(const QModelIndex& parent = QModelIndex()) const override;

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;

    /**
     * For horizontal headers as stated in qt documentation, section number
     * corresponds to the column number
     */
    QVariant headerData(int section,
            Qt::Orientation orientation,
            int role = Qt::DisplayRole) const override;

    bool setData(const QModelIndex& index,
            const QVariant& value,
            int role = Qt::EditRole) override;

    Qt::ItemFlags flags(const QModelIndex& index) const;

public slots:
    void setColor(const QModelIndex& modelIndex);

private:
    moreorg::ModelPool mModelPool;
    Role::List mRoles;

    QMap< QModelIndex, QMap<int, QVariant> > mModelData;
    int mNumberOfColumns;
};

} // end namespace models
} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_MODELS_AGENT_STYLE_MODEL_HPP
