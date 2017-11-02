#ifndef TEMPL_GUI_EDGE_ITEMS_ROLE_INFO_ITEM_HPP
#define TEMPL_GUI_EDGE_ITEMS_ROLE_INFO_ITEM_HPP

#include <graph_analysis/gui/EdgeItemBase.hpp>
#include "../../RoleInfoWeightedEdge.hpp"

namespace templ {
namespace gui {
namespace edge_items {

/**
 *
 * simplest possible implementation of EdgeItemBase: just a line from source to target
 *
 */
class RoleInfoItem : public graph_analysis::gui::EdgeItemBase
{

public:
    RoleInfoItem() {}
    RoleInfoItem(graph_analysis::gui::GraphWidget* graphWidget,
            const graph_analysis::Edge::Ptr& edge,
            QGraphicsItem* parent);
    virtual ~RoleInfoItem();
    virtual int type() const;

    void adjustEdgePositioning();

protected:
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget = 0);
    QRectF boundingRect() const;

    int mArrowSize;
    QGraphicsTextItem* mpLabel;
    QGraphicsTextItem* mpClassName;

    EdgeItemBase* createNewItem(graph_analysis::gui::GraphWidget* graphWidget,
                const graph_analysis::Edge::Ptr& edge,
                QGraphicsItem* parent) const;
};

} // end namespace edge_items
} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_EDGE_ITEMS_ROLE_INFO_ITEM_HPP
