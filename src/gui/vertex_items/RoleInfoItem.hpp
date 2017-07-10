#ifndef TEMPL_GUI_VERTEX_ITEMS_ROLE_INFO_ITEM_HPP
#define TEMPL_GUI_VERTEX_ITEMS_ROLE_INFO_ITEM_HPP

#include <graph_analysis/gui/VertexItemBase.hpp>

namespace templ {
namespace gui {
namespace vertex_items {

/**
 * Simplest possible implementation of a VertexItemBase
 * just a box with two strings -- type and
 * label
 */
class RoleInfoItem : public graph_analysis::gui::VertexItemBase
{
    friend class VertexItemTypeManager;

public:
    RoleInfoItem();

    RoleInfoItem(graph_analysis::gui::GraphWidget* graphWidget,
                    const graph_analysis::Vertex::Ptr& vertex,
                    QGraphicsItem* parent);

    virtual ~RoleInfoItem();

    virtual int type() const
    {
        return static_cast<int>(graph_analysis::gui::UserType) + 1;
    };

    /**
     * uses this (read only) notification to update the label showing current
     * canvas coordinate of the whole item. nice for debugging.
     */
    virtual QVariant itemChange(GraphicsItemChange change, const QVariant& value);

protected:
    /**
     * all items by default accept drag-n-drop events. override this function
     * in later classes to prevent certain drops?
     */
    void dragEnterEvent(QGraphicsSceneDragDropEvent* event);
    void dragLeaveEvent(QGraphicsSceneDragDropEvent* event);
    void dropEvent(QGraphicsSceneDragDropEvent* event);
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event);

    void hoverEnterEvent(QGraphicsSceneHoverEvent* event);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent* event);

    VertexItemBase* createNewItem(graph_analysis::gui::GraphWidget* graphWidget,
                const graph_analysis::Vertex::Ptr& vertex,
                QGraphicsItem* parent) const;

private:
    QGraphicsTextItem* mpLabel;
    QGraphicsTextItem* mpClassName;
    QGraphicsTextItem* mpInfoBox;
    QGraphicsRectItem* mpRect;

    /** convert the current scenePos of the item to smth like "(23, -14)" */
    QString getScenePosAsString() const;
};

} // end namespace vertex_items
} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_VERTEX_ITEMS_ROLE_INFO_ITEM_HPP
