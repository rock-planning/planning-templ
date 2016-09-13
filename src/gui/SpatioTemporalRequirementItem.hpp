#ifndef TEMPL_GUI_SPATIO_TEMPORAL_REQUIREMENT_ITEM_HPP
#define TEMPL_GUI_SPATIO_TEMPORAL_REQUIREMENT_ITEM_HPP

#include <QGraphicsItem>
#include <QGraphicsLayoutItem>
#include <QGraphicsRectItem>

#include <graph_analysis/gui/GraphicsItemTypes.hpp>

namespace templ {
namespace gui {

class SpatioTemporalRequirementItem : public QGraphicsItem, public QGraphicsLayoutItem
{

public:
    SpatioTemporalRequirementItem(QGraphicsItem* parent = 0);

    virtual ~SpatioTemporalRequirementItem();

    virtual int type() const
    {
        return graph_analysis::gui::UserType + 1;
    };


    /**
     * callback to trigger the base-graph to adjust all edges of this vertex
     * after its position has changed in the scene.
     */
    virtual QVariant itemChange(GraphicsItemChange change, const QVariant& value);

    // inherited from QGraphicsLayoutItem
    virtual void setGeometry(const QRectF& geometry);
    virtual QSizeF sizeHint(Qt::SizeHint which, const QSizeF& constraint = QSizeF()) const;

    // inherited from QGraphicsItem
    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget = 0);
    virtual QRectF boundingRect() const;

protected:

    virtual void mousePressEvent(QGraphicsSceneMouseEvent* event);
    /**
     * this is used to update the statusbar of the current active widget with
     * information about the currently hovered (or not hovered) vertex
     */
    virtual void hoverEnterEvent(QGraphicsSceneHoverEvent* event);
    virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent* event);

    QGraphicsRectItem* mpRect;

};

} // end namespace templ
} // end namespace templ
#endif // TEMPL_GUI_SPATIO_TEMPORAL_REQUIREMENT_ITEM_HPP
