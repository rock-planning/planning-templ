#include "SpatioTemporalRequirementItem.hpp"
#include <QPen>
#include <base-logging/Logging.hpp>

namespace templ {
namespace gui {

SpatioTemporalRequirementItem::SpatioTemporalRequirementItem(QGraphicsItem* parent)
    : QGraphicsItem(parent)
    , QGraphicsLayoutItem()
{
    // this enabled "itemChange()" notifications. when this item moves, it has
    // to tell its edges to follow it, so they stay visually connected. this is
    // done by calling "adjust()" for the respective edge
    setFlag(ItemSendsScenePositionChanges);
    setFlag(ItemIsMovable);

    // this cache-mode is for items that can move. not sure if we can move --
    // vertices can move?
    setCacheMode(DeviceCoordinateCache);

    // drag'n drop is used to add requirements
    setAcceptHoverEvents(true);

    // for QGraphicsLayoutItem
    setGraphicsItem(this);


    // at the lowest (so in the background) the rectangle
    mpRect = new QGraphicsRectItem(this);
    mpRect->setPen(QPen(Qt::blue));
    mpRect->setRect(QRectF(0,0,20,20));
}

SpatioTemporalRequirementItem::~SpatioTemporalRequirementItem()
{
    delete mpRect;
}

void SpatioTemporalRequirementItem::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
    LOG_WARN_S << "MOUSE PRESS";
    QGraphicsItem::mousePressEvent(event);
}

void SpatioTemporalRequirementItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    LOG_WARN_S << "HOVER";
    QGraphicsItem::hoverEnterEvent(event);
}

void SpatioTemporalRequirementItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    QGraphicsItem::hoverLeaveEvent(event);
}

QVariant SpatioTemporalRequirementItem::itemChange(GraphicsItemChange change,
                                    const QVariant& value)
{
    switch(change)
    {
    case ItemScenePositionHasChanged:
    {
        // notify the graph widget about our new position. there, relevant
        // caching and updating of connected items is performed.
        break;
    }
    default:
    {
        break;
    }
    };
    return QGraphicsItem::itemChange(change, value);
}

void SpatioTemporalRequirementItem::setGeometry(const QRectF& geometry)
{
    prepareGeometryChange();
    QGraphicsLayoutItem::setGeometry(geometry);
    setPos(geometry.topLeft());
}

QSizeF SpatioTemporalRequirementItem::sizeHint(Qt::SizeHint which, const QSizeF& constraint) const
{
    switch (which) {
        case Qt::MinimumSize:
        case Qt::PreferredSize:
            // Do not allow a size smaller than 30x30
            return QSize(30, 30);
        case Qt::MaximumSize:
            return QSizeF(1000,1000);
        default:
            break;
    }
    return constraint;
}

void SpatioTemporalRequirementItem::paint(QPainter* painter,
                             const QStyleOptionGraphicsItem* option,
                             QWidget* widget)
{
}

QRectF SpatioTemporalRequirementItem::boundingRect() const
{
    return childrenBoundingRect();
}

} // end namespace templ
} // end namespace templ
