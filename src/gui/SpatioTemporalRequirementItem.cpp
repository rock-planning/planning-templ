#include "SpatioTemporalRequirementItem.hpp"
#include <QPen>
#include <base-logging/Logging.hpp>
#include <owlapi/model/OWLCardinalityRestriction.hpp>

#include "../symbols/object_variables/LocationCardinality.hpp"

namespace templ {
namespace gui {

SpatioTemporalRequirementItem::SpatioTemporalRequirementItem(QGraphicsItem* parent)
    : QGraphicsItem(parent)
    , QGraphicsLayoutItem()
{
    // at the lowest (so in the background) the rectangle
    mpRect = new QGraphicsRectItem(this);
    mpRect->setPen(QPen(Qt::blue));

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


    mpFromTimePoint = new QGraphicsTextItem("from: ", this);
    mpToTimePoint = new QGraphicsTextItem("to: ", this);
    mpLocation = new QGraphicsTextItem("at: ", this);
    mpResource =  new QGraphicsTextItem(" ", this);
    mpCardinality =  new QGraphicsTextItem(" ", this);

    updateFromPersistenceCondition();

}

SpatioTemporalRequirementItem::~SpatioTemporalRequirementItem()
{
    delete mpRect;
}

void SpatioTemporalRequirementItem::updateFromPersistenceCondition()
{
    std::string from = "from: ";
    std::string to = "to: ";
    std::string location = "at: ";
    std::string resource = "";
    std::string cardinality = "";

    if(mpPersistenceCondition)
    {
        from += mpPersistenceCondition->getFromTimePoint()->getLabel();
        to += mpPersistenceCondition->getToTimePoint()->getLabel();

        Symbol::Ptr symbol = mpPersistenceCondition->getValue();
        symbols::object_variables::LocationCardinality::Ptr locationCardinality = dynamic_pointer_cast<symbols::object_variables::LocationCardinality>(symbol);
        resource += owlapi::model::IRI(mpPersistenceCondition->getStateVariable().getResource()).getFragment();
        if(locationCardinality)
        {
            location += locationCardinality->getLocation()->getInstanceName();
            std::string typeTxt = "?";
            using namespace owlapi::model;
            switch(locationCardinality->getCardinalityRestrictionType())
            {
                case OWLCardinalityRestriction::MIN:
                    typeTxt = ">=";
                    break;
                case OWLCardinalityRestriction::MAX:
                    typeTxt = "<=";
                    break;
                case OWLCardinalityRestriction::EXACT:
                    typeTxt = "=";
                    break;
                default:
                    break;
            }

            std::stringstream ss;
            ss << locationCardinality->getCardinality();
            cardinality = typeTxt + " " +ss.str();
        }
    }

    mpFromTimePoint->setPlainText(from.c_str());
    mpToTimePoint->setPlainText(to.c_str());
    mpLocation->setPlainText(location.c_str());
    mpResource->setPlainText(resource.c_str());
    mpCardinality->setPlainText(cardinality.c_str());

    mpFromTimePoint->setPos(mpRect->rect().topLeft());
    // First add the placeholder to get the proper bounding rect from all
    // children
    // Aftwards recompute the position to align at the top right of the
    // rectangle
    mpToTimePoint->setPos(mpFromTimePoint->pos() + QPointF(mpFromTimePoint->boundingRect().width() + 10,0));

    mpLocation->setPos(mpFromTimePoint->pos() +
            QPoint(0, mpFromTimePoint->boundingRect().height()));
    mpResource->setPos(mpLocation->pos() +
            QPoint(0, mpLocation->boundingRect().height()));
    mpCardinality->setPos(mpResource->pos() +
            QPoint(0, mpResource->boundingRect().height()));

    // now that all the children are there, we use their bounding-rect to
    // enlarge the background-rect. note that we never modify the boundingRect
    // afterwards.
    mpRect->setRect(childrenBoundingRect());

    mpToTimePoint->setPos(mpRect->rect().topRight() -
            QPointF(mpToTimePoint->boundingRect().width(), 0));

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
            return QSizeF(childrenBoundingRect().width(), childrenBoundingRect().height());
        case Qt::MaximumSize:
            return QSizeF(10000,10000);
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
