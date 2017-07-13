#include "CapacityLinkItem.hpp"
#include <graph_analysis/gui/GraphWidget.hpp>
#include <base-logging/Logging.hpp>
#include <cmath>

using namespace graph_analysis::gui;

namespace templ {
namespace gui {
namespace edge_items {
// kiss:
CapacityLinkItem::CapacityLinkItem(graph_analysis::gui::GraphWidget* graphWidget,
                               const graph_analysis::Edge::Ptr& edge,
                               QGraphicsItem* parent)
    : EdgeItemBase(graphWidget, edge, parent)
    , mArrowSize(10)
{
    mpLabel = new QGraphicsTextItem("", this);
    mpClassName =
        new QGraphicsTextItem(QString(edge->getClassName().c_str()), this);
    mpClassName->setDefaultTextColor(Qt::gray);
    mpMultiLine = new QGraphicsPolygonItem(this);
    mpMultiLine->setBrush(Qt::NoBrush);
    mpArrowHead = new QGraphicsPolygonItem(this);
    mpArrowHead->setBrush(QBrush(Qt::black));

    mpFillBar = new QGraphicsRectItem(mpMultiLine->pos().x(), mpMultiLine->pos().y(), 100, 10, this);
    mpFillBar->setPen( QPen(Qt::black) );
    mpFillBar->setBrush(QBrush(Qt::white));

    double consumptionInPercent = dynamic_pointer_cast<CapacityLink>( getEdge() )->getConsumptionLevel();

    mpFillStatus = new QGraphicsRectItem(mpMultiLine->pos().x(), mpMultiLine->pos().y(), consumptionInPercent*100 , 10, this);
    mpFillStatus->setPen( QPen(Qt::black) );
    mpFillStatus->setBrush(QBrush(Qt::black));

    setFlag(ItemIsMovable, false);
}

CapacityLinkItem::~CapacityLinkItem()
{
    delete mpLabel;
    delete mpClassName;
    delete mpMultiLine;
    delete mpArrowHead;
}

int CapacityLinkItem::type() const
{
    return static_cast<int>(graph_analysis::gui::UserType) + 11;
}

void CapacityLinkItem::adjustEdgePositioning()
{
    prepareGeometryChange();

    mpMultiLine->setPolygon(mPoints);
    mpLabel->setPos(mpMultiLine->boundingRect().center() -
                    mpLabel->boundingRect().center());
    mpClassName->setPos(mpLabel->pos() +
                        QPointF(0, mpLabel->boundingRect().height()));

    mpFillBar->setPos(mpLabel->pos().x(), mpLabel->pos().y() - mpFillBar->rect().height());
    mpFillStatus->setPos(mpLabel->pos().x(), mpLabel->pos().y() - mpFillStatus->rect().height());

    // draw the arrow!
    QLineF lastSegment(mPoints.at((size_t)mPoints.size() - 2),
                       mPoints.at((size_t)mPoints.size() - 1));
    double angle = std::acos(lastSegment.dx() / lastSegment.length());
    // in case this is a very short edge we cannot infer how to actually draw
    // the arrow. in this case we'll fall back to not draw it.
    if(std::isnan(angle))
    {
        mpArrowHead->setPolygon(QPolygonF());
        return;
    }
    if(lastSegment.dy() >= 0)
    {
        angle = 2 * M_PI - angle;
    }

    QPointF destArrowP1 =
        mPoints.last() + QPointF(sin(angle - M_PI / 3) * mArrowSize,
                                          cos(angle - M_PI / 3) * mArrowSize);
    QPointF destArrowP2 = mPoints.last() +
                          QPointF(sin(angle - M_PI + M_PI / 3) * mArrowSize,
                                  cos(angle - M_PI + M_PI / 3) * mArrowSize);
    mpArrowHead->setPolygon(QPolygonF() << mPoints.last() << destArrowP1
                                        << destArrowP2);
}

void CapacityLinkItem::paint(QPainter* painter,
                           const QStyleOptionGraphicsItem* option,
                           QWidget* widget)
{
}

QRectF CapacityLinkItem::boundingRect() const
{
    return childrenBoundingRect();
}

// reimplement "shape()" because the default implementation calls
// "boundingRect()" -- we are not rect!
QPainterPath CapacityLinkItem::shape() const
{
    QPainterPath path;
    path = mpMultiLine->shape() + mpArrowHead->shape() + mpLabel->shape() +
           mpClassName->shape();
    return path;
}

void CapacityLinkItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    mpClassName->setPlainText("");
    mpLabel->setPlainText(getEdge()->toString().c_str());
}

void CapacityLinkItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    mpClassName->setPlainText(getEdge()->getClassName().c_str());
    mpLabel->setPlainText("");
}

EdgeItemBase* CapacityLinkItem::createNewItem(graph_analysis::gui::GraphWidget* graphWidget,
        const graph_analysis::Edge::Ptr& edge,
        QGraphicsItem* parent) const
{
    return new CapacityLinkItem(graphWidget, edge, parent);
}

} // end namespace edge_items
} // end namespace gui
} // end namespace templ
