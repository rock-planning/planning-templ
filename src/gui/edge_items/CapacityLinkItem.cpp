#include "CapacityLinkItem.hpp"
#include <graph_analysis/gui/GraphWidget.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <base-logging/Logging.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <QSettings>
#include <QtCore/qmath.h>

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

    mpClassName =
        new QGraphicsTextItem(QString("Capacity"), this);
    mpClassName->setDefaultTextColor(Qt::gray);

    mpFillBar = new QGraphicsRectItem(getEdgePath()->pos().x()-10, getEdgePath()->pos().y()+125, 10, 100, this);
    mpFillBar->setPen( QPen(Qt::black) );
    mpFillBar->setBrush(QBrush(Qt::white));

    double consumptionInPercent = 0;
    CapacityLink::Ptr capacityLink = dynamic_pointer_cast<CapacityLink>( getEdge() );
    if(capacityLink)
    {
       consumptionInPercent = capacityLink->getConsumptionLevel();
    }

    mpFillStatus = new QGraphicsRectItem(getEdgePath()->pos().x()-10, getEdgePath()->pos().y()+125, 10, consumptionInPercent*100, this);
    mpFillStatus->setPen( QPen(Qt::black) );
    mpFillStatus->setBrush(QBrush(Qt::black));

    setFlag(ItemIsMovable, false);

    mpLabel = new QGraphicsTextItem("", this);
    // Make sure label is drawn on top of any edge that is might overlap with
    getEdgePath()->stackBefore(mpLabel);
    mpFillBar->stackBefore(mpLabel);
    mpFillStatus->stackBefore(mpLabel);
}

CapacityLinkItem::~CapacityLinkItem()
{
    delete mpLabel;
    delete mpClassName;
}

int CapacityLinkItem::type() const
{
    return static_cast<int>(graph_analysis::gui::UserType) + 11;
}

void CapacityLinkItem::adjustEdgePositioning()
{
    /// Allow setting via widget
    QSettings settings("templ","gui");
    QPen pen = settings.value("editor/edge/pen").value<QPen>();

    prepareGeometryChange();

    drawBezierEdge();

    drawArrowHead(mArrowSize*qSqrt(pen.width()), pen.brush(), QPen(pen.brush().color()));

    mpLabel->setPos(mpEdgePath->boundingRect().center() -
                    mpLabel->boundingRect().center());
    mpClassName->setPos(mpLabel->pos() +
                        QPointF(0, mpLabel->boundingRect().height()));

    mpFillBar->setPos(mpLabel->pos().x(), mpLabel->pos().y() - mpFillBar->rect().height());
    mpFillStatus->setPos(mpLabel->pos().x(), mpLabel->pos().y() - mpFillStatus->rect().height());

    getEdgePath()->setPen(pen);


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

void CapacityLinkItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    mpClassName->setPlainText("");

    QString text("");
    CapacityLink::Ptr capacityLink = dynamic_pointer_cast<CapacityLink>( getEdge() );
    if(capacityLink)
    {
        int consumptionInPercent = capacityLink->getConsumptionLevel()*100;
        text += "Consumed capacity: " + QString::number(consumptionInPercent) + "%\n";
    }

    text += QString(getEdge()->toString().c_str());
    mpLabel->setHtml(QString("<div style=\"background-color:#ffffff; white-space: pre;\">")
            + text
            + QString("</div>"));
}

void CapacityLinkItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    mpClassName->setPlainText("Capacity");
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
