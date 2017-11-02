#include "RoleInfoItem.hpp"
#include <graph_analysis/gui/GraphWidget.hpp>
#include <base-logging/Logging.hpp>
#include <cmath>

using namespace graph_analysis::gui;

namespace templ {
namespace gui {
namespace edge_items {

// kiss:
RoleInfoItem::RoleInfoItem(graph_analysis::gui::GraphWidget* graphWidget,
                               const graph_analysis::Edge::Ptr& edge,
                               QGraphicsItem* parent)
    : EdgeItemBase(graphWidget, edge, parent)
    , mArrowSize(10)
{
    mpLabel = new QGraphicsTextItem(QString(edge->toString().c_str()), this);
    mpClassName =
        new QGraphicsTextItem(QString(edge->getClassName().c_str()), this);
    mpClassName->setDefaultTextColor(Qt::gray);

    setFlag(ItemIsMovable, false);

    //mpGraphWidget->registerEdgeItem(mpEdge, this);
}

RoleInfoItem::~RoleInfoItem()
{
    delete mpLabel;
    delete mpClassName;
    //mpGraphWidget->deregisterEdgeItem(mpEdge, this);
}

int RoleInfoItem::type() const
{
    return static_cast<int>(graph_analysis::gui::UserType) + 10;
}

void RoleInfoItem::adjustEdgePositioning()
{
    prepareGeometryChange();

    drawBezierEdge();
    drawArrowHead(mArrowSize);

    mpLabel->setPos(mpEdgePath->boundingRect().center() -
                    mpLabel->boundingRect().center());
    mpClassName->setPos(mpLabel->pos() +
                        QPointF(0, mpLabel->boundingRect().height()));
}

void RoleInfoItem::paint(QPainter* painter,
                           const QStyleOptionGraphicsItem* option,
                           QWidget* widget)
{
}

QRectF RoleInfoItem::boundingRect() const
{
    return childrenBoundingRect();
}

EdgeItemBase* RoleInfoItem::createNewItem(graph_analysis::gui::GraphWidget* graphWidget,
        const graph_analysis::Edge::Ptr& edge,
        QGraphicsItem* parent) const
{
    return new RoleInfoItem(graphWidget, edge, parent);
}

} // end namespace edge_items
} // end namespace gui
} // end namespace templ
