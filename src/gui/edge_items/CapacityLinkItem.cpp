#include "CapacityLinkItem.hpp"
#include <graph_analysis/gui/GraphWidget.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <base-logging/Logging.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <QSettings>
#include <QFileDialog>
#include <QCoreApplication>
#include <QtCore/qmath.h>
#include <QDebug>

#include <organization_model/OrganizationModel.hpp>
#include <organization_model/OrganizationModelAsk.hpp>
#include <organization_model/ModelPool.hpp>

#include "../TemplGui.hpp"
#include "../../SpaceTime.hpp"


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
    , mMarkedAsInvalid(false)
{

    QPen pen(Qt::black);
    pen.setWidth(3.0);

    mpFillBar = new QGraphicsRectItem(getEdgePath()->pos().x()-10, getEdgePath()->pos().y()+125, 10, 100, this);
    mpFillBar->setPen( pen );
    mpFillBar->setBrush(QBrush(Qt::white));

    double consumptionInPercent = 0;
    CapacityLink::Ptr capacityLink = dynamic_pointer_cast<CapacityLink>( getEdge() );
    if(capacityLink)
    {
       consumptionInPercent = capacityLink->getConsumptionLevel();
    }
    RoleInfoWeightedEdge::Ptr weightedEdge = dynamic_pointer_cast<RoleInfoWeightedEdge>(getEdge());
    if(weightedEdge)
    {
        capacityLink = toCapacityLink(weightedEdge);
        consumptionInPercent = capacityLink->getConsumptionLevel();

        std::set<RoleInfo::Tag> tags = { RoleInfo::INFEASIBLE };
        Role::Set transitionRoles = weightedEdge->getRoles(tags);
        if(!transitionRoles.empty())
        {
            mMarkedAsInvalid = true;
        }
    }

    if(capacityLink)
    {
        mpFillStatus = new QGraphicsRectItem(getEdgePath()->pos().x()-10, getEdgePath()->pos().y()+125, 10, consumptionInPercent*100, this);
        mpFillStatus->setPen(pen);
        mpFillStatus->setBrush(QBrush(Qt::black));

        setFlag(ItemIsMovable, false);

        mpLabel = new QGraphicsTextItem("", this);
        // Make sure label is drawn on top of any edge that is might overlap with
        getEdgePath()->stackBefore(mpLabel);
        mpFillBar->stackBefore(mpLabel);
        mpFillStatus->stackBefore(mpLabel);
    }
}

CapacityLinkItem::~CapacityLinkItem()
{
    delete mpLabel;
}

int CapacityLinkItem::type() const
{
    return static_cast<int>(graph_analysis::gui::UserType) + 11;
}

void CapacityLinkItem::adjustEdgePositioning()
{
    /// Allow setting via widget
    QSettings settings(QCoreApplication::organizationName(),"GUI");
    QVariant v = settings.value("editor/edge/pen/default");
    QPen pen;
    if(v == QVariant())
    {
        pen.setColor(Qt::black);
        pen.setWidth(3.0);
        pen.setStyle(Qt::SolidLine);
    } else {
        pen = v.value<QPen>();
    }

    if(mMarkedAsInvalid)
    {
        QVariant v = settings.value("editor/edge/pen/error");
        if(v == QVariant())
        {
            pen.setColor(Qt::red);
            pen.setWidth(3.0);
            pen.setStyle(Qt::DashLine);
        } else {
            pen = v.value<QPen>();
        }
    }

    prepareGeometryChange();
    drawBezierEdge();

    drawArrowHead(mArrowSize*qSqrt(pen.width()), pen.brush(), QPen(pen.brush().color()));

    mpLabel->setPos(mpEdgePath->boundingRect().center() -
                    mpLabel->boundingRect().center());

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
    mpLabel->setPlainText("");
}

EdgeItemBase* CapacityLinkItem::createNewItem(graph_analysis::gui::GraphWidget* graphWidget,
        const graph_analysis::Edge::Ptr& edge,
        QGraphicsItem* parent) const
{
    return new CapacityLinkItem(graphWidget, edge, parent);
}

CapacityLink::Ptr CapacityLinkItem::toCapacityLink(const RoleInfoWeightedEdge::Ptr& roleInfo)
{
    CapacityLink::Ptr capacityLink = make_shared<CapacityLink>();
    SpaceTime::Network::tuple_t::Ptr from = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(roleInfo->getSourceVertex());
    SpaceTime::Network::tuple_t::Ptr to = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(roleInfo->getTargetVertex());
    if(!from || !to)
    {
        return capacityLink;
    }

    std::set<RoleInfo::Tag> tags = { RoleInfo::ASSIGNED, RoleInfo::AVAILABLE };
    Role::Set fromRoles = from->getRoles(tags);
    Role::Set toRoles = to->getRoles(tags);
    Role::List transitionRoles = RoleInfo::getIntersection(fromRoles, toRoles);

    if(from->first() == to->first())
    {
        capacityLink->addProvider( CapacityLink::getLocalTransitionRole(), std::numeric_limits<uint32_t>::max() );
    }

    organization_model::ModelPool pool;
    for(const Role& r : transitionRoles)
    {
        pool[r.getModel()] = 1;
    }

    organization_model::OrganizationModel::Ptr om = TemplGui::getOrganizationModel();
    organization_model::OrganizationModelAsk ask(om, pool);

    for(const Role& r : transitionRoles)
    {
        using namespace organization_model::facades;
        Robot robot = Robot::getInstance(r.getModel(), ask);
        if(robot.isMobile())
        {
            capacityLink->addProvider(r, robot.getTransportCapacity());
        } else {
            capacityLink->addConsumer(r, robot.getTransportDemand());
        }
    }

    return capacityLink;
}

} // end namespace edge_items
} // end namespace gui
} // end namespace templ
