#include "RoleInfoItem.hpp"
#include <graph_analysis/gui/GraphWidget.hpp>
#include <QGraphicsSceneMouseEvent>
#include <QDrag>

#include <graph_analysis/gui/EdgeMimeData.hpp>
#include <graph_analysis/gui/BaseGraphView/AddEdgeDialog.hpp>
#include <graph_analysis/EdgeTypeManager.hpp>
#include "../../SpaceTime.hpp"

#include <base-logging/Logging.hpp>
#include <QSvgRenderer>
#include <QTableWidget>
#include <QHeaderView>
#include <QDebug>
#include <QScrollBar>
#include <QResource>

using namespace graph_analysis;
using namespace graph_analysis::gui;

namespace templ {
namespace gui {
namespace vertex_items {

RoleInfoItemDelegate::RoleInfoItemDelegate(QObject* parent)
    : QItemDelegate(parent)
{}


void RoleInfoItemDelegate::paint(QPainter* painter,
        const QStyleOptionViewItem& option,
        const QModelIndex& index) const
{
    QItemDelegate::paint(painter, option, index);
    QPen pen(Qt::black);
    pen.setWidth(3.0);
    painter->setPen(pen);
    painter->drawRect(option.rect);
}

RoleInfoItem::RoleInfoItem()
    : VertexItemBase()
    , mpLabel(0)
    , mpClassName(0)
    , mpRect(0)
    , mpEllipse(0)
    , mpEllipseText(0)
    , mBorderPen(QPen(Qt::black))
{
    mBorderPen.setWidth(3);
}

/**
 *
 * kiss: very simple base implementation of a rectangle
 *
 */
RoleInfoItem::RoleInfoItem(graph_analysis::gui::GraphWidget* graphWidget,
                            const graph_analysis::Vertex::Ptr& vertex,
                            QGraphicsItem* parent)
    : VertexItemBase(graphWidget, vertex, parent)
    , mpLabel(0)
    , mpClassName(0)
    , mpRect(0)
    , mpEllipse(0)
    , mpEllipseText(0)
    , mBorderPen(QPen(Qt::black))
{

    mBorderPen.setWidth(3);

    // at the lowest (so in the background) the rectangle but use it only to
    // generate the size, so set color alpha value to 0
    mpRect = new QGraphicsRectItem(this);
    mpRect->setPen(QPen(QColor(255,255,255,0)));

    SpaceTime::Network::tuple_t::Ptr tuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(vertex);
    assert(tuple);

    mpLabel = new QGraphicsTextItem(QString(""), this);
    //QFont font = mpLabel->font();
    //font.setBold(true);
    //mpLabel->setFont(font);

    qreal fontSize = 25.0;
    int yOffset = 25;
    int xLabelOffset = 30;
    mpLocationSvg = new QGraphicsSvgItem(":/resources/pictograms/location.svg", this);
    mpLocationSvg->setScale(fontSize/ mpLocationSvg->renderer()->defaultSize().height());
    mpLocationSvg->setPos(-50,-50);
    mpLocationLabel = new QGraphicsTextItem(QString(tuple->first()->getInstanceName().c_str()), this);
    mpLocationLabel->setPos(mpLocationSvg->pos() + QPointF(xLabelOffset,-5));

    mpTimepointSvg = new QGraphicsSvgItem(":/resources/pictograms/timepoint.svg", this);
    mpTimepointSvg->setScale(fontSize/ mpTimepointSvg->renderer()->defaultSize().height());
    mpTimepointSvg->setPos(mpLocationSvg->pos() + QPoint(0,yOffset + 3));
    mpTimepointLabel = new QGraphicsTextItem(QString(tuple->second()->getLabel().c_str()), this);
    mpTimepointLabel->setPos(mpTimepointSvg->pos() + QPointF(xLabelOffset,-5));

    QFont font = mpTimepointLabel->font();
    font.setPointSize(15);
    font.setBold(true);
    mpLocationLabel->setFont(font);
    mpTimepointLabel->setFont(font);

    int yPos = 40;
    yOffset = 40;
    fontSize = 25.0;
    QPointF prevItemPos = mpTimepointSvg->pos();
    QPointF prevItemLabelPos = mpTimepointLabel->pos();

    font.setPointSize(15);
    if(tuple->hasAttribute(RoleInfo::SAFETY))
    {
        double safety = tuple->getAttribute(RoleInfo::SAFETY);
        mpSafetySvg = new QGraphicsSvgItem(":/resources/pictograms/safety.svg", this);
        mpSafetySvg->setScale(fontSize/ mpSafetySvg->renderer()->defaultSize().height());
        mpSafetySvg->setPos(prevItemPos + QPoint(5, yOffset+3));
        mpSafetyLabel = new QGraphicsTextItem(QString::number(safety,'f',2), this);
        mpSafetyLabel->setPos(prevItemLabelPos + QPointF(0,yOffset+3));
        yPos += 20;

        mpSafetyLabel->setFont(font);

        prevItemPos = mpSafetySvg->pos();
        prevItemLabelPos = mpSafetyLabel->pos();
        yOffset = fontSize;
    }

    if(tuple->hasAttribute(RoleInfo::RECONFIGURATION_COST))
    {
        double reconfigurationCost = tuple->getAttribute(RoleInfo::RECONFIGURATION_COST);
        mpReconfigurationSvg = new QGraphicsSvgItem(":/resources/pictograms/reconfiguration.svg", this);
        mpReconfigurationSvg->setScale(fontSize/mpReconfigurationSvg->renderer()->defaultSize().height());
        mpReconfigurationSvg->setPos(prevItemPos + QPoint(-10, yOffset + 3));
        mpReconfigurationLabel = new QGraphicsTextItem(QString::number(reconfigurationCost), this);
        mpReconfigurationLabel->setPos(prevItemLabelPos + QPointF(0,yOffset+3));
        yPos += 20;

        mpReconfigurationLabel->setFont(font);

        prevItemPos = mpReconfigurationSvg->pos();
        prevItemLabelPos = mpReconfigurationLabel->pos();
        yOffset = fontSize;
    }

    Role::TypeMap typeMap = Role::toTypeMap( tuple->getAllRoles());
    for(const Role::TypeMap::value_type& v : typeMap)
    {
        int numberOfRows = 4;
        int numberOfColumns = 4;
        int sizeOfRow = 20;
        int sizeOfColumn = 20;
        addModelTable(v.first, 0, yPos,
                numberOfColumns, numberOfRows,
                sizeOfRow, sizeOfColumn);
        yPos += numberOfRows*sizeOfRow + 5;
    }

    // now that all the children are there, we use their bounding-rect to
    // enlarge the background-rect. note that we never modify the boundingRect
    // afterwards.
    int margin = mBorderPen.width() + 2;
    QRectF bRect = childrenBoundingRect();
    QRectF rect(bRect.x() - margin,
            bRect.y() - margin,
            bRect.width() + 2*margin,
            bRect.height() + 2*margin);
    mpRect->setRect(rect);
    //mpRect->setRect(childrenBoundingRect());

    // for this "Simple" type we want to have it movable. this graphical
    // "object" will not be contained inside other items, so thats ok
    setFlag(ItemIsMovable);

    // for thi specific "Simple" implmentation, we accept drops -- and allow
    // drops -- to create new edges. see later.
    setAcceptDrops(true);

    std::stringstream ss;
    ss << vertex->toString();
    Role::List missing = tuple->getRelativeComplement(RoleInfo::TagTxt[ RoleInfo::REQUIRED ], RoleInfo::TagTxt[ RoleInfo::ASSIGNED ]);
    if(!missing.empty())
    {
        ss << "Missing roles:" << std::endl;
        for(const Role& r : missing)
        {
            ss << "    " << r.toString() << std::endl;
        }
        mBorderPen.setColor(Qt::red);
        mBorderPen.setStyle( Qt::DashLine);
    }
    Role::List superfluous = tuple->getRelativeComplement(RoleInfo::TagTxt[ RoleInfo::ASSIGNED ], RoleInfo::TagTxt[ RoleInfo::REQUIRED ]);
    if(!superfluous.empty())
    {
        ss << "Superfluous roles:" << std::endl;
        for(const Role& r : superfluous)
        {
            ss << "    " << r.toString() << std::endl;
        }
    }

    setToolTip(QString( ss.str().c_str() ));
}

RoleInfoItem::~RoleInfoItem()
{
    delete mpLabel;
    delete mpClassName;
    delete mpRect;
}


void RoleInfoItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget)
{
    // we want a rectangle with round edges
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setRenderHint(QPainter::HighQualityAntialiasing, true);
    QPainterPath path;
    path.addRoundedRect(childrenBoundingRect(),15,15);
    painter->setPen(mBorderPen);
    painter->drawPath(path);
}

// drag'n drop!
//
// this function is called as long as it is not "accepted" (by calling the
// relevant function.
void RoleInfoItem::dragEnterEvent(QGraphicsSceneDragDropEvent* event)
{
    // by default, this is not accepted.
    event->setAccepted(false);
    // checking that the vertex stored in the mimetype is different from ours
    // -- this prevents self-drag'n drops
    const EdgeMimeData* pMimeData =
        dynamic_cast<const EdgeMimeData*>(event->mimeData());
    if(pMimeData)
    {
        if(!pMimeData->sourceVertexIsSameAs(getVertex()))
        {
            // after this, the mouse icon will change and the drag-object will
            // be active
            event->setAccepted(true);
            mpRect->setBrush(Qt::green);
        }
        else
        {
            // TODO: would be nice to give visual feedback on the "non
            // acceptance" -- change coloring to red?
            return;
        }
    }
    else
    {
        LOG_ERROR_S << "unexpected pointer type for mimedata?";
    }
}

void RoleInfoItem::dragLeaveEvent(QGraphicsSceneDragDropEvent* event)
{
    mpRect->setBrush(Qt::NoBrush);
}

void RoleInfoItem::dropEvent(QGraphicsSceneDragDropEvent* event)
{
    // the other side's vertex pointer is stored inside the mimedata. we can
    // check that we did not get dropped on ourselfes for example.
    const EdgeMimeData* pMimeData =
        dynamic_cast<const EdgeMimeData*>(event->mimeData());
    if(pMimeData)
    {
        if(!pMimeData->sourceVertexIsSameAs(getVertex()))
        {
            // we are ok with the drop. now store this Items vertex in the
            // reference given in the mimedata, so that the originating side
            // can actuall ycreate and insert the new Egde.
            pMimeData->mpTargetVertex = getVertex();
            event->acceptProposedAction();
            mpRect->setBrush(Qt::NoBrush);
        }
        else
        {
            LOG_INFO_S << "source and target are the same?";
            return;
        }
    }
    else
    {
        LOG_ERROR_S << "unexpected pointer type for mimedata?";
    }
}

void RoleInfoItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event)
{
    Vertex::Ptr sourceVertex = getVertex();
    Vertex::Ptr targetVertex = sourceVertex;

    QDrag* drag = new QDrag(getGraphWidget());
    // stores reference to the two vertices, so that the receiving side
    // can do error-checking and store its vertex as target on success.
    EdgeMimeData* mimeData = new EdgeMimeData(sourceVertex, targetVertex);

    drag->setMimeData(mimeData);

    // when this returns, the user finished its drag-operation
    Qt::DropAction dropAction = drag->exec();

    if(dropAction == Qt::MoveAction)
    {
        // check that the targetVertex got updated
        if(!targetVertex)
        {
            LOG_ERROR_S
            << "could not find a target vertex after dropEvent";
            return;
        }
        AddEdgeDialog dialog;
        dialog.exec();
        if(dialog.result() == QDialog::Accepted)
        {
            Edge::Ptr edge = EdgeTypeManager::getInstance()->createEdge(
                dialog.getClassname().toStdString(), sourceVertex, targetVertex,
                dialog.getLabel().toStdString());
            getGraph()->addEdge(edge);
        }
    }
}

QString RoleInfoItem::getScenePosAsString() const {
    return QString("(%1, %2)").arg(scenePos().x()).arg(scenePos().y());
}

QVariant RoleInfoItem::itemChange(GraphicsItemChange change,
                                    const QVariant& value)
{
    switch(change)
    {
    case ItemScenePositionHasChanged:
    {
        // and also move the whole label a bit to stay aligned with the rect
        break;
    }
    default:
    {
        break;
    }
    };
    return VertexItemBase::itemChange(change, value);
}

void RoleInfoItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    //mpLabel->setPlainText(getVertex()->toString().c_str());
}

void RoleInfoItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
}

QGraphicsSvgItem* RoleInfoItem::getSvgItemForModel(const owlapi::model::IRI& iri, size_t height, size_t width)
{
    QString file = ":/resources/pictograms/pictogram-unknown.svg";
    if(!iri.empty())
    {
        QString agentmodel( iri.getFragment().c_str());
        QString filename = ":/resources/pictograms/pictogram-"+ agentmodel.toLower() +".svg";
        QResource resource(filename);
        if(resource.isValid())
        {
            file = filename;
        }
    }

    QGraphicsSvgItem* modelSvg = new QGraphicsSvgItem(file, this);
    int svgActualHeight = modelSvg->renderer()->defaultSize().height();
    int svgActualWidth = modelSvg->renderer()->defaultSize().width();

    double heightScale = height*1.0/svgActualHeight;
    double weightScale = width*1.0/svgActualWidth;
    double scale = qMin(heightScale, weightScale);

    modelSvg->setScale(scale);
    modelSvg->setToolTip(iri.toString().c_str());
    return modelSvg;
}

QGraphicsProxyWidget* RoleInfoItem::addModelTable(const owlapi::model::IRI& model,
        int xPos, int yPos,
        size_t columnCount,
        size_t rowCount,
        size_t defaultColumnSize,
        size_t defaultRowSize
        )
{
    // Make sure model svg is as high as the table
    QGraphicsSvgItem* modelSvg = getSvgItemForModel(model, rowCount*defaultRowSize, 75 );

    size_t tableWidth = columnCount*defaultColumnSize + 3;
    size_t tableHeight = rowCount*defaultRowSize + 3;

    // row x cols
    QTableWidget* tableWidget = new QTableWidget(rowCount, columnCount);
    tableWidget->setItemDelegate(new RoleInfoItemDelegate());
    tableWidget->setShowGrid(false);
    QString style;
    style += "QTableWidget { background-color: transparent; border-style: solid; border: 3px solid black; border-radius:4px;}";

    tableWidget->setStyleSheet(style);
    tableWidget->setGeometry(QRect(0,0,tableWidth, tableHeight));
    tableWidget->setMaximumWidth(tableWidth);
    tableWidget->setMaximumHeight(tableHeight);

    //tableWidget->resizeColumnsToContents();
    tableWidget->verticalHeader()->setVisible(false);
    tableWidget->verticalHeader()->setMinimumSectionSize(1);
    tableWidget->verticalHeader()->setDefaultSectionSize(defaultRowSize);
    tableWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    tableWidget->horizontalHeader()->setVisible(false);
    tableWidget->horizontalHeader()->setMinimumSectionSize(1);
    tableWidget->horizontalHeader()->setDefaultSectionSize(defaultColumnSize);
    tableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    QGraphicsProxyWidget* tableProxy = new QGraphicsProxyWidget(this);
    tableProxy->setWidget(tableWidget);
    tableProxy->setGeometry(QRect(0,0,tableWidth, tableHeight));

    SpaceTime::Network::tuple_t::Ptr tuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(getVertex());

    for(size_t r = 0; r < rowCount; ++r)
    {
        for(size_t c = 0; c < columnCount; ++c)
        {
            uint32_t id = c*rowCount + r;
            RoleInfo::Status status = tuple->getStatus(model, id);
            QTableWidgetItem* item = new QTableWidgetItem();
            item->setFlags(item->flags() & ~Qt::ItemIsEditable & ~Qt::ItemIsSelectable);
            tableWidget->setItem(r,c,item);

            if(status == RoleInfo::UNKNOWN_STATUS)
            {
                tableWidget->item(r,c)->setToolTip(QString("empty"));
            } else {
                tableWidget->item(r,c)->setToolTip(QString(model.getFragment().c_str()) + "_" + QString::number(id));
            }


            switch(status)
            {
                case RoleInfo::NOTREQUIRED_AVAILABLE:
                    tableWidget->item(r,c)->setBackground(Qt::black);
                    break;
                case RoleInfo::NOTREQUIRED_ASSIGNED:
                    tableWidget->item(r,c)->setBackground(Qt::lightGray);
                    break;
                case RoleInfo::REQUIRED_ASSIGNED:
                    tableWidget->item(r,c)->setBackground(Qt::green);
                    break;
                case RoleInfo::REQUIRED_UNASSIGNED:
                    tableWidget->item(r,c)->setBackground(Qt::red);
                    break;
                default:
                    tableWidget->item(r,c)->setBackground(Qt::white);
                    break;
            }
        }
    }

    // Use sceneBoundingRect here, to get size after(!) the svg has been scaled
    modelSvg->setPos(xPos - modelSvg->sceneBoundingRect().width() - 5, yPos);
    tableProxy->setPos(xPos, yPos);
    return tableProxy;
}

VertexItemBase* RoleInfoItem::createNewItem(graph_analysis::gui::GraphWidget* graphWidget,
           const graph_analysis::Vertex::Ptr& vertex,
            QGraphicsItem* parent) const
{
    return new RoleInfoItem(graphWidget, vertex, parent);
}

} // end namespace vertex_items
} // end namespace gui
} // end namespace templ
