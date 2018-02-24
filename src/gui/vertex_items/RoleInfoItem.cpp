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

RoleInfoItem::RoleInfoItem()
    : VertexItemBase()
    , mpLabel(0)
    , mpClassName(0)
    , mpInfoBox(0)
    , mpRect(0)
    , mpEllipse(0)
    , mpEllipseText(0)
{}

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
    , mpInfoBox(0)
    , mpRect(0)
    , mpEllipse(0)
    , mpEllipseText(0)
{

    // at the lowest (so in the background) the rectangle
    mpRect = new QGraphicsRectItem(this);
    mpRect->setPen(QPen(Qt::blue));

    SpaceTime::Network::tuple_t::Ptr tuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(vertex);
    assert(tuple);

    mpLabel = new QGraphicsTextItem(QString(""), this);
    QFont font = mpLabel->font();
    font.setBold(true);
    mpLabel->setFont(font);

    mpLocationSvg = new QGraphicsSvgItem(":/resources/pictograms/location.svg", this);
    mpLocationSvg->setScale(25.0/ mpLocationSvg->renderer()->defaultSize().height());
    mpLocationSvg->setPos(0,-50);
    mpLocationLabel = new QGraphicsTextItem(QString(tuple->first()->getInstanceName().c_str()), this);
    mpLocationLabel->setPos(mpLocationSvg->pos() + QPointF(25,0));


    mpTimepointSvg = new QGraphicsSvgItem(":/resources/pictograms/timepoint.svg", this);
    mpTimepointSvg->setScale(25.0/ mpTimepointSvg->renderer()->defaultSize().height());
    mpTimepointSvg->setPos(mpLocationSvg->pos() + QPoint(0,25 + 3));
    mpTimepointLabel = new QGraphicsTextItem(QString(tuple->second()->getLabel().c_str()), this);
    mpTimepointLabel->setPos(mpTimepointSvg->pos() + QPointF(25,0));

    mpClassName =
        new QGraphicsTextItem(QString(vertex->getClassName().c_str()), this);
    mpClassName->setPos(mpLabel->pos() +
                        QPoint(0, mpLabel->boundingRect().height()));
    mpClassName->setDefaultTextColor(Qt::gray);

    mpInfoBox = new QGraphicsTextItem("", this);
    mpInfoBox->setDefaultTextColor(Qt::darkGreen);

    // change the position of the "coordinate" label to be pinned in the
    // top-right corner of the blue rect
    mpInfoBox->setPos(mpRect->rect().topRight()-
                         QPointF(mpInfoBox->boundingRect().width(), 0));


    // Add extra visualization
    mpEllipse = new QGraphicsEllipseItem(-45,-45,40,40, this);
    mpEllipse->setPos(mpRect->rect().bottomRight());
    mpEllipse->setPen(QPen(Qt::gray));
    //mpEllipse->setBrush(QBrush(Qt::darkGray, Qt::NoBrush) );

    mpEllipseText = new QGraphicsTextItem("0.95", this);
    mpEllipseText->setPos(mpEllipse->pos() + (mpEllipse->boundingRect().center() - mpEllipseText->boundingRect().center()) );
    mpEllipseText->setPlainText("0.95");
    mpEllipseText->setDefaultTextColor(Qt::gray);


    int yPos = 50;
    Role::TypeMap typeMap = Role::toTypeMap( tuple->getAllRoles());
    for(const Role::TypeMap::value_type& v : typeMap)
    {
        addModelTable(v.first, 0, yPos,
                5, 5,
                10, 10);
        yPos += 5*10 + 3;
    }

    // now that all the children are there, we use their bounding-rect to
    // enlarge the background-rect. note that we never modify the boundingRect
    // afterwards.
    mpRect->setRect(childrenBoundingRect());

    // for this "Simple" type we want to have it movable. this graphical
    // "object" will not be contained inside other items, so thats ok
    setFlag(ItemIsMovable);

    // for thi specific "Simple" implmentation, we accept drops -- and allow
    // drops -- to create new edges. see later.
    setAcceptDrops(true);

    QPen pen = mpRect->pen();
    std::stringstream ss;
    Role::List missing = tuple->getRelativeComplement(RoleInfo::TagTxt[ RoleInfo::REQUIRED ], RoleInfo::TagTxt[ RoleInfo::ASSIGNED ]);
    if(!missing.empty())
    {
        ss << "Missing roles:" << std::endl;
        for(const Role& r : missing)
        {
            ss << "    " << r.toString() << std::endl;
        }
        pen.setColor(Qt::red);
        mpRect->setPen(pen);
    }
    Role::List superfluous = tuple->getRelativeComplement(RoleInfo::TagTxt[ RoleInfo::ASSIGNED ], RoleInfo::TagTxt[ RoleInfo::REQUIRED ]);
    if(!superfluous.empty())
    {
        ss << "Superfluous roles:" << std::endl;
        for(const Role& r : superfluous)
        {
            ss << "    " << r.toString() << std::endl;
        }

        //pen.setStyle(Qt::DashLine);
        //mpRect->setPen(pen);
        mpInfoBox->setPlainText("+");
        font.setPointSize(25);
        mpInfoBox->setFont(font);
    }

    setToolTip(QString( ss.str().c_str() ));
}

RoleInfoItem::~RoleInfoItem()
{
    delete mpLabel;
    delete mpClassName;
    delete mpInfoBox;
    delete mpRect;
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
        mpInfoBox->setPos(mpRect->rect().topRight()-
                             QPointF(mpInfoBox->boundingRect().height(), 0));
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

QGraphicsSvgItem* RoleInfoItem::getSvgItemForModel(const owlapi::model::IRI& iri, size_t height)
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
    modelSvg->setScale(height*1.0/svgActualHeight);
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
    QGraphicsSvgItem* modelSvg = getSvgItemForModel(model, rowCount*defaultRowSize );

    size_t tableWidth = columnCount*defaultColumnSize + 3;
    size_t tableHeight = rowCount*defaultRowSize + 3;

    // row x cols
    QTableWidget* tableWidget = new QTableWidget(rowCount, columnCount);
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

    // Use sceneBoundingRect here, to get size after(!) the svg has been scaled
    modelSvg->setPos(xPos - modelSvg->sceneBoundingRect().width() - 3, yPos);
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
