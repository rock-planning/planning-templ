#include "MissionView.hpp"
#include <templ/io/MissionReader.hpp>

// QT specific includes
//#include "ui_MissionView.h"
#include <QDirIterator>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <QtDebug>
#include <QLabel>

// Rock specific includes
#include <base-logging/Logging.hpp>
#include <graph_analysis/gui/RegisterQtMetatypes.hpp>
#include <sstream>
#include <QGraphicsWidget>
#include <QGraphicsProxyWidget>

#include <QMenu>
#include <templ/symbols/object_variables/LocationCardinality.hpp>
#include <templ/gui/SpatioTemporalRequirementItem.hpp>

namespace templ {
namespace gui {
MissionView::MissionView(QWidget* parent)
    : QGraphicsView(parent)
    , mpGraphicsGridLayout()
    , mpScene(new QGraphicsScene(this))
{
    mpScene->setItemIndexMethod(QGraphicsScene::NoIndex);
    setScene(mpScene);
    setAcceptDrops(true);
    setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing |
                   QPainter::SmoothPixmapTransform);

}

MissionView::~MissionView()
{
}

void MissionView::updateVisualization()
{
}

void MissionView::on_loadMissionButton_clicked()
{
    if(!mpOrganizationModel)
    {
        QMessageBox msgBox;
        msgBox.setText("Cannot load a mission without loading the organization model (ontology) first");
        msgBox.exec();
    } else {
        QString filename = QFileDialog::getOpenFileName(
            this, tr("Load mission description"),
            QDir::currentPath(),
            tr("Mission Description File (*.mdf *.xml)"));

        if(!filename.isEmpty())
        {
            Mission mission = templ::io::MissionReader::fromFile(filename.toStdString(), mpOrganizationModel);
            mpMission = Mission::Ptr(new Mission(mission));

            refreshView();
        }
    }
}

void MissionView::on_loadOrganizationModelButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load ontology"),
        QDir::currentPath(),
        tr("Ontology Description File (*.owl)"));

    if(!filename.isEmpty())
    {
        mpOrganizationModel = organization_model::OrganizationModel::getInstance(filename.toStdString());
    }
}

void MissionView::on_saveButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(
        this, tr("Store mission description"),
        QDir::currentPath(),
        tr("Component Network Definition File (*.mdf *.xml)"));

    if(!filename.isEmpty())
    {
        //templ::io::MissionWriter writer;
        // do write
    }
}

void MissionView::on_clearButton_clicked()
{
}

void MissionView::on_addConstraintButton_clicked()
{
    LOG_INFO_S << "addConstraintButton clicked";
}

void MissionView::on_removeConstraintButton_clicked()
{
    LOG_INFO_S << "removeConstraintButton clicked";
}

void MissionView::on_planMission_clicked()
{
    LOG_DEBUG_S << "planMission clicked";
}

void MissionView::on_updateButton_clicked()
{
    LOG_DEBUG_S << "updateButton clicked";
}

void MissionView::wheelEvent(QWheelEvent *event)
{
    scaleView(pow(2.0, -event->delta() / 240.0));
}

void MissionView::scaleView(qreal scaleFactor)
{
    qreal factor = transform()
                       .scale(scaleFactor, scaleFactor)
                       .mapRect(QRectF(0, 0, 1, 1))
                       .width();
    if(factor < 0.07 || factor > 100)
    {
        return;
    }
    scale(scaleFactor, scaleFactor);
    std::string status_msg = scaleFactor > 1. ? "Zoomed-in" : "Zoomed-out";
}
void MissionView::mousePressEvent(QMouseEvent* event)
{
    // enable panning by pressing+dragging the left mouse button if there is
    // _no_ item under the cursor right now.
    if ((event->button() == Qt::LeftButton) && (!itemAt(event->pos()))) {
        setDragMode(ScrollHandDrag);
        QGraphicsView::mousePressEvent(event);
        return;
    }

    QGraphicsView::mousePressEvent(event);
}

void MissionView::mouseReleaseEvent(QMouseEvent* event)
{
    // always try to reset drag mode, just to be sure
    if (dragMode() != QGraphicsView::NoDrag) {
        setDragMode(NoDrag);
    }

    QGraphicsView::mouseReleaseEvent(event);
}

void MissionView::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu contextMenu;
    QAction* loadOrganizationModel = contextMenu.addAction("Load organization model");
    QAction* loadMission = contextMenu.addAction("Load mission");

    QAction* selectedAction = contextMenu.exec(event->globalPos());
    if(selectedAction == loadOrganizationModel)
    {
        on_loadOrganizationModelButton_clicked();
    } else if(selectedAction == loadMission)
    {
        on_loadMissionButton_clicked();
    }
}

void MissionView::refreshView()
{
    if(mpMission)
    {
        scene()->clear();

        mpGraphicsGridLayout = new QGraphicsGridLayout;

        std::vector<symbols::constants::Location::Ptr> locations = mpMission->getLocations();
        std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints = mpMission->getTimepoints();

        using namespace templ::solvers::temporal;
        std::vector<PersistenceCondition::Ptr> pConditions = mpMission->getPersistenceConditions();
        std::vector<PersistenceCondition::Ptr>::const_iterator cit = pConditions.begin();

        for(; cit != pConditions.end(); ++cit)
        {
            const PersistenceCondition::Ptr& pc = *cit;

            SpatioTemporalRequirementItem* requirementItem = new SpatioTemporalRequirementItem;
            requirementItem->setPersistenceCondition(pc);
            requirementItem->updateFromPersistenceCondition();

            Symbol::Ptr symbol = pc->getValue();
            symbols::object_variables::LocationCardinality::Ptr locationCardinality = dynamic_pointer_cast<symbols::object_variables::LocationCardinality>(symbol);

            int rowIndex = 0;
            symbols::constants::Location::Ptr location = locationCardinality->getLocation();
            if(location)
            {
                rowIndex = std::distance(locations.begin(), std::find(locations.begin(), locations.end(), location));
                // Column is Time
            } else {
                LOG_WARN_S << "Could not cast to location pointer --BEGIN " << symbol->toString()
                    << " --END";
            }

            int columnIndex = std::distance( timepoints.begin(), std::find(timepoints.begin(), timepoints.end(), pc->getFromTimePoint()));

            LOG_WARN_S << "ADD ITEM AT" << columnIndex << "/" << rowIndex
                << pc->getFromTimePoint()->toString()
                << " "
                << location->toString();

            mpGraphicsGridLayout->addItem(requirementItem, rowIndex, columnIndex);
        }

        //for(int c = 0; c < 4; ++c)
        //{
        //    for(int r = 0; r < 4; ++r)
        //    {
        //        QLabel* label = new QLabel;
        //        std::stringstream ss;
        //        ss << c << "--" << r;
        //        label->setText(ss.str().c_str());
        //        //QGraphicsProxyWidget* pw = scene()->addWidget(label);

        //        SpatioTemporalRequirementItem* pw = new SpatioTemporalRequirementItem;
        //        mpGraphicsGridLayout->addItem(pw, c, r);
        //    }
        //}

        QGraphicsWidget* form = new QGraphicsWidget;
        form->setLayout(mpGraphicsGridLayout);
        scene()->addItem(form);
    }
}




} // end namespace gui
} // end namespace templ
