#include "MissionView.hpp"
#include <templ/io/MissionReader.hpp>

// QT specific includes
#include "ui_MissionView.h"
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

#include <templ/gui/SpatioTemporalRequirementItem.hpp>

namespace templ {
namespace gui {
MissionView::MissionView(QWidget* parent)
    : QGraphicsView(parent)
    , mpGraphicsGridLayout(new QGraphicsGridLayout)
    , mpScene(new QGraphicsScene(this))
{
    mpScene->setItemIndexMethod(QGraphicsScene::NoIndex);
    setScene(mpScene);
    setAcceptDrops(true);
    setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing |
                   QPainter::SmoothPixmapTransform);

    for(int c = 0; c < 4; ++c)
    {
        for(int r = 0; r < 4; ++r)
        {
            QLabel* label = new QLabel;
            std::stringstream ss;
            ss << c << "--" << r;
            label->setText(ss.str().c_str());
            if(!scene())
            {
                throw std::runtime_error("NO SCENE");
            }

            //QGraphicsProxyWidget* pw = scene()->addWidget(label);

            SpatioTemporalRequirementItem* pw = new SpatioTemporalRequirementItem;
            mpGraphicsGridLayout->addItem(pw, c*100, r*100);
        }
    }

    QGraphicsWidget* form = new QGraphicsWidget;
    form->setLayout(mpGraphicsGridLayout);
    scene()->addItem(form);
}

MissionView::~MissionView()
{
}

void MissionView::updateVisualization()
{
}

void MissionView::on_loadMissionButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load mission description"),
        QDir::currentPath(),
        tr("Mission Description File (*.mdf *.xml)"));

    if(!filename.isEmpty())
    {
        templ::io::MissionReader reader;
    }
}

void MissionView::on_loadOntologyButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load ontology"),
        QDir::currentPath(),
        tr("Mission Description File (*.xml)"));

    if(!filename.isEmpty())
    {
        // owlapi::MissionReader reader;
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




} // end namespace gui
} // end namespace templ
