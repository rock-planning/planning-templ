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

#include <QMenu>
#include <QMessageBox>
//#include <QDom

namespace templ {
namespace gui {
MissionView::MissionView(QWidget* parent)
    : QTreeView(parent)
{
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
        Mission mission = templ::io::MissionReader::fromFile(filename.toStdString());
        mpMission = Mission::Ptr(new Mission(mission));

        loadXML(filename);
        expandAll();
        show();

        refreshView();
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

void MissionView::refreshView()
{
    if(mpMission)
    {
    }
}

void MissionView::loadXML(const QString& filename)
{
    QDomDocument doc("mission");

    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(this,"Mission loading failed", "Failed to load the mission from file " + filename);
        return;
    }

    if(!doc.setContent(&file))
    {
        QMessageBox::information(this,"Mission loading failed", "Failed to load the xml as DOM for mission from file " + filename);
        return;
    }

    file.close();

    QStandardItemModel* model = new QStandardItemModel();
    //QStandardItem* item = new QStandardItem("test0");
    //model->setItem(0, item);

    //QStandardItem* item1 = new QStandardItem("test1");
    //model->setItem(1, item1);

    preOrder(doc, model);

    setModel(model);
}

QStandardItem* MissionView::createRoot(const QString& name)
{
    QStandardItem* item = new QStandardItem(name);
    item->setEditable(false);
    return item;
}

QStandardItem* MissionView::createChild(QStandardItem* item, const QDomNode& node)
{
    QString textLabel = node.nodeName();
    if(node.isText())
    {
        textLabel = node.nodeValue();
    }

    QStandardItem* childItem = new QStandardItem(textLabel);
    childItem->setEditable(false);
    item->appendRow(childItem);

    for(int i = 0; i < node.attributes().size(); ++i)
    {
        QDomNode attribute = node.attributes().item(i);
        QString attributeTxt(attribute.nodeName() + "='" + attribute.nodeValue() + "'");
        QStandardItem* attrItem = new QStandardItem(attributeTxt);
        attrItem->setEditable(false);
        childItem->appendRow(attrItem);
    }
    return childItem;
}


void MissionView::preOrder(QDomNode parentNode, QStandardItemModel* model, QStandardItem* item)
{
    for(int i = 0;  i < parentNode.childNodes().size(); ++i)
    {
        const QDomNode& node = parentNode.childNodes().at(i);
        if(!node.isNull())
        {
            QStandardItem* newItem = item;
            if(!node.isComment())
            {
                if(item)
                {
                    // new child to append
                    newItem = createChild(item,node);
                } else {
                    // new parent (root) node
                    newItem = createRoot(node.nodeName());
                    model->setItem(0, newItem);
                }
            }
            preOrder(node, model, newItem);
        }
    }
}

} // end namespace gui
} // end namespace templ
