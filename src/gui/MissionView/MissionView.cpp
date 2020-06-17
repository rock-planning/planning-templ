#include "MissionView.hpp"
#include "../../io/MissionReader.hpp"

// QT specific includes
//#include "ui_MissionView.h"
#include <QDirIterator>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <QtDebug>
#include <QLabel>
#include <QSettings>
#include <QCoreApplication>

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

bool MissionView::loadMission(const QString& settingsLabel, const QString& _filename)
{
    QSettings settings(QCoreApplication::organizationName(), settingsLabel);
    QString dir = QDir::currentPath();

    QString dirValue = settings.value("recentImportDir").toString();
    if(!dirValue.isEmpty())
    {
        dir = dirValue;
    }

    QString filename;
    qDebug() << "MissionView: loadMission with: " << _filename;

    if(_filename.isEmpty() || !QFileInfo(_filename).exists())
    {
        filename = QFileDialog::getOpenFileName(
        this, tr("Load mission description"),
        dir,
        tr("Mission Description File (*.mdf *.xml)"));
        if(filename.isEmpty())
        {
            return false;
        }
    } else {
        filename = _filename;
    }

    if(!filename.isEmpty())
    {
        // update recent files list
        QStringList files = settings.value("recentImportFileList").toStringList();
        files.removeAll(filename);
        files.prepend(filename);
        while(files.size() > 10)
        {
            files.removeLast();
        }
        settings.setValue("recentImportFileList", files);
        // end update recent files list

        mFilename = filename;
        QFileInfo fileinfo(filename);
        settings.setValue("recentImportDir", fileinfo.absolutePath());

        try {
            Mission mission = templ::io::MissionReader::fromFile(filename.toStdString());
            mpMission = make_shared<Mission>(mission);

            loadXML(filename);
            expandAll();
            show();

            refreshView();
            return true;
        } catch(const std::exception& e)
        {
            QMessageBox::warning(this, "Templ", QString("MissionView: failed to load file --") + QString(e.what()));
        }
    }
    return false;
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
