#include "SolutionProperties.hpp"

#include <QString>
#include <QTableWidgetItem>
#include "ui_SolutionProperties.h"

namespace templ {
namespace gui {

SolutionProperties::SolutionProperties(const solvers::SolutionAnalysis& sa, QWidget* parent)
    : mpUi(new Ui::SolutionProperties)
{
    mpUi->setupUi(this);

    mpUi->tableWidgetProperties->setRowCount(12);
    mpUi->tableWidgetProperties->setColumnCount(3);

    int row = 0;
    setRow("alpha", sa.getAlpha(), row, "Weight for safety");
    setRow("beta", sa.getBeta(), ++row, "Weight for efficacy");
    setRow("sigma", sa.getSigma(), ++row, "Weight for efficiency");
    setRow("quality", sa.getQuality(), ++row, "Quality estimate");
    setRow("cost", sa.getCost(), ++row, "Quality estimate");
    setRow("safety", sa.getSafety(), ++row);
    setRow("efficacy", sa.getEfficacy(), ++row);
    setRow("efficiceny", sa.getEfficiency(), ++row);
    setRow("reconfiguration cost", sa.getReconfigurationCost(), ++row);
    setRow("travellded distance in m", sa.getTravelledDistance(), ++row);
    setRow("time horizon in s", sa.getTimeHorizon(), ++row);
}

SolutionProperties::~SolutionProperties()
{
    delete mpUi;
}

void SolutionProperties::setRow(const QString& name, double value, int row,
                const QString& description)
{
    QTableWidgetItem* nameItem = new QTableWidgetItem(tr("%1").arg(name));
    mpUi->tableWidgetProperties->setItem(row, 0, nameItem);

    QTableWidgetItem* valueItem = new QTableWidgetItem(tr("%1").arg(value));
    mpUi->tableWidgetProperties->setItem(row, 1, valueItem);

    QTableWidgetItem* descriptionItem = new QTableWidgetItem(tr("%1").arg(description));
    mpUi->tableWidgetProperties->setItem(row, 2, descriptionItem);
}



} // end namespace gui
} // end namespace templ
