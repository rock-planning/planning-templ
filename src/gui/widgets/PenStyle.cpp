#include "PenStyle.hpp"
#include "ui_PenStyle.h"
#include <QColorDialog>
#include <iostream>

namespace templ {
namespace gui {
namespace widgets {

PenStyle::PenStyle(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::PenStyle())
    , mSettings("templ","gui")
{
    mpUi->setupUi(this);
    QVariant variant = mSettings.value("editor/edge/pen");

    // style
    mpUi->styleComboBox->addItem(/*icon, */"Solid Line", (int)Qt::SolidLine);
    mpUi->styleComboBox->addItem(/*icon, */"Dash Line", (int)Qt::DashLine);
    mpUi->styleComboBox->addItem(/*icon, */"Dot Line", (int)Qt::DotLine);

    // pen join style
    mpUi->joinStyleComboBox->addItem(/*icon, */"Bevel", Qt::BevelJoin);
    mpUi->joinStyleComboBox->addItem(/*icon, */"Miter", Qt::MiterJoin);
    mpUi->joinStyleComboBox->addItem(/*icon, */"Round", Qt::RoundJoin);

    // pen cap style
    mpUi->capStyleComboBox->addItem(/*icon, */"Square Cap", Qt::SquareCap);
    mpUi->capStyleComboBox->addItem(/*icon, */"Flat Cap", Qt::FlatCap);
    mpUi->capStyleComboBox->addItem(/*icon, */"Round Cap", Qt::RoundCap);

    // First time init with defaults
    if(variant == QVariant())
    {
        mpUi->colorPushButton->setStyleSheet("background-color: black");
    } else {
        QPen pen = variant.value<QPen>();
        mpUi->colorPushButton->setStyleSheet("background-color: " + pen.brush().color().name());
        mpUi->styleComboBox->setCurrentIndex( mpUi->styleComboBox->findData( (int)pen.style() ) );
        mpUi->joinStyleComboBox->setCurrentIndex( mpUi->joinStyleComboBox->findData( pen.joinStyle() ) );
        mpUi->capStyleComboBox->setCurrentIndex( mpUi->capStyleComboBox->findData( pen.capStyle() ) );
        mpUi->widthSpinBox->setValue(pen.width());
    }

    connect(mpUi->colorPushButton, SIGNAL(clicked()), this, SLOT(selectColor()));
    connect(mpUi->styleComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStyle()));
    connect(mpUi->widthSpinBox, SIGNAL(valueChanged(int)), this, SLOT(updateStyle()));
}

PenStyle::~PenStyle()
{
    delete mpUi;
}

void PenStyle::selectColor()
{
    QPen pen = mSettings.value("editor/edge/pen").value<QPen>();
    QColor color = QColorDialog::getColor(pen.color(), this);
    pen.setColor(color);
    mSettings.setValue("editor/edge/pen", pen);
    mpUi->colorPushButton->setStyleSheet("background-color: " + color.name());
}

void PenStyle::updateStyle()
{
    QPen pen = mSettings.value("editor/edge/pen").value<QPen>();

    pen.setWidth( mpUi->widthSpinBox->value());

    pen.setStyle( static_cast<Qt::PenStyle>(mpUi->styleComboBox->itemData( mpUi->styleComboBox->currentIndex()).toInt()) );
    pen.setJoinStyle( static_cast<Qt::PenJoinStyle>(mpUi->joinStyleComboBox->itemData( mpUi->joinStyleComboBox->currentIndex()).toInt()));
    pen.setCapStyle( static_cast<Qt::PenCapStyle>(mpUi->capStyleComboBox->itemData( mpUi->capStyleComboBox->currentIndex() ).toInt() ) );

    mSettings.setValue("editor/edge/pen", pen);
}




} // end namespace widgets
} // end namespace gui
} // end namespace templ
