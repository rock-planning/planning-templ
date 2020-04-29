#include "AddTemporalConstraintQualitative.hpp"
#include <QDialog>
#include "ui_EmptyDialog.h"

namespace templ {
namespace gui {
namespace dialogs {

AddTemporalConstraintQualitative::AddTemporalConstraintQualitative(
        const QList<QString>& timepoints,
        QWidget* parent
        )
    : QDialog(parent)
    , mpUi(new Ui::EmptyDialog)
    , mpTemporalConstraintQualitative(new widgets::TemporalConstraintQualitative)
{
    mpUi->setupUi(this);
    mpTemporalConstraintQualitative->prepare(timepoints);
    mpUi->gridLayout->addWidget(mpTemporalConstraintQualitative);
}

AddTemporalConstraintQualitative::~AddTemporalConstraintQualitative()
{
    delete mpUi;
}

io::TemporalConstraint AddTemporalConstraintQualitative::getConstraint() const
{
    return mpTemporalConstraintQualitative->getConstraint();
}

} // end namespace dialogs
} // end namespace gui
} // end namespace templ
