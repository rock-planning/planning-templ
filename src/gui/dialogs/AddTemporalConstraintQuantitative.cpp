#include "AddTemporalConstraintQuantitative.hpp"
#include <QDialog>
#include "ui_EmptyDialog.h"

namespace templ {
namespace gui {
namespace dialogs {

AddTemporalConstraintQuantitative::AddTemporalConstraintQuantitative(
        const QList<QString>& timepoints,
        QWidget* parent
        )
    : QDialog(parent)
    , mpUi(new Ui::EmptyDialog)
    , mpTemporalConstraintQuantitative(new widgets::TemporalConstraintQuantitative)
{
    mpUi->setupUi(this);
    mpTemporalConstraintQuantitative->prepare(timepoints);
    mpUi->gridLayout->addWidget(mpTemporalConstraintQuantitative);
}

AddTemporalConstraintQuantitative::~AddTemporalConstraintQuantitative()
{
    delete mpUi;
}

io::TemporalConstraint AddTemporalConstraintQuantitative::getConstraint() const
{
    return mpTemporalConstraintQuantitative->getConstraint();
}

} // end namespace dialogs
} // end namespace gui
} // end namespace templ
