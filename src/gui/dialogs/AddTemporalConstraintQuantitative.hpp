#ifndef TEMPL_GUI_DIALOGS_ADD_TEMPORAL_CONSTRAINTS_QUANTITATIVE_HPP
#define TEMPL_GUI_DIALOGS_ADD_TEMPORAL_CONSTRAINTS_QUANTITATIVE_HPP

#include "../../io/TemporalConstraint.hpp"
#include "../widgets/TemporalConstraintQuantitative.hpp"

#include <QDialog>

namespace Ui
{
    class EmptyDialog;
}

namespace templ {
namespace gui {
namespace dialogs {

class AddTemporalConstraintQuantitative : public QDialog
{
    Q_OBJECT

public:
    AddTemporalConstraintQuantitative(const QList<QString>& timepoints = QList<QString>(),
            QWidget* parent = NULL
    );
    ~AddTemporalConstraintQuantitative();

    io::TemporalConstraint getConstraint() const;

private:
    Ui::EmptyDialog* mpUi;
    widgets::TemporalConstraintQuantitative* mpTemporalConstraintQuantitative;
};

} // end namespace dialogs
} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_DIALOGS_ADD_TEMPORAL_CONSTRAINTS_QUANTITATIVE_HPP
