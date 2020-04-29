#ifndef TEMPL_GUI_DIALOGS_ADD_TEMPORAL_CONSTRAINTS_QUALITATIVE_HPP
#define TEMPL_GUI_DIALOGS_ADD_TEMPORAL_CONSTRAINTS_QUALITATIVE_HPP

#include "../../io/TemporalConstraint.hpp"
#include "../widgets/TemporalConstraintQualitative.hpp"

#include <QDialog>

namespace Ui
{
    class EmptyDialog;
}

namespace templ {
namespace gui {
namespace dialogs {

class AddTemporalConstraintQualitative : public QDialog
{
    Q_OBJECT

public:
    AddTemporalConstraintQualitative(const QList<QString>& timepoints = QList<QString>(),
            QWidget* parent = NULL
    );
    ~AddTemporalConstraintQualitative();

    io::TemporalConstraint getConstraint() const;

private:
    Ui::EmptyDialog* mpUi;
    widgets::TemporalConstraintQualitative* mpTemporalConstraintQualitative;
};

} // end namespace dialogs
} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_DIALOGS_ADD_TEMPORAL_CONSTRAINTS_QUALITATIVE_HPP
