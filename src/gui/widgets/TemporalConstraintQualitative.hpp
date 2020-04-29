#ifndef TEMPL_GUI_WIDGETS_TEMPORAL_CONSTRAINT_QUALITATIVE_HPP
#define TEMPL_GUI_WIDGETS_TEMPORAL_CONSTRAINT_QUALITATIVE_HPP

#include "../../io/TemporalConstraint.hpp"
#include <QWidget>

namespace Ui
{
    class TemporalConstraintQualitative;
}

namespace templ {
namespace gui {
namespace widgets {

class TemporalConstraintQualitative : public QWidget
{
    Q_OBJECT

public:
    TemporalConstraintQualitative(QWidget* parent = NULL);
    ~TemporalConstraintQualitative();

    void prepare(const QList<QString>& timepoints);

    void setConstraint(const io::TemporalConstraint& constraint);
    io::TemporalConstraint getConstraint() const;

private:
    Ui::TemporalConstraintQualitative* mpUi;
};

} // namespace widgets
} // namespace gui
} // namespace templ

#endif // TEMPL_GUI_WIDGETS_TEMPORAL_CONSTRAINT_QUALITATIVE_HPP
