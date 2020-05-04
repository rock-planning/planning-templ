#ifndef TEMPL_GUI_WIDGETS_TEMPORAL_CONSTRAINT_QUANTITATIVE_HPP
#define TEMPL_GUI_WIDGETS_TEMPORAL_CONSTRAINT_QUANTITATIVE_HPP

#include "../../io/TemporalConstraint.hpp"
#include "../../solvers/temporal/IntervalConstraint.hpp"
#include <QWidget>

namespace Ui
{
    class TemporalConstraintQuantitative;
}

namespace templ {
namespace gui {
namespace widgets {

class TemporalConstraintQuantitative : public QWidget
{
    Q_OBJECT

public:
    TemporalConstraintQuantitative(QWidget* parent = NULL);
    ~TemporalConstraintQuantitative();

    void prepare(const QList<QString>& timepoints);

    void setConstraint(const io::TemporalConstraint& constraint);
    io::TemporalConstraint getConstraint() const;

    solvers::temporal::IntervalConstraint::Ptr getIntervalConstraint() const;

private:
    Ui::TemporalConstraintQuantitative* mpUi;
};

} // namespace widgets
} // namespace gui
} // namespace templ

#endif // TEMPL_GUI_WIDGETS_TEMPORAL_CONSTRAINT_QUANTITATIVE_HPP
