#include "TemporalConstraintQuantitative.hpp"
#include "ui_TemporalConstraintQuantitative.h"

namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace gui {
namespace widgets {

TemporalConstraintQuantitative::TemporalConstraintQuantitative(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::TemporalConstraintQuantitative)
{
    mpUi->setupUi(this);
    mpUi->comboBoxFrom->setEditable(true);
    mpUi->comboBoxTo->setEditable(true);
}

TemporalConstraintQuantitative::~TemporalConstraintQuantitative()
{
    delete mpUi;
}

void TemporalConstraintQuantitative::prepare(const QList<QString>& timepoints)
{
    mpUi->comboBoxFrom->clear();
    mpUi->comboBoxTo->clear();

    for(const QString& t : timepoints)
    {
        mpUi->comboBoxFrom->addItem(t);
        mpUi->comboBoxTo->addItem(t);
    }
}

void TemporalConstraintQuantitative::setConstraint(const io::TemporalConstraint& constraint)
{
    {
        QString value = QString::fromStdString( constraint.lval);
        int index = mpUi->comboBoxFrom->findText( value );
        if(index == -1)
        {
            mpUi->comboBoxFrom->addItem(value);
            index = mpUi->comboBoxFrom->findText( value );
        }
        mpUi->comboBoxFrom->setCurrentIndex( index );
    }

    {
        QString value = QString::fromStdString( constraint.rval);
        int index = mpUi->comboBoxTo->findText( value );
        if(index == -1)
        {
            mpUi->comboBoxTo->addItem(value);
            index = mpUi->comboBoxTo->findText( value );
        }
        mpUi->comboBoxTo->setCurrentIndex( index );
    }

    {
        mpUi->spinBoxMin->setValue( constraint.minDuration );
        mpUi->spinBoxMax->setValue( constraint.maxDuration );
    }
}

io::TemporalConstraint TemporalConstraintQuantitative::getConstraint() const
{
    io::TemporalConstraint constraint;
    constraint.lval = mpUi->comboBoxFrom->currentText().toStdString();
    constraint.rval = mpUi->comboBoxTo->currentText().toStdString();
    constraint.minDuration = mpUi->spinBoxMin->value();
    constraint.maxDuration = mpUi->spinBoxMax->value();
    return constraint;
}

solvers::temporal::IntervalConstraint::Ptr TemporalConstraintQuantitative::getIntervalConstraint() const
{
    io::TemporalConstraint tc = getConstraint();
    pa::TimePoint::Ptr from = pa::TimePoint::get(tc.lval);
    if(!from)
    {
        throw std::runtime_error("templ::gui::widgets::TemporalConstraintQuantitative::getQuantitativeTemporalConstraint:"
                " no timepoint '" + tc.lval + "' known");
    }

    pa::TimePoint::Ptr to = pa::TimePoint::get(tc.rval);
    if(!to)
    {
        throw std::runtime_error("templ::gui::widgets::TemporalConstraintQuantitative::getQuantitativeTemporalConstraint:"
                " no timepoint '" + tc.rval + "' known");
    }

    solvers::temporal::IntervalConstraint::Ptr ic = make_shared<solvers::temporal::IntervalConstraint>(from,to);

    solvers::temporal::Bounds bounds(tc.minDuration, tc.maxDuration);
    ic->addInterval(bounds);
    return ic;
}

} // namespace widgets
} // namespace gui
} // namespace templ
