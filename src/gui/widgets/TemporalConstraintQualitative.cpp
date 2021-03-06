#include "TemporalConstraintQualitative.hpp"
#include "ui_TemporalConstraintQualitative.h"

namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace gui {
namespace widgets {

TemporalConstraintQualitative::TemporalConstraintQualitative(QWidget* parent)
    : QWidget(parent)
    , mpUi(new Ui::TemporalConstraintQualitative)
{
    mpUi->setupUi(this);

    mpUi->comboBoxLVal->setEditable(true);
    mpUi->comboBoxRVal->setEditable(true);
}

TemporalConstraintQualitative::~TemporalConstraintQualitative()
{
    delete mpUi;
}

void TemporalConstraintQualitative::prepare(const QList<QString>& timepoints)
{
    mpUi->comboBoxLVal->clear();
    mpUi->comboBoxRVal->clear();
    mpUi->comboBoxRelation->clear();

    for(const QString& t : timepoints)
    {
        mpUi->comboBoxLVal->addItem(t);
        mpUi->comboBoxRVal->addItem(t);
    }

    for(const auto& v: pa::QualitativeTimePointConstraint::TypeSymbol)
    {
        mpUi->comboBoxRelation->addItem( QString::fromStdString( v.second ) );
    }

}

void TemporalConstraintQualitative::setConstraint(const io::TemporalConstraint& constraint)
{
    {
        QString value = QString::fromStdString( constraint.lval);
        int index = mpUi->comboBoxLVal->findText( value );
        if(index == -1)
        {
            mpUi->comboBoxLVal->addItem(value);
            index = mpUi->comboBoxLVal->findText( value );
        }
        mpUi->comboBoxLVal->setCurrentIndex( index );
    }

    {
        QString value = QString::fromStdString( constraint.rval);
        int index = mpUi->comboBoxRVal->findText( value );
        if(index == -1)
        {
            mpUi->comboBoxRVal->addItem(value);
            index = mpUi->comboBoxRVal->findText( value );
        }
        mpUi->comboBoxRVal->setCurrentIndex( index );
    }

    {
        QString value = QString::fromStdString( pa::QualitativeTimePointConstraint::TypeSymbol[ constraint.type ] );
        int index = mpUi->comboBoxRelation->findText( value );
        if(index == -1)
        {
            mpUi->comboBoxRelation->addItem(value);
            index = mpUi->comboBoxRelation->findText( value );
        }
        mpUi->comboBoxRelation->setCurrentIndex( index );
    }
}

io::TemporalConstraint TemporalConstraintQualitative::getConstraint() const
{
    io::TemporalConstraint constraint;
    QString relation = mpUi->comboBoxRelation->currentText();
    constraint.type = pa::QualitativeTimePointConstraint::getTypeFromSymbol(relation.toStdString());
    constraint.lval = mpUi->comboBoxLVal->currentText().toStdString();
    constraint.rval = mpUi->comboBoxRVal->currentText().toStdString();
    return constraint;
}

solvers::temporal::point_algebra::QualitativeTimePointConstraint::Ptr
    TemporalConstraintQualitative::getQualitativeTemporalConstraint() const
{
    namespace pa = solvers::temporal::point_algebra;

    io::TemporalConstraint tc = getConstraint();
    pa::QualitativeTimePointConstraint::Ptr qtpc = make_shared<pa::QualitativeTimePointConstraint>();
    qtpc->setType(tc.type);

    pa::TimePoint::Ptr from = pa::TimePoint::create(tc.lval);
    if(!from)
    {
        throw std::runtime_error("templ::gui::widgets::TemporalConstraintQualitative::getQualitativeTemporalConstraint:"
                " no timepoint '" + tc.lval + "' known");
    } else {
        qtpc->setSourceVertex(from);
    }

    pa::TimePoint::Ptr to = pa::TimePoint::create(tc.rval);
    if(!to)
    {
        throw std::runtime_error("templ::gui::widgets::TemporalConstraintQualitative::getQualitativeTemporalConstraint:"
                " no timepoint '" + tc.rval + "' known");
    } else {
        qtpc->setTargetVertex(to);
    }
    return qtpc;
}

} // namespace widgets
} // namespace gui
} // namespace templ
