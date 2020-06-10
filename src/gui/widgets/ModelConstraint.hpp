#ifndef TEMPL_GUI_WIDGETS_MODEL_CONSTRAINT_HPP
#define TEMPL_GUI_WIDGETS_MODEL_CONSTRAINT_HPP

#include "../../constraints/ModelConstraint.hpp"
#include <moreorg/OrganizationModelAsk.hpp>

#include <QWidget>
#include <QComboBox>

namespace Ui
{
    class ModelConstraint;
}

namespace templ {
namespace gui {
namespace widgets {

class ModelConstraint : public QWidget
{
    Q_OBJECT

public:
    ModelConstraint(QWidget* parent = NULL);
    ~ModelConstraint();

    void prepare(const moreorg::OrganizationModelAsk& ask);
    void setValue(const constraints::ModelConstraint::Ptr& modelConstraint);

    void setButtonVisibility(bool visible);

    constraints::ModelConstraint::Ptr getConstraint() const;

    std::vector<SpaceTime::SpaceIntervalTuple> getIntervals() const;

public slots:
    void typeChanged(const QString& type);
    void modelChanged(const QString& type);

    QComboBox* addRequirement();
    void removeRequirements();
    void updateRequirements(const QList<QString>& requirements);

private:
    Ui::ModelConstraint* mpUi;
    QList<QString> mRequirements;
};

} // namespace widgets
} // namespace gui
} // namespace templ

#endif // TEMPL_GUI_WIDGETS_MODEL_CONSTRAINT_HPP
