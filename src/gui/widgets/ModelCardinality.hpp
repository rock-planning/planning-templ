#ifndef TEMPL_GUI_WIDGETS_MODEL_CARDINALITY_HPP
#define TEMPL_GUI_WIDGETS_MODEL_CARDINALITY_HPP

#include "../../io/MissionRequirements.hpp"
#include <organization_model/OrganizationModelAsk.hpp>

#include <QWidget>

namespace Ui
{
    class ModelCardinality;
}

namespace templ {
namespace gui {
namespace widgets {

class ModelCardinality : public QWidget
{
    Q_OBJECT

public:
    ModelCardinality(QWidget* parent = NULL);
    ~ModelCardinality();

    void prepare(const organization_model::OrganizationModelAsk& ask);

    void setRequirement(const io::ResourceRequirement& requrirement);

    io::ResourceRequirement getRequirement() const;

private:
    Ui::ModelCardinality* mpUi;
};

} // namespace widgets
} // namespace gui
} // namespace templ

#endif // TEMPL_GUI_WIDGETS_MODEL_CARDINALITY_HPP
