#ifndef TEMPL_GUI_WIDGETS_MODEL_CARDINALITY_HPP
#define TEMPL_GUI_WIDGETS_MODEL_CARDINALITY_HPP

#include "../../io/MissionRequirements.hpp"
#include <moreorg/OrganizationModelAsk.hpp>

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

    void prepare(const moreorg::OrganizationModelAsk& ask);

    void setRequirement(const io::ResourceRequirement& requirement);

    void setMinVisible(bool visible);
    void setMaxVisible(bool visible);

    io::ResourceRequirement getRequirement() const;

private:
    Ui::ModelCardinality* mpUi;
};

} // namespace widgets
} // namespace gui
} // namespace templ

#endif // TEMPL_GUI_WIDGETS_MODEL_CARDINALITY_HPP
