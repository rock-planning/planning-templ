#ifndef TEMPL_GUI_WIDGETS_SPATIO_TEMPORAL_REQUIREMENT_HPP
#define TEMPL_GUI_WIDGETS_SPATIO_TEMPORAL_REQUIREMENT_HPP

#include "../../io/MissionRequirements.hpp"
#include "ModelCardinality.hpp"
#include <organization_model/OrganizationModelAsk.hpp>

#include <QWidget>

namespace Ui
{
    class SpatioTemporalRequirement;
}

namespace templ {
namespace gui {
namespace widgets {

class SpatioTemporalRequirement : public QWidget
{
    Q_OBJECT

public:
    SpatioTemporalRequirement(
            const organization_model::OrganizationModelAsk& ask,
            QWidget* parent = NULL);

    ~SpatioTemporalRequirement();

    void prepareLocations(const QList<QString>& locations);
    void prepareTimepoints(const QList<QString>& timepoints);

    void setRequirement(const io::SpatioTemporalRequirement::Ptr& requirement);

    io::SpatioTemporalRequirement::Ptr getRequirement() const;

public slots:
    ModelCardinality* addModelCardinality();
    void removeModelCardinalities();

private:
    Ui::SpatioTemporalRequirement* mpUi;
    organization_model::OrganizationModelAsk mAsk;
};

} // namespace widgets
} // namespace gui
} // namespace templ

#endif // TEMPL_GUI_WIDGETS_SPATIO_TEMPORAL_REQUIREMENT_HPP
