#ifndef TEMPL_GUI_WIDGETS_SPATIO_TEMPORAL_REQUIREMENT_HPP
#define TEMPL_GUI_WIDGETS_SPATIO_TEMPORAL_REQUIREMENT_HPP

#include "../../io/MissionRequirements.hpp"
#include "ModelCardinality.hpp"
#include "../../solvers/temporal/PersistenceCondition.hpp"
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

    void updateRequirement(const solvers::temporal::PersistenceCondition& pc);

    io::SpatioTemporalRequirement::Ptr getRequirement() const;

    /**
     * Get the key label for this spatio temporal requirement
     * constisting of the location and timeinterval
     */
    QString getKey() const;

    void setLocation(const std::string& location);
    void setTimepointFrom(const std::string& from);
    void setTimepointTo(const std::string& to);

public slots:
    ModelCardinality* addModelCardinality();
    void removeModelCardinalities();
    void updateKey();

signals:
    void keyChanged(const QString& s);


private:
    Ui::SpatioTemporalRequirement* mpUi;
    organization_model::OrganizationModelAsk mAsk;
};

} // namespace widgets
} // namespace gui
} // namespace templ

#endif // TEMPL_GUI_WIDGETS_SPATIO_TEMPORAL_REQUIREMENT_HPP
