#ifndef TEMPL_GUI_MISSION_EDITOR_HPP
#define TEMPL_GUI_MISSION_EDITOR_HPP

#include <QWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <graph_analysis/Graph.hpp>
#include "../../Mission.hpp"
#include "../../constraints/ModelConstraint.hpp"
#include "../../io/TemporalConstraint.hpp"

namespace Ui {
    class MissionEditor;
}

namespace templ {
namespace gui {

namespace widgets {
    class ModelCardinality;
    class ModelConstraint;
    class Location;
    class TemporalConstraintQualitative;
    class TemporalConstraintQuantitative;
    class SpatioTemporalRequirement;
}

class MissionEditor : public QWidget
{
    Q_OBJECT

public:
    MissionEditor(QWidget* parent = NULL);
    ~MissionEditor();

    QString getClassName() const
    {
        return "templ::gui::MissionEditor";
    }

    Mission::Ptr getMission() const { return mpMission; }

    symbols::constants::Location::PtrList getLocations() const;
    solvers::temporal::point_algebra::TimePoint::PtrList getTimepoints() const;

private:
    // GUI Elements
    Ui::MissionEditor* mpUi;

    Mission::Ptr mpMission;
    QString mMissionFilename;

    // (Re)load the data from the current mission
    void updateVisualization();
    void save(const QString& filename);

    void activateOrganizationModelDependants();
    void deactivateOrganizationModelDependants();

    Mission::Ptr currentMission() const;

public slots:
    void on_planMission_clicked();
    /**
     *  Loading/Storing Missions
     *  \return True, when mission was loaded, false otherwise
     */
    bool loadMission(const QString& settingsLabel ="IOMission",
            const QString& filename = "");

    bool loadOrganizationModel(const QString& settingsLabel = "IOMission",
            const QString& filenameOrIri = "");

    void on_updateMissionName(const QString& name);
    void on_updateMissionDescription(const QString& description);

    // Loading/Storing Missions
    void on_loadMissionButton_clicked();
    void on_loadOntologyButton_clicked();

    void on_saveButton_clicked();
    void on_updateButton_clicked();
    void on_clearButton_clicked();

    widgets::ModelCardinality* addResourceCardinality();
    void removeResourceCardinalities();

    void addOverride();
    void removeOverrides();

    void addConstant();
    void removeConstants();
    widgets::Location* addLocation(const symbols::constants::Location::Ptr& location);

    widgets::SpatioTemporalRequirement* addRequirement();
    void removeRequirements();
    void requirementsUpdated();
    QList<QString> getRequirementsKeys() const;

    void addConstraint();
    void removeConstraints();
    widgets::ModelConstraint* addModelConstraint(const constraints::ModelConstraint::Ptr& constraint);
    widgets::TemporalConstraintQualitative* addTemporalConstraintQualitative(const io::TemporalConstraint& constraint);
    widgets::TemporalConstraintQuantitative* addTemporalConstraintQuantitative(const io::TemporalConstraint& constraint);

    void clear();

    const QString& getFilename() const { return mMissionFilename; }
};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_MISSION_EDITOR_HPP
