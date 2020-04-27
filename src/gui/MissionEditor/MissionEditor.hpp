#ifndef TEMPL_GUI_MISSION_EDITOR_HPP
#define TEMPL_GUI_MISSION_EDITOR_HPP

#include <QWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <graph_analysis/Graph.hpp>
#include "../../Mission.hpp"

namespace Ui {
    class MissionEditor;
}

namespace templ {
namespace gui {

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

private:
    // GUI Elements
    Ui::MissionEditor* mpUi;

    Mission::Ptr mpMission;
    QString mMissionFilename;

    // Adding/Removing Constraints
    void on_addConstraintButton_clicked();
    void on_removeConstraintButton_clicked();



    // (Re)load the data from the current mission
    void updateVisualization();
    void save(const QString& filename);

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

    // Loading/Storing Missions
    void on_loadMissionButton_clicked();
    void on_loadOntologyButton_clicked();

    void on_saveButton_clicked();
    void on_updateButton_clicked();
    void on_clearButton_clicked();

    void removeCheckedRows(QLayout* parent);
    void removeRow(QLayout* parent, QHBoxLayout* rowLayout);

    void addResourceCardinality();
    void removeResourceCardinalities();

    void addConstant();
    void removeConstants();

    const QString& getFilename() const { return mMissionFilename; }

};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_MISSION_EDITOR_HPP
