#ifndef TEMPL_GUI_SOLUTION_ANALYSIS_SOLUTION_PROPERTIES_HPP
#define TEMPL_GUI_SOLUTION_ANALYSIS_SOLUTION_PROPERTIES_HPP

#include "../../solvers/SolutionAnalysis.hpp"
#include <QWidget>

namespace Ui {
    class SolutionProperties;
}

namespace templ {
namespace gui {

class SolutionProperties : public QWidget
{
    Q_OBJECT

public:
    SolutionProperties(const solvers::SolutionAnalysis& analysis, QWidget* parent = NULL);
    ~SolutionProperties();

private:
    Ui::SolutionProperties* mpUi;

    void setRow(const QString& name, double value, int row,
                const QString& description = "");

};

} // namespace gui
} // namespace templ
#endif // TEMPL_GUI_SOLUTION_ANALYSIS_SOLUTION_PROPERTIES_HPP
