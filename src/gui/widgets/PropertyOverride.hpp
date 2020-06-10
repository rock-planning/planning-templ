#ifndef TEMPL_GUI_WIDGETS_PROPERTY_OVERRIDE_HPP
#define TEMPL_GUI_WIDGETS_PROPERTY_OVERRIDE_HPP

#include "../../DataPropertyAssignment.hpp"
#include <moreorg/OrganizationModelAsk.hpp>

#include <QWidget>

namespace Ui
{
    class PropertyOverride;
}

namespace templ {
namespace gui {
namespace widgets {

class PropertyOverride : public QWidget
{
    Q_OBJECT

public:
    PropertyOverride(
        const moreorg::OrganizationModelAsk& ask,
        QWidget* parent = NULL);
    ~PropertyOverride();

    void clear();
    void prepare();

    void setOrganizationModelAsk(const moreorg::OrganizationModelAsk& ask);


    DataPropertyAssignment getDataPropertyAssignment() const;

private slots:
    /**
     * Update the properties combobox, once the subject changes
     */
    void updateProperties();

private:
    Ui::PropertyOverride* mpUi;
    moreorg::OrganizationModelAsk mAsk;
};

} // end namespace widgets
} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_WIDGETS_PROPERTY_OVERRIDE_HPP
