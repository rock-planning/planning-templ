#ifndef TEMPL_GUI_WIDGETS_PEN_STYLE_HPP
#define TEMPL_GUI_WIDGETS_PEN_STYLE_HPP

#include <QWidget>
#include <QPen>
#include <QSettings>

namespace Ui {
    class PenStyle;
}

namespace templ {
namespace gui {
namespace widgets {

class PenStyle : public QWidget
{
    Q_OBJECT

public:
        PenStyle(QWidget* parent = NULL);
        ~PenStyle();

        QString getClassName() const
        {
            return "templ::gui::widgets::PenStyle";
        }
public slots:
    void selectColor();
    void updateStyle();

private:
        Ui::PenStyle* mpUi;

        QPen mEdgePen;
        QPen mVertexPen;

        QSettings mSettings;
};

} // end namespace widgets
} // end namespace gui
} // end namespace templ

#endif // TEMPL_GUI_WIDGETS_PEN_STYLE_HPP
