#if !defined(WIDGET_HH)
#define WIDGET_HH

#include <QtWidgets>

class Widget : public QWidget
{
public:
    explicit Widget(QWidget* pParent = nullptr);
    virtual ~Widget();

private:
    void onStartRegularRosbag();
    void onStopRegularRosbag();
    void onStartExperimentRosbag();
    void onStopExperimentRosbag();
    void onStartPlotJuggler();
    void onStopPlotJuggler();
    void onBrowseButtonClicked();

private:
    QLineEdit* mpRegularOutputDirLineEdit;
    QPushButton* mpRegularOutputDirBrowseButton;
    QSpinBox* mpSplitMinuteSpinBox;
    QLabel* mpNextLogLabel;
    QSpinBox* mpMaxSplitsSpinBox;
    QLineEdit* mpExperimentOutputDirLineEdit;
    QPushButton* mpExperimentOutputDirBrowseButton;
    QComboBox* mpTestTypeComboBox;
    QSpinBox* mpTestNoSpinBox;
    QLineEdit* mpStatusLineEdit;
};

#endif // !defined(WIDGET_HH)
