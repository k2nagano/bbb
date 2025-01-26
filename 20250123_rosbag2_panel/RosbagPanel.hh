#ifndef URO_RVIZ_PLUGINS_ROSBAG_PANEL_HH
#define URO_RVIZ_PLUGINS_ROSBAG_PANEL_HH

#if !defined(Q_MOC_RUN)
#include <QtWidgets>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#endif // #if !defined(Q_MOC_RUN)

namespace uro_rviz_plugins
{
    class RosbagPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        RosbagPanel(QWidget* pParent = nullptr);
        virtual ~RosbagPanel();

        virtual void onInitialize();
        virtual void load(const rviz_common::Config& config);
        virtual void save(rviz_common::Config config) const;

    signals:
        void completed(const QString& status, bool success);

    private slots:
        void SendRequest();
        void OnCompleted(const QString& status, bool success);

    protected:
        rclcpp::Node::SharedPtr mpNode;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mpClientStartRegularRosbag;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mpClientStopRegularRosbag;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mpClientStartExperimentRosbag;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mpClientStopExperimentRosbag;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mpClientSetPlotJuggler;

    private:
        std::shared_ptr<std_srvs::srv::Trigger::Request> mpRequestStartRegularRosbag;
        std::shared_ptr<std_srvs::srv::Trigger::Request> mpRequestStopRegularRosbag;
        std::shared_ptr<std_srvs::srv::Trigger::Request> mpRequestStartExperimentRosbag;
        std::shared_ptr<std_srvs::srv::Trigger::Request> mpRequestStopExperimentRosbag;
        std::shared_ptr<std_srvs::srv::SetBool::Request> mpRequestSetPlotJuggler;

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

} // namespace uro_rviz_plugins

#endif // URO_RVIZ_PLUGINS_ROSBAG_PANEL_HH
