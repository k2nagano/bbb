#ifndef URO_RVIZ_PLUGINS_SONAR_PANEL_HH
#define URO_RVIZ_PLUGINS_SONAR_PANEL_HH

#if !defined(Q_MOC_RUN)
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>
#endif // #if !defined(Q_MOC_RUN)

namespace uro_rviz_plugins
{
    class SonarPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        SonarPanel(QWidget* pParent = nullptr);
        virtual ~SonarPanel();

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
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mpClient;

    private:
        std::shared_ptr<std_srvs::srv::SetBool::Request> mpRequest;

    private:
        QButtonGroup* mpGroup;
        QPushButton* mpArm;
        QPushButton* mpDisarm;
        QLineEdit *mpStatus;
    };

} // namespace uro_rviz_plugins

#endif // URO_RVIZ_PLUGINS_SONAR_PANEL_HH
