#include "uro_rviz_plugins/SonarPanel.hh"
#include <QGridLayout>
#include <QPushButton>
#include <chrono>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <thread>

using namespace uro_rviz_plugins;
using namespace std::chrono_literals;

SonarPanel::SonarPanel(QWidget* pParent) : rviz_common::Panel(pParent), mpNode(nullptr), mpClient(nullptr)
{
    QGridLayout* p_layout = new QGridLayout();
    setLayout(p_layout);
    mpGroup = new QButtonGroup(this);
    mpArm = new QPushButton("ARM", this);
    mpDisarm = new QPushButton("DISARM", this);
    mpArm->setCheckable(true);
    mpDisarm->setCheckable(true);
    mpArm->setStyleSheet("QPushButton:checked{background-color:red;}");
    mpDisarm->setStyleSheet("QPushButton:checked{background-color:#f5f5f5;}");
    mpGroup->addButton(mpArm);
    mpGroup->addButton(mpDisarm);
    mpStatus = new QLineEdit();
    mpStatus->setReadOnly(true);
    mpStatus->setFocusPolicy(Qt::ClickFocus);
    QColor col = palette().color(QPalette::Window);
    QPalette pal = mpStatus->palette();
    pal.setColor(QPalette::Base, col);
    mpStatus->setPalette(pal);
    p_layout->setContentsMargins(0, 0, 0, 0);
    p_layout->addWidget(mpArm, 0, 0);
    p_layout->addWidget(mpDisarm, 0, 1);
    p_layout->addWidget(mpStatus, 1, 0, 1, 2);
    connect(mpArm, &QPushButton::clicked, this, &SonarPanel::SendRequest);
    connect(mpDisarm, &QPushButton::clicked, this, &SonarPanel::SendRequest);
    connect(this, &SonarPanel::completed, this, &SonarPanel::OnCompleted);
}

// virtual
SonarPanel::~SonarPanel()
{
}

void
SonarPanel::SendRequest()
{
    if (mpClient == nullptr)
    {
        return;
    }

    mpRequest = std::make_shared<std_srvs::srv::SetBool::Request>();
    mpRequest->data = (sender() == mpArm);

    this->setEnabled(false);
    std::thread([&] {
        auto result = mpClient->async_send_request(mpRequest);
        int i = 0;
        while (!mpClient->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                emit completed("interrupted while waiting for service", false);
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            if (++i > 10)
            {
                emit completed("service not available", false);
                return;
            }
        }

        i = 0;
        while (rclcpp::ok())
        {
            if (result.wait_for(1s) == std::future_status::ready)
            {
                QString status;
                status.sprintf("success: %s message: %s", result.get()->success ? "true" : "false",
                               result.get()->message.c_str());
                emit completed(status, result.get()->success);
                break;
            }
            if (++i > 10)
            {
                emit completed("failed to call service", false);
                break;
            }
        }
    }).detach();
}

void
SonarPanel::onInitialize()
{
    mpNode = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    mpClient = mpNode->create_client<std_srvs::srv::SetBool>("/rov_control_node/controller_manager");
}

void
SonarPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

void
SonarPanel::load(const rviz_common::Config& config)
{
    rviz_common::Panel::load(config);
}

void
SonarPanel::OnCompleted(const QString& status, bool success)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), status.toStdString());
    if (!success)
    {
        mpArm->setCheckable(false);
        mpDisarm->setCheckable(false);
    }
    mpStatus->setText(status);
    mpStatus->setSelection(0, 0);
    mpStatus->setCursorPosition(0);
    this->setEnabled(true);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uro_rviz_plugins::SonarPanel, rviz_common::Panel)
