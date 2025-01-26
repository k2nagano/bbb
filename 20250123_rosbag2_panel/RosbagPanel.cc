#include "uro_rviz_plugins/RosbagPanel.hh"
#include <QGridLayout>
#include <QPushButton>
#include <chrono>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <thread>

using namespace uro_rviz_plugins;
using namespace std::chrono_literals;

namespace
{
    void
    setButtonWidth(QPushButton* button, int width)
    {
        button->setMinimumWidth(width);
        button->setMaximumWidth(width);
    }
} // namespace

RosbagPanel::RosbagPanel(QWidget* pParent)
    : rviz_common::Panel(pParent), mpNode(nullptr), mpClientStartRegularRosbag(nullptr),
      mpClientStopRegularRosbag(nullptr), mpClientStartExperimentRosbag(nullptr),
      mpClientStopExperimentRosbag(nullptr), mpClientSetPlotJuggler(nullptr),
      mpRequestStartRegularRosbag(nullptr), mpRequestStopRegularRosbag(nullptr),
      mpRequestStartExperimentRosbag(nullptr), mpRequestStopExperimentRosbag(nullptr),
      mpRequestSetPlotJuggler(nullptr)

{
    // メインレイアウト
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    // RegularRosbag Frame
    QLabel* regularLabel = new QLabel("RegularRosbag", this);
    QFrame* regularFrame = new QFrame(this);
    regularFrame->setFrameShape(QFrame::StyledPanel);
    QVBoxLayout* regularLayout = new QVBoxLayout(regularFrame);

    // OutputDirectory
    QHBoxLayout* outputDirLayout1 = new QHBoxLayout();
    QLabel* outputDirLabel1 = new QLabel("保存先:", this);
    mpRegularOutputDirLineEdit = new QLineEdit(this);
    mpRegularOutputDirBrowseButton = new QPushButton("...", this);
    setButtonWidth(mpRegularOutputDirBrowseButton, 30); // ボタン幅を狭く設定
    connect(mpRegularOutputDirBrowseButton, &QPushButton::clicked, this,
            &RosbagPanel::onBrowseButtonClicked);
    outputDirLayout1->addWidget(outputDirLabel1, 10);
    outputDirLayout1->addWidget(mpRegularOutputDirLineEdit, 100);
    outputDirLayout1->addWidget(mpRegularOutputDirBrowseButton, 1);

    // Split Minute
    QHBoxLayout* splitMinuteLayout = new QHBoxLayout();
    QLabel* splitMinuteLabel = new QLabel("Split Minute:", this);
    mpSplitMinuteSpinBox = new QSpinBox(this);
    mpSplitMinuteSpinBox->setRange(1, 120);
    mpSplitMinuteSpinBox->setValue(10);
    splitMinuteLayout->addWidget(splitMinuteLabel, 1);
    splitMinuteLayout->addWidget(mpSplitMinuteSpinBox, 10);

    mpNextLogLabel = new QLabel("Next Log: 10 min 00 sec", this);

    // Max Splits
    QHBoxLayout* maxSplitsLayout = new QHBoxLayout();
    QLabel* maxSplitsLabel = new QLabel("Max Splits:", this);
    mpMaxSplitsSpinBox = new QSpinBox(this);
    mpMaxSplitsSpinBox->setRange(1, 100);
    mpMaxSplitsSpinBox->setValue(10);
    maxSplitsLayout->addWidget(maxSplitsLabel, 1);
    maxSplitsLayout->addWidget(mpMaxSplitsSpinBox, 10);

    // Start/Stop Rosbag Buttons
    QHBoxLayout* rosbagButtonsLayout1 = new QHBoxLayout();
    QPushButton* startRosbagButton1 = new QPushButton("Start Rosbag", this);
    QPushButton* stopRosbagButton1 = new QPushButton("Stop Rosbag", this);
    connect(startRosbagButton1, &QPushButton::clicked, this, &RosbagPanel::onStartRegularRosbag);
    connect(stopRosbagButton1, &QPushButton::clicked, this, &RosbagPanel::onStopRegularRosbag);
    rosbagButtonsLayout1->addWidget(startRosbagButton1);
    rosbagButtonsLayout1->addWidget(stopRosbagButton1);

    // Add to RegularRosbag Layout
    regularLayout->addLayout(outputDirLayout1);
    regularLayout->addLayout(splitMinuteLayout);
    regularLayout->addWidget(mpNextLogLabel);
    regularLayout->addLayout(maxSplitsLayout);
    regularLayout->addLayout(rosbagButtonsLayout1);

    // Add RegularRosbag to Main Layout
    mainLayout->addWidget(regularLabel);
    mainLayout->addWidget(regularFrame);

    // ExperimentRosbag Frame
    QLabel* experimentLabel = new QLabel("ExperimentRosbag", this);
    QFrame* experimentFrame = new QFrame(this);
    experimentFrame->setFrameShape(QFrame::StyledPanel);
    QVBoxLayout* experimentLayout = new QVBoxLayout(experimentFrame);

    // OutputDirectory
    QHBoxLayout* outputDirLayout2 = new QHBoxLayout();
    QLabel* outputDirLabel2 = new QLabel("保存先:", this);
    mpExperimentOutputDirLineEdit = new QLineEdit(this);
    mpExperimentOutputDirBrowseButton = new QPushButton("...", this);
    setButtonWidth(mpExperimentOutputDirBrowseButton, 30); // ボタン幅を狭く設定
    connect(mpExperimentOutputDirBrowseButton, &QPushButton::clicked, this,
            &RosbagPanel::onBrowseButtonClicked);
    outputDirLayout2->addWidget(outputDirLabel2, 10);
    outputDirLayout2->addWidget(mpExperimentOutputDirLineEdit, 100);
    outputDirLayout2->addWidget(mpExperimentOutputDirBrowseButton, 1);

    // Test Type
    QHBoxLayout* testTypeLayout = new QHBoxLayout();
    QLabel* testTypeLabel = new QLabel("Test Type:", this);
    mpTestTypeComboBox = new QComboBox(this);
    mpTestTypeComboBox->setEditable(true);
    mpTestTypeComboBox->addItems({"", "Type A", "Type B", "Type C"}); // config.yamlから読み込む想定
    testTypeLayout->addWidget(testTypeLabel, 1);
    testTypeLayout->addWidget(mpTestTypeComboBox, 10);

    // Test No
    QHBoxLayout* testNoLayout = new QHBoxLayout();
    QLabel* testNoLabel = new QLabel("Test No.:", this);
    mpTestNoSpinBox = new QSpinBox(this);
    mpTestNoSpinBox->setRange(1, 100);
    testNoLayout->addWidget(testNoLabel, 1);
    testNoLayout->addWidget(mpTestNoSpinBox, 10);

    // Buttons
    QHBoxLayout* rosbagButtonsLayout2 = new QHBoxLayout();
    QPushButton* startRosbagButton2 = new QPushButton("Start Rosbag", this);
    QPushButton* stopRosbagButton2 = new QPushButton("Stop Rosbag", this);
    QPushButton* startPlotButton = new QPushButton("Start Plot Juggler", this);
    QPushButton* stopPlotButton = new QPushButton("Stop Plot Juggler", this);
    connect(startRosbagButton2, &QPushButton::clicked, this, &RosbagPanel::onStartExperimentRosbag);
    connect(stopRosbagButton2, &QPushButton::clicked, this, &RosbagPanel::onStopExperimentRosbag);
    connect(startPlotButton, &QPushButton::clicked, this, &RosbagPanel::onStartPlotJuggler);
    connect(stopPlotButton, &QPushButton::clicked, this, &RosbagPanel::onStopPlotJuggler);
    rosbagButtonsLayout2->addWidget(startRosbagButton2);
    rosbagButtonsLayout2->addWidget(stopRosbagButton2);
    rosbagButtonsLayout2->addWidget(startPlotButton);
    rosbagButtonsLayout2->addWidget(stopPlotButton);

    // Add to ExperimentRosbag Layout
    experimentLayout->addLayout(outputDirLayout2);
    experimentLayout->addLayout(testTypeLayout);
    experimentLayout->addLayout(testNoLayout);
    experimentLayout->addLayout(rosbagButtonsLayout2);

    // Add ExperimentRosbag to Main Layout
    mainLayout->addWidget(experimentLabel);
    mainLayout->addWidget(experimentFrame);

    // Status Message LineEdit
    mpStatusLineEdit = new QLineEdit(this);
    mpStatusLineEdit->setReadOnly(true);
    mpStatusLineEdit->setStyleSheet("background: lightgray;");
    mainLayout->addWidget(mpStatusLineEdit);

    connect(this, &RosbagPanel::completed, this, &RosbagPanel::OnCompleted);
}

// virtual
RosbagPanel::~RosbagPanel()
{
}

// コールバック関数の定義
void
RosbagPanel::onStartRegularRosbag()
{ /* 処理を記述 */
}
void
RosbagPanel::onStopRegularRosbag()
{ /* 処理を記述 */
}
void
RosbagPanel::onStartExperimentRosbag()
{ /* 処理を記述 */
}
void
RosbagPanel::onStopExperimentRosbag()
{ /* 処理を記述 */
}
void
RosbagPanel::onStartPlotJuggler()
{ /* 処理を記述 */
}
void
RosbagPanel::onStopPlotJuggler()
{ /* 処理を記述 */
}
void
RosbagPanel::onBrowseButtonClicked()
{
    // QMetaObject::invokeMethod(this, [this]() {
    //     QFileDialog dialog(this, "Select Directory");
    //     dialog.setFileMode(QFileDialog::Directory);
    //     dialog.setOption(QFileDialog::ShowDirsOnly, true);

    //     if (dialog.exec() == QDialog::Accepted) {
    //         QString dir = dialog.selectedFiles().first();
    //         if (!dir.isEmpty()) {
    //             if (sender() == mpRegularOutputDirBrowseButton) {
    //                 mpRegularOutputDirLineEdit->setText(dir);
    //             } else if (sender() == mpExperimentOutputDirBrowseButton) {
    //                 mpExperimentOutputDirLineEdit->setText(dir);
    //             }
    //         }
    //     }
    // }, Qt::QueuedConnection);



    // QTimer::singleShot(0, this, [this]() {
    //     auto dialog = new QFileDialog(this, "Select Directory");
    //     dialog->setFileMode(QFileDialog::Directory);       // ディレクトリ選択モード
    //     dialog->setOption(QFileDialog::ShowDirsOnly, true); // ディレクトリのみ表示

    //     // 非同期でファイル選択を処理
    //     connect(dialog, &QFileDialog::fileSelected, this, [this, dialog](const QString& dir) {
    //         if (!dir.isEmpty())
    //         {
    //             if (sender() == mpRegularOutputDirBrowseButton)
    //             {
    //                 mpRegularOutputDirLineEdit->setText(dir);
    //             }
    //             else if (sender() == mpExperimentOutputDirBrowseButton)
    //             {
    //                 mpExperimentOutputDirLineEdit->setText(dir);
    //             }
    //         }
    //         dialog->deleteLater(); // ダイアログを解放
    //     });

    //     dialog->setAttribute(Qt::WA_DeleteOnClose); // 閉じた際に自動解放
    //     dialog->show(); // 非同期で表示
    // });




    // QFileDialog* dialog = new QFileDialog(this, "Select Directory");
    // dialog->setFileMode(QFileDialog::Directory); // ディレクトリ選択モードに設定
    // dialog->setOption(QFileDialog::ShowDirsOnly, true); // ディレクトリのみ表示

    // connect(dialog, &QFileDialog::fileSelected, this, [this](const QString& dir) {
    //     if (!dir.isEmpty())
    //     {
    //         if (sender() == mpRegularOutputDirBrowseButton)
    //         {
    //             mpRegularOutputDirLineEdit->setText(dir);
    //         }
    //         else
    //         {
    //             mpExperimentOutputDirLineEdit->setText(dir);
    //         }
    //     }
    // });

    // dialog->show();

    // QFileDialog をローカル変数として作成
    // QFileDialog dialog(this, "Select Directory");
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::Directory);        // ディレクトリ選択モード
    dialog.setOption(QFileDialog::ShowDirsOnly, true); // ディレクトリのみ表示

    // ダイアログをモーダルで表示し、選択結果を取得
    if (dialog.exec() == QDialog::Accepted)
    {
        QString dir = dialog.selectedFiles().first(); // 選択されたディレクトリ
        if (!dir.isEmpty())
        {
            if (sender() == mpRegularOutputDirBrowseButton)
            {
                mpRegularOutputDirLineEdit->setText(dir);
            }
            else if (sender() == mpExperimentOutputDirBrowseButton)
            {
                mpExperimentOutputDirLineEdit->setText(dir);
            }
        }
    }
}

void
RosbagPanel::SendRequest()
{
    if (mpClientSetPlotJuggler == nullptr)
    {
        return;
    }
    if (mpRequestSetPlotJuggler == nullptr)
    {
        mpRequestSetPlotJuggler = std::make_shared<std_srvs::srv::SetBool::Request>();
    }
    mpRequestSetPlotJuggler->data = true;
    this->setEnabled(false);
    std::thread([&] {
        auto result = mpClientSetPlotJuggler->async_send_request(mpRequestSetPlotJuggler);
        int i = 0;
        while (!mpClientSetPlotJuggler->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                             "Interrupted while waiting for the service. Exiting.");
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
RosbagPanel::onInitialize()
{
    mpNode = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    mpClientSetPlotJuggler = mpNode->create_client<std_srvs::srv::SetBool>("/set_plot_juggler");
}

void
RosbagPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

void
RosbagPanel::load(const rviz_common::Config& config)
{
    rviz_common::Panel::load(config);
}

void
RosbagPanel::OnCompleted(const QString& status, bool success)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), status.toStdString());
    if (!success)
    {
    }
    mpStatusLineEdit->setText(status);
    mpStatusLineEdit->setSelection(0, 0);
    mpStatusLineEdit->setCursorPosition(0);
    this->setEnabled(true);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uro_rviz_plugins::RosbagPanel, rviz_common::Panel)
