#include "Widget.hh"

namespace
{
    void
    setButtonWidth(QPushButton* button, int width)
    {
        button->setMinimumWidth(width);
        button->setMaximumWidth(width);
    }
} // namespace

// explicit
Widget::Widget(QWidget* pParent) : QWidget(pParent)
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
            &Widget::onBrowseButtonClicked);
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
    connect(startRosbagButton1, &QPushButton::clicked, this, &Widget::onStartRegularRosbag);
    connect(stopRosbagButton1, &QPushButton::clicked, this, &Widget::onStopRegularRosbag);
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
            &Widget::onBrowseButtonClicked);
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
    connect(startRosbagButton2, &QPushButton::clicked, this, &Widget::onStartExperimentRosbag);
    connect(stopRosbagButton2, &QPushButton::clicked, this, &Widget::onStopExperimentRosbag);
    connect(startPlotButton, &QPushButton::clicked, this, &Widget::onStartPlotJuggler);
    connect(stopPlotButton, &QPushButton::clicked, this, &Widget::onStopPlotJuggler);
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
}

// virtual
Widget::~Widget()
{
}

// コールバック関数の定義
void
Widget::onStartRegularRosbag()
{ /* 処理を記述 */
}
void
Widget::onStopRegularRosbag()
{ /* 処理を記述 */
}
void
Widget::onStartExperimentRosbag()
{ /* 処理を記述 */
}
void
Widget::onStopExperimentRosbag()
{ /* 処理を記述 */
}
void
Widget::onStartPlotJuggler()
{ /* 処理を記述 */
}
void
Widget::onStopPlotJuggler()
{ /* 処理を記述 */
}
void
Widget::onBrowseButtonClicked()
{
    // QFileDialog をローカル変数として作成
    QFileDialog dialog(this, "Select Directory");
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
