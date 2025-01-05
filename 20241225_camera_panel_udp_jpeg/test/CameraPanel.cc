#include "CameraPanel.hh"

#include <QBuffer>
#include <QByteArray>
#include <QDebug>
#include <QHostAddress>
#include <QImageReader>
#include <QMap>

#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// explicit
SlideButton::SlideButton(QWidget* parent) : QWidget(parent), isOn(false), buttonPos(0)
{
    setFixedSize(100, 40); // ウィジェットのサイズを設定
}

// virtual
SlideButton::~SlideButton()
{
}

// override
void
SlideButton::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing |
                           QPainter::HighQualityAntialiasing);

    // スイッチ背景の描画
    if (isOn)
    {
        painter.setBrush(QColor(180, 50, 50)); // ON状態の色
    }
    else
    {
        painter.setBrush(QColor(50, 180, 50)); // OFF状態の色
    }
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(0, 0, width(), height(), 20, 20);

    // ボタン部分の描画
    painter.setBrush(Qt::white);
    painter.drawEllipse(buttonPos, 0, 38, 38); // ボタンの位置に基づいて円を描画

    // テキストの描画
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 12, QFont::Bold));

    if (isOn)
    {
        painter.drawText(rect(), Qt::AlignCenter, "REC"); // ONの場合、中央にONと表示
    }
    else
    {
        painter.drawText(rect(), Qt::AlignCenter, "OFF"); // OFFの場合、中央にOFFと表示
    }
}
// override
void
SlideButton::mousePressEvent(QMouseEvent* /*event*/)
{
    toggle(); // ボタンがクリックされたらON/OFFを切り替え
}
void
SlideButton::toggle()
{
    isOn = !isOn; // ON/OFF状態を切り替え

    // アニメーションの設定
    QPropertyAnimation* animation = new QPropertyAnimation(this, "buttonPos");
    animation->setDuration(200); // アニメーションの速度
    animation->setStartValue(buttonPos);
    animation->setEndValue(isOn ? width() - 40 : 0); // ONの場合は右、OFFの場合は左に動かす
    animation->start(QAbstractAnimation::DeleteWhenStopped); // アニメーション終了後に自動削除

    emit clicked();
}
int
SlideButton::getButtonPos() const
{
    return buttonPos;
}
void
SlideButton::setButtonPos(int pos)
{
    buttonPos = pos;
    update(); // 再描画
}
bool
SlideButton::isRecording()
{
    return isOn;
}

// explicit
CaptureButton::CaptureButton(QWidget* parent)
    : QPushButton(parent), flashTimer(nullptr), isFlashing(false)
{
    setFixedSize(50, 50); // ボタンサイズ設定
    setText("CAP");
    setStyleSheet("background-color: blue; color: white; border-radius: 50px;");

    // フラッシュ用タイマー設定
    flashTimer = new QTimer(this);
    flashTimer->setInterval(100); // 0.1秒間フラッシュ
    flashTimer->setSingleShot(true);

    // フラッシュ終了後に青に戻す
    connect(flashTimer, &QTimer::timeout, this, &CaptureButton::resetFlash);
}
// virtual
CaptureButton::~CaptureButton()
{
}

// override
void
CaptureButton::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);

    // 背景色の変更（通常は青、フラッシュ中は白）
    if (isFlashing)
    {
        painter.setBrush(QColor(Qt::white));
    }
    else
    {
        // painter.setBrush(QColor(Qt::blue));
        painter.setBrush(QColor(50, 50, 180));
    }

    painter.setPen(Qt::NoPen);
    painter.drawEllipse(0, 0, width(), height()); // 丸いボタン

    // テキストの描画
    painter.setPen(QColor(Qt::white));
    painter.setFont(QFont("Arial", 12, QFont::Bold));
    painter.drawText(rect(), Qt::AlignCenter, "CAP");
}

// override
void
CaptureButton::mousePressEvent(QMouseEvent* event)
{
    QPushButton::mousePressEvent(event);

    // 背景を白に変更してフラッシュを開始
    isFlashing = true;
    update();

    // タイマーを開始して0.5秒後に元の色に戻す
    flashTimer->start();
}

void
CaptureButton::resetFlash()
{
    isFlashing = false;
    update(); // 元の色に戻して再描画
}

UdpThread::UdpThread(QObject* parent) : QThread(parent), running(false), udpSocket(nullptr)
{
}

UdpThread::~UdpThread()
{
    stop();
}

void
UdpThread::stop()
{
    running = false;
}

void
UdpThread::run()
{
    udpSocket = new QUdpSocket();

    // ソケットを127.0.0.1にバインド（ループバックアドレスで受信）
    if (!udpSocket->bind(QHostAddress("127.0.0.1"), 5601))
    {
        qDebug() << "Error: Unable to bind UDP socket on port 5601";
        return;
    }

    running = true;

    // パケットを再構成するためのデータ構造
    QMap<int, QByteArray> packetBuffer;
    int expectedPacketCount = -1;

    while (running)
    {
        if (udpSocket->hasPendingDatagrams())
        {
            QByteArray datagram;
            datagram.resize(int(udpSocket->pendingDatagramSize()));
            udpSocket->readDatagram(datagram.data(), datagram.size());

            if (datagram.size() < 8)
            {
                qDebug() << "Invalid packet received (too small)";
                continue;
            }

            int packetId =
                qFromBigEndian<int32_t>(reinterpret_cast<const uchar*>(datagram.constData()));
            int packetCount =
                qFromBigEndian<int32_t>(reinterpret_cast<const uchar*>(datagram.constData() + 4));

            if (expectedPacketCount == -1)
            {
                expectedPacketCount = packetCount;
            }

            QByteArray jpegData = datagram.mid(8);
            packetBuffer[packetId] = jpegData;

            if (packetBuffer.size() == expectedPacketCount)
            {
                QByteArray completeJpegData;
                for (int i = 0; i < expectedPacketCount; ++i)
                {
                    completeJpegData.append(packetBuffer[i]);
                }

                QBuffer buffer(&completeJpegData);
                buffer.open(QIODevice::ReadOnly);
                QImageReader reader(&buffer, "JPEG");
                QImage image = reader.read();

                if (!image.isNull())
                {
                    emit newImage(image);
                }
                else
                {
                    qDebug() << "Failed to decode JPEG image.";
                }

                packetBuffer.clear();
                expectedPacketCount = -1;
            }
        }

        QThread::msleep(10);
    }

    udpSocket->close();
    delete udpSocket;
}

CameraPanel::CameraPanel(QWidget* parent)
    : QWidget(parent), udpThread(new UdpThread(this)), background(QImage())
{
    // QPushButton を作成し、レイアウトに追加
    // button = new QPushButton("Start Receiving", this);
    QVBoxLayout* layout = new QVBoxLayout(this);
    mpSlideButton = new SlideButton();
    mpCaptureButton = new CaptureButton();
    layout->addWidget(mpSlideButton);
    layout->addWidget(mpCaptureButton);
    layout->addStretch();

    // UDPスレッドからのJPEG画像データを受け取る
    connect(udpThread, &UdpThread::newImage, this, &CameraPanel::updateBackground);

    // UDPスレッドを開始
    udpThread->start();

    // ros2
    mpNode = rclcpp::Node::make_shared("camera_panel_node");
    mpClientPlay = mpNode->create_client<std_srvs::srv::SetBool>("/toggle_play");
    mpClientRecord = mpNode->create_client<std_srvs::srv::SetBool>("/toggle_record");
    mpClientCapture = mpNode->create_client<std_srvs::srv::Trigger>("/capture_frame");
    mpRequestPlay = std::make_shared<std_srvs::srv::SetBool::Request>();
    mpRequestRecord = std::make_shared<std_srvs::srv::SetBool::Request>();
    mpRequestCapture = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::thread([&] { rclcpp::spin(mpNode); }).detach();

    connect(mpSlideButton, &SlideButton::clicked, this, &CameraPanel::onSlideButtonClicked);
    connect(mpCaptureButton, &QPushButton::clicked, this, &CameraPanel::onCaptureButtonClicked);
    connect(this, &CameraPanel::completedPlay, this, &CameraPanel::onCompletedPlay);
    connect(this, &CameraPanel::completedRecord, this, &CameraPanel::onCompletedRecord);
    connect(this, &CameraPanel::completedCapture, this, &CameraPanel::onCompletedCapture);
}

CameraPanel::~CameraPanel()
{
    udpThread->stop();
    udpThread->wait();
}

void
CameraPanel::onCompletedPlay(const QString& message, bool success)
{
    mMessage = message;
    update();
}
void
CameraPanel::onCompletedRecord(const QString& message, bool success)
{
    mMessage = message;
    update();
}
void
CameraPanel::onCompletedCapture(const QString& message, bool success)
{
    mMessage = message;
    update();
}
void
CameraPanel::onSlideButtonClicked()
{
    printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaa %s\n", __PRETTY_FUNCTION__);
    bool is_recording = mpSlideButton->isRecording();
    std::thread([&] { requestToggleRecord(is_recording); }).detach();
}
void
CameraPanel::onCaptureButtonClicked()
{
    printf("bbbbbbbbbbbbbbb  %s\n", __PRETTY_FUNCTION__);
    std::thread([&] { requestCaptureImage(); }).detach();
}
void
CameraPanel::updateBackground(const QImage& image)
{
    // 受信した画像を背景に設定
    background = image;
    update(); // QWidget を再描画
}
void
CameraPanel::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu contextMenu(this);
    QAction* startAction = contextMenu.addAction("Start");
    QAction* stopAction = contextMenu.addAction("Stop");

    // スロットをアクションに接続
    connect(startAction, &QAction::triggered, this, &CameraPanel::startStreaming);
    connect(stopAction, &QAction::triggered, this, &CameraPanel::stopStreaming);

    // コンテキストメニューを表示
    contextMenu.exec(event->globalPos());
}

void
CameraPanel::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    if (!background.isNull())
    {
        // 画像のサイズを取得
        int imgWidth = background.width();
        int imgHeight = background.height();

        // ウィジェットのサイズを取得
        int widgetWidth = width();
        int widgetHeight = height();

        // 画像のアスペクト比を計算
        float imgAspect = static_cast<float>(imgWidth) / imgHeight;
        float widgetAspect = static_cast<float>(widgetWidth) / widgetHeight;

        // 描画領域を計算
        QRect targetRect;
        if (imgAspect > widgetAspect)
        {
            // 画像がウィジェットよりも横長の場合
            int newWidth = widgetWidth;
            int newHeight = static_cast<int>(widgetWidth / imgAspect);
            int yOffset = (widgetHeight - newHeight) / 2;
            targetRect = QRect(0, yOffset, newWidth, newHeight);
        }
        else
        {
            // 画像がウィジェットよりも縦長の場合
            int newHeight = widgetHeight;
            int newWidth = static_cast<int>(widgetHeight * imgAspect);
            int xOffset = (widgetWidth - newWidth) / 2;
            targetRect = QRect(xOffset, 0, newWidth, newHeight);
        }

        // 背景を黒で塗りつぶす
        painter.fillRect(rect(), Qt::black);

        // 画像を描画
        painter.drawImage(targetRect, background);

        // messageを描画
        if (!mMessage.isEmpty())
        {
            QFont font = painter.font();
            font.setPointSize(12); // フォントサイズを設定
            painter.setFont(font);

            // テキストのサイズを取得
            QFontMetrics fm(font);
            int textWidth = fm.horizontalAdvance(mMessage);
            int textHeight = fm.height();

            // テキストの背景矩形の座標を計算
            int x = (width() - textWidth) / 2;
            int y = height() - textHeight - 10; // ウィジェットの下部から10px離す

            // テキスト背景の矩形を黄色で描画
            QRect backgroundRect(x, y, textWidth, textHeight);
            painter.setBrush(Qt::yellow);
            painter.setPen(Qt::NoPen); // 枠線なし
            painter.drawRect(backgroundRect);

            // テキストを黒色で描画
            painter.setPen(Qt::black);
            painter.drawText(backgroundRect, Qt::AlignCenter, mMessage);
        }
    }
    QWidget::paintEvent(event);
}
void
CameraPanel::startStreaming()
{
    std::thread([&] { requestTogglePlay(true); }).detach();
}
void
CameraPanel::stopStreaming()
{
    std::thread([&] { requestTogglePlay(false); }).detach();
}
void
CameraPanel::startRecording()
{
}
void
CameraPanel::stopRecording()
{
}
void
CameraPanel::captureImage()
{
}
void
CameraPanel::requestTogglePlay(bool toggle)
{
    mpRequestPlay->data = toggle;
    auto result = mpClientPlay->async_send_request(mpRequestPlay);
    int i = 0;
    while (!mpClientPlay->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            emit completedPlay("interrupted while waiting for service", false);
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        if (++i > 10)
        {
            emit completedPlay("service not available", false);
            return;
        }
    }

    i = 0;
    while (rclcpp::ok())
    {
        if (result.wait_for(1s) == std::future_status::ready)
        {
            QString message;
            message.sprintf("success: %s message: %s", result.get()->success ? "true" : "false",
                            result.get()->message.c_str());
            emit completedPlay(message, result.get()->success);
            break;
        }
        if (++i > 10)
        {
            emit completedPlay("failed to call service", false);
            break;
        }
    }
}
void
CameraPanel::requestToggleRecord(bool toggle)
{
    mpRequestRecord->data = toggle;
    auto result = mpClientRecord->async_send_request(mpRequestRecord);
    int i = 0;
    while (!mpClientRecord->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            emit completedRecord("interrupted while waiting for service", false);
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        if (++i > 10)
        {
            emit completedRecord("service not available", false);
            return;
        }
    }

    i = 0;
    while (rclcpp::ok())
    {
        if (result.wait_for(1s) == std::future_status::ready)
        {
            QString message;
            message.sprintf("success: %s message: %s", result.get()->success ? "true" : "false",
                            result.get()->message.c_str());
            emit completedRecord(message, result.get()->success);
            break;
        }
        if (++i > 10)
        {
            emit completedRecord("failed to call service", false);
            break;
        }
    }
}
void
CameraPanel::requestCaptureImage()
{
    auto result = mpClientCapture->async_send_request(mpRequestCapture);
    int i = 0;
    while (!mpClientCapture->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            emit completedCapture("interrupted while waiting for service", false);
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        if (++i > 10)
        {
            emit completedCapture("service not available", false);
            return;
        }
    }

    i = 0;
    while (rclcpp::ok())
    {
        if (result.wait_for(1s) == std::future_status::ready)
        {
            QString message;
            message.sprintf("success: %s message: %s", result.get()->success ? "true" : "false",
                            result.get()->message.c_str());
            emit completedCapture(message, result.get()->success);
            break;
        }
        if (++i > 10)
        {
            emit completedCapture("failed to call service", false);
            break;
        }
    }
}
