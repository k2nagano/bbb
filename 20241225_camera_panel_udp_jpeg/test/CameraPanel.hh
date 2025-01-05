#ifndef CAMERA_PANEL_HH
#define CAMERA_PANEL_HH

#include <QImage>
#include <QThread>
#include <QUdpSocket>
#include <QWidget>
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class SlideButton : public QWidget
{
    Q_OBJECT
public:
    explicit SlideButton(QWidget* parent = nullptr);
    virtual ~SlideButton();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
signals:
    void clicked();
private slots:
    void toggle();

public:
    int getButtonPos() const;
    void setButtonPos(int pos);
    bool isRecording();

private:
    bool isOn;     // ON/OFF状態を保持
    int buttonPos; // ボタンの現在位置
    // ボタン位置のプロパティを定義
    Q_PROPERTY(int buttonPos READ getButtonPos WRITE setButtonPos)
};

class CaptureButton : public QPushButton
{
    Q_OBJECT
public:
    explicit CaptureButton(QWidget* parent = nullptr);
    virtual ~CaptureButton();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
private slots:
    void resetFlash();

private:
    QTimer* flashTimer; // フラッシュタイマー
    bool isFlashing;    // フラッシュ状態を管理
};

class UdpThread : public QThread
{
    Q_OBJECT
public:
    explicit UdpThread(QObject* parent = nullptr);
    ~UdpThread();
    void stop();
signals:
    void newImage(const QImage& image);

protected:
    void run() override;

private:
    std::atomic<bool> running;
    QUdpSocket* udpSocket;
};

class CameraPanel : public QWidget
{
    Q_OBJECT
public:
    CameraPanel(QWidget* parent = nullptr);
    ~CameraPanel();
signals:
    void completedPlay(const QString& message, bool success);
    void completedRecord(const QString& message, bool success);
    void completedCapture(const QString& message, bool success);
private slots:
    void onCompletedPlay(const QString& message, bool success);
    void onCompletedRecord(const QString& message, bool success);
    void onCompletedCapture(const QString& message, bool success);
    void onSlideButtonClicked();
    void onCaptureButtonClicked();
public slots:
    void updateBackground(const QImage& image);

protected:
    void contextMenuEvent(QContextMenuEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

private:
    void startStreaming();
    void stopStreaming();
    void startRecording();
    void stopRecording();
    void captureImage();
    void requestTogglePlay(bool toggle);
    void requestToggleRecord(bool toggle);
    void requestCaptureImage();

private:
    rclcpp::Node::SharedPtr mpNode;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mpClientPlay;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mpClientRecord;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mpClientCapture;
    std::shared_ptr<std_srvs::srv::SetBool::Request> mpRequestPlay;
    std::shared_ptr<std_srvs::srv::SetBool::Request> mpRequestRecord;
    std::shared_ptr<std_srvs::srv::Trigger::Request> mpRequestCapture;

private:
    QImage background;
    UdpThread* udpThread;
    QPushButton* button;
    SlideButton* mpSlideButton;
    CaptureButton* mpCaptureButton;
    QString mMessage;
};

#endif // CAMERA_PANEL_HH
