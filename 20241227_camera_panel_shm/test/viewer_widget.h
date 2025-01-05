#ifndef VIEWER_WIDGET_H
#define VIEWER_WIDGET_H

#include <QWidget>
#include <QTimer>
#include <QImage>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <mutex>

#define MAX_WIDTH 1920
#define MAX_HEIGHT 1080

struct SharedMemory {
    unsigned long serial_number;
    int width;
    int height;
    unsigned char data[MAX_HEIGHT * MAX_WIDTH * 3];  // RGB888 format
};

class ViewerWidget : public QWidget {
    Q_OBJECT

public:
    explicit ViewerWidget(QWidget *parent = nullptr);
    ~ViewerWidget();

protected:
    void paintEvent(QPaintEvent *event) override;

private slots:
    void updateFrame();

private:
    QTimer *timer_;
    QImage current_image_;
    int shm_fd_;
    SharedMemory *shared_memory_;
    std::mutex mutex_;
};

#endif // VIEWER_WIDGET_H

