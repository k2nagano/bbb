#include "viewer_widget.h"
#include <QPainter>
#include <QImage>
#include <QTimer>

ViewerWidget::ViewerWidget(QWidget *parent) : QWidget(parent) {
    // Initialize shared memory
    shm_fd_ = shm_open("/video_shm", O_RDONLY, 0666);
    if (shm_fd_ == -1) {
        throw std::runtime_error("Failed to open shared memory.");
    }

    shared_memory_ = (SharedMemory*)mmap(0, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (shared_memory_ == MAP_FAILED) {
        throw std::runtime_error("Failed to map shared memory.");
    }

    // Timer for updating the frame at 30Hz
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &ViewerWidget::updateFrame);
    timer_->start(33);  // 30Hz = 1000ms / 30 = 33ms
}

ViewerWidget::~ViewerWidget() {
    munmap(shared_memory_, sizeof(SharedMemory));
    // close(shm_fd_);
}

void ViewerWidget::updateFrame() {
    std::lock_guard<std::mutex> lock(mutex_);

    // Ensure the shared memory has valid data
    if (shared_memory_->width > 0 && shared_memory_->height > 0) {
        // Create QImage from shared memory data (RGB888 format)
        current_image_ = QImage(shared_memory_->data, 
                                shared_memory_->width, 
                                shared_memory_->height, 
                                shared_memory_->width * 3,  // Bytes per line (3 bytes per pixel for RGB888)
                                QImage::Format_RGB888);

        // Update the display
        update();
    }
}

void ViewerWidget::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);

    if (current_image_.isNull()) return;

    // Paint the image onto the widget
    QPainter painter(this);

    // Scale the image while keeping aspect ratio
    QImage scaled_image = current_image_.scaled(this->size(), Qt::KeepAspectRatio);

    // Calculate top-left position to center the image
    int x = (this->width() - scaled_image.width()) / 2;
    int y = (this->height() - scaled_image.height()) / 2;

    // Draw the image at the calculated position
    painter.drawImage(x, y, scaled_image);
}
