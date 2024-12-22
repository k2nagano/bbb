#include "frame_receiver.h"
#include <gst/gst.h>

FrameReceiver::FrameReceiver(int port) : port(port), running(false) {
    gst_init(nullptr, nullptr);
}

FrameReceiver::~FrameReceiver() {
    stop();
}

void FrameReceiver::start() {
    running = true;
    receiveThread = std::thread(&FrameReceiver::receiveFrames, this);
}

void FrameReceiver::stop() {
    running = false;
    if (receiveThread.joinable()) {
        receiveThread.join();
    }
}

bool FrameReceiver::getFrame(cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(frameMutex);
    if (!currentFrame.empty()) {
        frame = currentFrame.clone();
        return true;
    }
    return false;
}

void FrameReceiver::receiveFrames() {
    std::string pipelineStr = "udpsrc port=" + std::to_string(port) + " ! application/x-rtp, payload=96 ! "
                              "rtph264depay ! avdec_h264 ! videoconvert ! appsink";
    cv::VideoCapture cap(pipelineStr, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "Failed to open pipeline for UDP H.264 stream." << std::endl;
        return;
    }

    cv::Mat frame;
    while (running) {
        if (cap.read(frame)) {
            std::lock_guard<std::mutex> lock(frameMutex);
            currentFrame = frame;
        }
    }
}

