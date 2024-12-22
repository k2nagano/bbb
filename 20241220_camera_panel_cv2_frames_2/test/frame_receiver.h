#ifndef FRAME_RECEIVER_H
#define FRAME_RECEIVER_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>

class FrameReceiver {
public:
    FrameReceiver(int port);
    ~FrameReceiver();

    void start();
    void stop();
    bool getFrame(cv::Mat& frame);

private:
    void receiveFrames();

    std::atomic<bool> running;
    int port;
    cv::Mat currentFrame;
    std::mutex frameMutex;
    std::thread receiveThread;
};

#endif // FRAME_RECEIVER_H

