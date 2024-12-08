#include "Widget.hh"
#include "VideoWidget.hh"
#include <QMultimedia>

// explicit
Widget::Widget(QWidget* pParent) : QWidget(pParent)
{
    QVBoxLayout* p_layout = new QVBoxLayout();
    setLayout(p_layout);
    mpMediaPlayer = new QMediaPlayer();
    mpVideoWidget = new VideoWidget(this);
    mpVideoWidget->show();
    p_layout->addWidget(mpVideoWidget);
    mpMediaPlayer->setVideoOutput(mpVideoWidget);
    // mpMediaPlayer->setMedia(QUrl("gst-pipeline: videotestsrc ! qtvideosink"));
    // mpMediaPlayer->setMedia(QUrl("file:///home/nagano/xxx/20241207_qt_media_player_video_widget/test/sample-5s.mp4"));
    // mpMediaPlayer->setMedia(QUrl("gst-pipeline: videotestsrc ! xvimagesink name=\"qtvideosink\""));
    // mpMediaPlayer->setMedia(QUrl::fromLocalFile("/home/nagano/xxx/20241207_qt_media_player_video_widget/test/sample-5s.mp4"));
    // mpMediaPlayer->setMedia(QUrl::fromLocalFile("./sample-5s.mp4"));
    mpMediaPlayer->play();
}

// virtual
Widget::~Widget()
{
}
