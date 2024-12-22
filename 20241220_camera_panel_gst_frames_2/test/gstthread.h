#ifndef GSTTHREAD_H
#define GSTTHREAD_H

#include <QThread>
#include <QImage>
#include <gst/gst.h>

class GstThread : public QThread
{
    Q_OBJECT

public:
    explicit GstThread(QObject *parent = nullptr);
    virtual ~GstThread();

signals:
    void newFrame(const QImage &image);

protected:
    void run() override;

private:
    GstElement *pipeline;
    GstBus *bus;
    bool stop;

    static gboolean onNewSample(GstElement *sink, GstThread *thread);
    QImage gstSampleToQImage(GstSample *sample);
};

#endif // GSTTHREAD_H

