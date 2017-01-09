#ifndef MYCLASS_H
#define MYCLASS_H

#include <QMainWindow>
#include <QGroupBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "qt_gcs/uavInterface.h"

class GCS : public QMainWindow {
    Q_OBJECT

public:
    explicit GCS(int _argc, char **_argv, QWidget *parent = 0);
    ~GCS();

private:
    QMenuBar *mMenuBar;

    QHBoxLayout *mMainLayout;
    QWidget *mCentralWidget;

    QImage *mDummyMap;
    QLabel *mMapLayout;

    QGroupBox *mUavListGroup;
    QVBoxLayout *mUavListLayout;

    UavInterface *mUavInterface1, *mUavInterface2, *mUavInterface3;

};

#endif // MYCLASS_H
