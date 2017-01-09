#ifndef MYCLASS_H
#define MYCLASS_H

#include <QMainWindow>
#include <QGroupBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace Ui {
class GCSInterface;
}

class GCS : public QMainWindow
{
    Q_OBJECT

public:
    explicit GCS(QWidget *parent = 0);
    ~GCS();

private:
    QMenuBar *mMenuBar;

    QHBoxLayout *mMainLayout;
    QWidget *mCentralWidget;

    QImage *mDummyMap;
    QLabel *mMapLayout;

    QGroupBox *mUavListGroup;
    QVBoxLayout *mUavListLayout;

    QGroupBox *mUavInterface1, *mUavInterface2, *mUavInterface3;

};

#endif // MYCLASS_H
