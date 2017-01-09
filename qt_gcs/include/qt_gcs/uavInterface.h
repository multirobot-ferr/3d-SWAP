///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _QTGCS_UAVINTERFACE_H_
#define _QTGCS_UAVINTERFACE_H_

#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QRadioButton>

#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/server.h>

class UavInterface : public QGroupBox {
    Q_OBJECT

public:
    UavInterface(int _argc, char **_argv, int _index);
    ~UavInterface();

private slots:
    void takeOffCallback();
    void targetCallback();

private:
    int mUavId;
    QHBoxLayout *mMainLayoutUav;

    QVBoxLayout *mOdometryLayoutUav;

    QVBoxLayout *mActionsLayoutUav;
    QHBoxLayout *mTakeOffLayout;
    QPushButton *mTakeOffButton;
    QDoubleSpinBox *mTakeOffAltitude;

    QHBoxLayout *mTargetLayout;
    QPushButton *mTargetButton;
    QRadioButton *mTargetEnable;
    QSpinBox *mColorSpin;
    QSpinBox *mShapeSpin;

};

#endif
