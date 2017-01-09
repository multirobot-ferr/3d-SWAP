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

class UavInterface : public QGroupBox {
    Q_OBJECT

public:
    UavInterface(int _index);
    ~UavInterface();
private:
    QHBoxLayout *mMainLayoutUav;

    QVBoxLayout *mOdometryLayoutUav;

    QVBoxLayout *mActionsLayoutUav;
    QHBoxLayout *mTakeOffLayout;
    QPushButton *mTakeOffButton;
    QSpinBox *mTakeOffAltitude;

    QHBoxLayout *mTargetLayout;
    QPushButton *mTargetButton;
    QSpinBox *mColorSpin;
    QSpinBox *mShapeSpin;
};

#endif
