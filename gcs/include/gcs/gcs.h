#ifndef MYCLASS_H
#define MYCLASS_H

#include <QMainWindow>

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
    Ui::GCSInterface*ui;
};

#endif // MYCLASS_H
