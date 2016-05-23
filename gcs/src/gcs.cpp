#include "gcs/gcs.h"
#include "ui_gcs.h"

GCS::GCS(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GCSInterface)
{
    ui->setupUi(this);
}

GCS::~GCS()
{
    delete ui;
}
