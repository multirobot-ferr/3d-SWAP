#include <qt_gcs/gcs.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GCS gcs;
    gcs.show();

    return a.exec();
}
