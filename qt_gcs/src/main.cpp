#include <qt_gcs/gcs.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app (argc, argv);

    GCS gcs;
    gcs.show();

    return app.exec();
}
