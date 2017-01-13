///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <qt_gcs/gcs.h>
#include <QApplication>
#include <ros/ros.h>
#include <thread>

int main(int argc, char *argv[])
{
    if(!ros::isInitialized()){
        ros::init(argc, argv, "qt_gcs");
    }

    std::thread spinThread([&](){
       ros::spin();
    });

    QApplication app (argc, argv);

    GCS gcs(argc, argv);
    gcs.show();

    return app.exec();
}
