#include <QApplication>
#include "water_column_view.h"

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "water_column_view");
  QApplication a(argc, argv);
  WaterColumnView w;
  w.show();  
  return a.exec();
}
