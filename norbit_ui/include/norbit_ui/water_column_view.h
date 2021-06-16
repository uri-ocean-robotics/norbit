#ifndef WATER_COLUMN_VIEW_H
#define WATER_COLUMN_VIEW_H

#include <QWidget>
#include "qcustomplot.h"
#include <ros/ros.h>
#include <norbit_msgs/WaterColumnStamped.h>
#include <qtimer.h>

namespace Ui {
class WaterColumnView;
}

class WaterColumnView : public QWidget
{
  Q_OBJECT

public:
  explicit WaterColumnView(QWidget *parent = nullptr);
  ~WaterColumnView();

  void wcCallback(const norbit_msgs::WaterColumnStamped::ConstPtr& wc_msg);
private slots:
  void spinOnce();



private:
  Ui::WaterColumnView *ui;
  ros::NodeHandlePtr nh_;
  ros::Subscriber wc_sub_;
  QTimer *ros_timer;
  //QCPColorMap *colorMap;
};

#endif // WATER_COLUMN_VIEW_H
