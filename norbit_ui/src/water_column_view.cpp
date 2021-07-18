#include "water_column_view.h"
#include "ui_water_column_view.h"

WaterColumnView::WaterColumnView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::WaterColumnView)
{
  ui->setupUi(this);
  nh_.reset(new ros::NodeHandle("~"));
  wc_sub_ = nh_->subscribe<norbit_msgs::WaterColumnStamped>("/herc/perception/sensors/norbit/water_column", 1, &WaterColumnView::wcCallback, this);

  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);

  ui->plot->setInteractions(QCP::iRangeDrag|QCP::iRangeZoom); // this will also allow rescaling the color scale by dragging/zooming
  ui->plot->axisRect()->setupFullAxesBox(true);
  ui->plot->xAxis->setLabel("x");
  ui->plot->yAxis->setLabel("y");

  //colorMap = new QCPColorMap(ui->plot->xAxis, ui->plot->yAxis);
}

WaterColumnView::~WaterColumnView()
{
  delete ui;
}

float row(size_t index, size_t M, size_t N){

}

void WaterColumnView::wcCallback(const norbit_msgs::WaterColumnStamped::ConstPtr &wc_msg){
  auto M = wc_msg->water_column.water_column_header.M;
  auto N = wc_msg->water_column.water_column_header.N;
  const uint8_t *bits = wc_msg->water_column.pixel_data.data();

  void * mutable_data;
  auto data_vect = wc_msg->water_column.pixel_data;
  mutable_data= reinterpret_cast<void*>(data_vect.data());
  //mutable_data = malloc(sizeof(uint8_t)*M*N);
  //memcpy(mutable_data,reinterpret_cast<void const*>(wc_msg->water_column.pixel_data.data()),sizeof(uint8_t)*M*N);
  cv::Mat raw_mat =           cv::Mat(M,N,CV_16U,mutable_data);
  cv::Mat transformed_mat =   cv::Mat(500,500,CV_16U);
  cv::Mat x_map =             cv::Mat(M,N,CV_32FC1);
  cv::Mat y_map =             cv::Mat(M,N,CV_32FC1);

  for(int row = 0; row<transformed_mat.rows ; row++){
    for(int col = 0; col<transformed_mat.cols; col++){
      x_map.at<double>(row,col) = //row * std::sin( wc_msg->water_column.beam_directions[beam]);
      y_map.at<double>(row,col) = //row * std::cos( wc_msg->water_column.beam_directions[beam]);
    }
  }

  cv::remap(raw_mat,transformed_mat,x_map,y_map,cv::INTER_LINEAR);

  cv::imshow("WC",transformed_mat);
  cv::imshow("raw",raw_mat);


  //ROS_INFO("wc callback");
  size_t type_size = 0;
  ui->plot->clearItems();


  QCPColorMap *colorMap = new QCPColorMap(ui->plot->xAxis, ui->plot->yAxis);
  colorMap->data()->setSize(N, M);
  colorMap->setInterpolate(false);


  switch (wc_msg->water_column.water_column_header.dtype) {
  case norbit_msgs::WaterColumnHeader::DTYPE_INT16:
    type_size = 2;
    auto len = M*N;
    auto data = reinterpret_cast<const int16_t*>(bits);
    double x, y, z;
    for (size_t arr_idx = 0; arr_idx<len; arr_idx++) {
      double val = data[arr_idx];

      size_t yIndex = arr_idx / N;
      size_t xIndex = arr_idx % N;

      //colorMap->data()->cellToCoord(xIndex, yIndex, &x, &y);
      colorMap->data()->setCell(xIndex, yIndex, val);

    }
    break;
  }

//  QCPColorScale *colorScale = new QCPColorScale(ui->plot);
//  ui->plot->plotLayout()->addElement(0, 1, colorScale); // add it to the right of the main axis rect
//  colorScale->setType(QCPAxis::atRight); // scale shall be vertical bar with tick/axis labels right (actually atRight is already the default)
//  colorMap->setColorScale(colorScale); // associate the color map with the color scale
//  colorScale->axis()->setLabel("Magnetic Field Strength");

  // set the color gradient of the color map to one of the presets:
  colorMap->setGradient(QCPColorGradient::gpHot);
  // we could have also created a QCPColorGradient instance and added own colors to
  // the gradient, see the documentation of QCPColorGradient for what's possible.

  // rescale the data dimension (color) such that all data points lie in the span visualized by the color gradient:
  colorMap->rescaleDataRange();

  // make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
  QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot);
  ui->plot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
//  colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);

  // rescale the key (x) and value (y) axes so the whole color map is visible:
  ui->plot->rescaleAxes();

  ui->plot->replot();


  transformed_mat.deallocate();
  x_map.deallocate();
  y_map.deallocate();

  return;
}

void WaterColumnView::spinOnce(){
  if(ros::ok())
      ros::spinOnce();
  else
      QApplication::quit();
}

