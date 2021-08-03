#include "water_column_view.h"
#include "ui_water_column_view.h"

WaterColumnView::WaterColumnView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::WaterColumnView)
{
  ui->setupUi(this);
  nh_.reset(new ros::NodeHandle("~"));

  updateTopics();

  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);

  ui->plot->setInteractions(QCP::iRangeDrag|QCP::iRangeZoom); // this will also allow rescaling the color scale by dragging/zooming
  ui->plot->axisRect()->setupFullAxesBox(true);
  colorMap = new QCPColorMap(ui->plot->xAxis, ui->plot->yAxis);
  setRange(ui->range->value());

  setupSignals();

  //colorMap = new QCPColorMap(ui->plot->xAxis, ui->plot->yAxis);
}

WaterColumnView::~WaterColumnView()
{
  delete ui;
}

void WaterColumnView::setupSignals(){
    QObject::connect(ui->plot, SIGNAL(mouseMove(QMouseEvent*)),
                     this,SLOT(updateRangeBearing(QMouseEvent*)));
}

double getRange(const norbit_msgs::WaterColumnStamped::ConstPtr &wc_msg, size_t sample_number){
    double range = double(sample_number) *
            wc_msg->water_column.water_column_header.snd_velocity /
            (2.0 * wc_msg->water_column.water_column_header.sample_rate);
    return  range;
}

int getSampleNo(const norbit_msgs::WaterColumnStamped::ConstPtr &wc_msg, double range){
    double scale = (2.0 * wc_msg->water_column.water_column_header.sample_rate) /
                       wc_msg->water_column.water_column_header.snd_velocity;
    int sample_no = range * scale;
    return sample_no;
}

double rowMajor(const norbit_msgs::WaterColumnStamped::ConstPtr &wc_msg, double  u, double v){

  auto data = reinterpret_cast<const int16_t*>(wc_msg->water_column.pixel_data.data());
  u = u + 0.5 - (u<0);
  v = v + 0.5 - (v<0);
  auto rows = wc_msg->water_column.water_column_header.M;
  auto cols = wc_msg->water_column.water_column_header.N;
  auto index = int(v)*int(cols)+int(u);

  if (int(u)>=cols || int(v)>=rows || int(u)<0 || int(v)<0 || index >= int(rows * cols) || index < 0){
    return 0.0;
  }else {
    return data[index];
  }
}

double getVal(const norbit_msgs::WaterColumnStamped::ConstPtr &wc_msg,_1D::LinearInterpolator<double> beam_idx_interp, double x, double y){


  auto angle = atan2(x,y);
  if(angle>wc_msg->water_column.beam_directions.back() || angle<wc_msg->water_column.beam_directions.front()){
    return 0;
  }else{

      x = getSampleNo(wc_msg,x);
      y = getSampleNo(wc_msg,y);


      auto u = beam_idx_interp(angle);
      auto v = std::sqrt(std::pow(x,2)+std::pow(y,2));
      auto out = rowMajor(wc_msg,u,v);

      return out;
  }
}




void WaterColumnView::wcCallback(const norbit_msgs::WaterColumnStamped::ConstPtr &wc_msg){
  auto M = wc_msg->water_column.water_column_header.M;
  auto N = wc_msg->water_column.water_column_header.N;
  const uint8_t *bits = wc_msg->water_column.pixel_data.data();

  auto beam_angles = wc_msg->water_column.beam_directions;
  std::vector<float> beam_index;
  beam_index.resize(N);
  for(size_t i = 0; i< beam_index.size(); i++){
    beam_index[i] = i;
  }
  _1D::LinearInterpolator<double> beam_idx_interp;
  beam_idx_interp.setData(beam_angles,beam_index);

//  void * mutable_data;
//  auto data_vect = wc_msg->water_column.pixel_data;
//  mutable_data= reinterpret_cast<void*>(data_vect.data());
////  //mutable_data = malloc(sizeof(uint8_t)*M*N);
//  //memcpy(mutable_data,reinterpret_cast<void const*>(wc_msg->water_column.pixel_data.data()),sizeof(uint8_t)*M*N);
//  cv::Mat raw_mat =           cv::Mat(M,N,CV_16U,mutable_data);
//  cv::Mat transformed_mat =   cv::Mat(200,200,CV_16U);
//  cv::Mat x_map =             cv::Mat(M,N,CV_32FC1);
//  cv::Mat y_map =             cv::Mat(M,N,CV_32FC1);

//  for(int row = 0; row<transformed_mat.rows ; row++){
//    for(int col = 0; col<transformed_mat.cols; col++){
//      double x = (row - double(transformed_mat.rows)/2.0)*4;
//      double y = (col - double(transformed_mat.cols)/2.0)*4;
//      transformed_mat.at<uint16_t>(row,col) = getVal(wc_msg,beam_idx_interp,x,y);//row * std::sin( wc_msg->water_column.beam_directions[beam]);
//      transformed_mat.at<uint16_t>(row,col) = getVal(wc_msg,beam_idx_interp,x,y);//row * std::cos( wc_msg->water_column.beam_directions[beam]);
//    }
//  }

  //cv::remap(raw_mat,transformed_mat,x_map,y_map,cv::INTER_NEAREST);

//  cv::Point2f center( (float)raw_mat.cols / 2, 0);
//  double maxRadius = raw_mat.rows;

//  cv::warpPolar(raw_mat, transformed_mat, transformed_mat.size(), center, maxRadius, cv::WARP_INVERSE_MAP);

//  cv::imshow("WC",transformed_mat);
//  cv::imshow("raw",raw_mat);


  //ROS_INFO("wc callback");
  size_t type_size = 0;
  ui->plot->clearItems();

  ui->plot->yAxis->setRangeReversed(ui->reverse_y->checkState());
  ui->plot->xAxis->setRangeReversed(ui->reverse_x->checkState());


  int nx = 300;
  int ny = 400;
  colorMap->data()->setSize(nx, ny); // we want the color map to have nx * ny data points

  //double max_range = ui->range->value();

  ui->plot->yAxis->setScaleRatio(ui->plot->xAxis,1.0);
  colorMap->data()->setRange(QCPRange(ui->plot->xAxis->range().lower, ui->plot->xAxis->range().upper), QCPRange(ui->plot->yAxis->range().lower, ui->plot->yAxis->range().upper));


  switch (wc_msg->water_column.water_column_header.dtype) {
  case norbit_msgs::WaterColumnHeader::DTYPE_INT16:
    type_size = 2;
    auto len = M*N;
    auto data = reinterpret_cast<const int16_t*>(bits);


    double x, y, z;
    for (int xIndex=0; xIndex<nx; ++xIndex)
    {
      for (int yIndex=0; yIndex<ny; ++yIndex)
      {
        colorMap->data()->cellToCoord(xIndex, yIndex, &x, &y);
        z = getVal(wc_msg,beam_idx_interp,x,y);
        colorMap->data()->setCell(xIndex, yIndex, z);
      }
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
  //colorMap->rescaleDataRange();

  colorMap->setDataRange(QCPRange(0,ui->gain->maximum() - ui->gain->value()));

  // make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
  QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot);
  ui->plot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
//  colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);

  // rescale the key (x) and value (y) axes so the whole color map is visible:
  //ui->plot->rescaleAxes();


  ui->plot->replot();



//  transformed_mat.deallocate();
//  x_map.deallocate();
//  y_map.deallocate();

  return;
}


void WaterColumnView::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}


void WaterColumnView::on_wc_topic_currentIndexChanged(const QString &arg1)
{
  if(wc_sub_.getTopic() != arg1.toStdString())
    wc_sub_ = nh_->subscribe<norbit_msgs::WaterColumnStamped>(arg1.toStdString(), 1, &WaterColumnView::wcCallback, this);
}

void WaterColumnView::updateRangeBearing(QMouseEvent *event){
    QPoint p = event->pos();
    double x = ui->plot->xAxis->pixelToCoord(p.x());
    double y = ui->plot->yAxis->pixelToCoord(p.y());
    double range = std::sqrt(std::pow(x,2)+std::pow(y,2));
    double bearing = std::atan2(x,y)*180/3.14519;
    QString text;
    text.sprintf("Cursor Location:  x=%04.1f, y=%04.1f, range=%04.1f, bearing=%04.1f", x,y,range,bearing);
    ui->range_bearing->setText(text);
}

void WaterColumnView::on_fullscreen_btn_clicked()
{
    isFullScreen() ? showNormal() : showFullScreen();
}

void WaterColumnView::setRange(double range){
  ui->plot->yAxis->setRange(0,range);
  ui->plot->xAxis->setScaleRatio(ui->plot->yAxis,1.0);
  auto size = ui->plot->xAxis->range().size();
  ui->plot->xAxis->setRange(-size/2,size/2);
}

void WaterColumnView::on_range_valueChanged(double arg1)
{
  setRange(arg1);
}

void WaterColumnView::updateTopics(){
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  ui->wc_topic->clear();
  QStringList topic_list;
  for(auto topic : master_topics){
    if(topic.datatype=="norbit_msgs/WaterColumnStamped"){
      QString::fromStdString(topic.name);
      topic_list.push_back(QString::fromStdString(topic.name));
    }
  }
  ui->wc_topic->addItems(topic_list);
}

void WaterColumnView::on_refresh_btn_clicked()
{
  updateTopics();
}
