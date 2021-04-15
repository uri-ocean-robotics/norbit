#include "norbit_connection.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "norbit_node");
  NorbitConnection con;
  con.spin();

}

