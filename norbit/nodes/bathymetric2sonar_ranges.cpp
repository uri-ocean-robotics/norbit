#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <norbit/conversions.h>


int main(int argc, char *argv[]) {

  if(argc < 3){
    std::cerr <<  "please specify an input and output bag \nbathymetric2sonar_ranges input.bag output.bag" << std::endl;
  }
  rosbag::Bag in_bag;
  in_bag.open(argv[1]);  // BagMode is Read by default

  rosbag::Bag out_bag;
  out_bag.open(argv[2], rosbag::bagmode::Write);

  acoustic_msgs::SonarRanges::Ptr sonar_ranges_msg(new acoustic_msgs::SonarRanges);

  for(rosbag::MessageInstance const m: rosbag::View(in_bag))
  {
    norbit_msgs::BathymetricStamped::ConstPtr i = m.instantiate<norbit_msgs::BathymetricStamped>();
    if (i != nullptr){
      norbit::conversions::bathymetric2SonarRanges(*i,*sonar_ranges_msg);
      out_bag.write(m.getTopic()+"/ranges",m.getTime(),sonar_ranges_msg);
    }
    out_bag.write(m.getTopic(), m.getTime(), m);
  }

  in_bag.close();
  out_bag.close();
  return 0;
}
