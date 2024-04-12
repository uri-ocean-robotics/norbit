#ifndef TYPE_CONVERER_H
#define TYPE_CONVERER_H

#include "defs.h"
#include <marine_acoustic_msgs/DetectionFlag.h>
#include <marine_acoustic_msgs/RawSonarImage.h>
#include <marine_acoustic_msgs/SonarRanges.h>
#include <marine_acoustic_msgs/SonarDetections.h>

#include <norbit_msgs/BathymetricStamped.h>
#include <norbit_msgs/WaterColumnStamped.h>

NS_HEAD
namespace conversions {
  /*!
   * \brief Converts norbit_msgs::BathymetricStamped to marine_acoustic_msgs::SonarRanges all parameters passed by reference
   * \param in the norbit_msgs::BathymetricStamped you want to convert
   * \param out the marine_acoustic_msgs::SonarRanges that will be overwritten with the converted Batymetric data
   */
  void bathymetric2SonarRanges(const norbit_msgs::BathymetricStamped & in, marine_acoustic_msgs::SonarRanges & out);
  void bathymetric2SonarDetections(const norbit_msgs::BathymetricStamped & in, marine_acoustic_msgs::SonarDetections & out);
  void norbitWC2RawSonarImage(const norbit_msgs::WaterColumnStamped & in, marine_acoustic_msgs::RawSonarImage & out);
}
NS_FOOT

#endif // TYPE_CONVERER_H
