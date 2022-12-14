#ifndef TYPE_CONVERER_H
#define TYPE_CONVERER_H

#include "defs.h"
#include <acoustic_msgs/DetectionFlag.h>
#include <acoustic_msgs/RawSonarImage.h>
#include <acoustic_msgs/SonarRanges.h>
#include <acoustic_msgs/SonarDetections.h>

#include <norbit_msgs/BathymetricStamped.h>
#include <norbit_msgs/WaterColumnStamped.h>

NS_HEAD
namespace conversions {
  /*!
   * \brief Converts norbit_msgs::BathymetricStamped to acoustic_msgs::SonarRanges all parameters passed by reference
   * \param in the norbit_msgs::BathymetricStamped you want to convert
   * \param out the acoustic_msgs::SonarRanges that will be overwritten with the converted Batymetric data
   */
  void bathymetric2SonarRanges(const norbit_msgs::BathymetricStamped & in, acoustic_msgs::SonarRanges & out);
  void bathymetric2SonarDetections(const norbit_msgs::BathymetricStamped & in, acoustic_msgs::SonarDetections & out);
  void norbitWC2RawSonarImage(const norbit_msgs::WaterColumnStamped & in, acoustic_msgs::RawSonarImage & out);
}
NS_FOOT

#endif // TYPE_CONVERER_H
