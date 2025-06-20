#ifndef TYPE_CONVERER_H
#define TYPE_CONVERER_H

#include <cmath>
#include "defs.h"
#include <marine_acoustic_msgs/msg/detection_flag.hpp>
#include <marine_acoustic_msgs/msg/raw_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_ranges.hpp>
#include <marine_acoustic_msgs/msg/sonar_detections.hpp>

#include <norbit_msgs/msg/bathymetric_stamped.hpp>
#include <norbit_msgs/msg/water_column_stamped.hpp>

NS_HEAD
namespace conversions {
    /*!
    * \brief Converts norbit_msgs::msg::BathymetricStamped to marine_acoustic_msgs::msg::SonarRanges all parameters passed by reference
    * \param in the norbit_msgs::msg::BathymetricStamped you want to convert
    * \param out the marine_acoustic_msgs::msg::SonarRanges that will be overwritten with the converted Batymetric data
    */
    void bathymetric2SonarRanges(
        const norbit_msgs::msg::BathymetricStamped & in, 
        marine_acoustic_msgs::msg::SonarRanges & out);

    void bathymetric2SonarDetections(
        const norbit_msgs::msg::BathymetricStamped & in, 
        marine_acoustic_msgs::msg::SonarDetections & out);

    void norbitWC2RawSonarImage(
        const norbit_msgs::msg::WaterColumnStamped & in, 
        marine_acoustic_msgs::msg::RawSonarImage & out);
}
NS_FOOT

#endif // TYPE_CONVERER_H
