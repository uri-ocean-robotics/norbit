#include "conversions.h"

NS_HEAD

namespace conversions {


  void bathymetric2SonarRanges(const norbit_msgs::BathymetricStamped & in, acoustic_msgs::SonarRanges & out){


    auto n = in.bathy.bahtymetric_header.N;
    out.flag.resize(n);
    out.two_way_travel_time.resize(n);
    out.transmit_delay.resize(n);
    out.intensity.resize(n);
    out.elevation_angle.resize(n);
    out.azimuth_angle.resize(n);
    out.elevation_beamwidth.resize(n);
    out.azimuth_beamwidth.resize(0); // not reported

    out.header                    = in.header;
    out.sound_speed               = in.bathy.bahtymetric_header.snd_velocity;

    for (size_t i = 0; i < n; i++) {
      if (in.bathy.detections[i].quality_flag == 3)
        out.flag[i]               = acoustic_msgs::SonarRanges::BEAM_OK;
      else
        out.flag[i]               = acoustic_msgs::SonarRanges::BEAM_BAD_SONAR;

      out.two_way_travel_time[i]  = in.bathy.detections[i].sample_number / in.bathy.bahtymetric_header.sample_rate;
      out.transmit_delay[i]       = 0;
      out.intensity[i]            = in.bathy.detections[i].intensity;
      out.elevation_angle[i]      = in.bathy.bahtymetric_header.tx_angle;
      out.azimuth_angle[i]        = in.bathy.detections[i].angle;
      out.elevation_beamwidth[i]  = in.bathy.bahtymetric_header.tx_bw;
    }
    return;
  }


}

NS_FOOT
