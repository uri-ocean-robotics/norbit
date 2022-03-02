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

  void bathymetric2MultibeamDetections(const norbit_msgs::BathymetricStamped & in, acoustic_msgs::MultibeamDetections & out){


    auto n = in.bathy.bahtymetric_header.N;
    out.flag.resize(n);
    out.two_way_travel_time.resize(n);
    out.transmit_delay.resize(n);
    out.intensity.resize(n);
    out.tx_angle.resize(n);
    out.rx_angle.resize(n);
    out.tx_beamwidth.resize(n);
    out.rx_beamwidth.resize(0); // not reported

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
      out.tx_angle[i]      = in.bathy.bahtymetric_header.tx_angle;
      out.rx_angle[i]        = in.bathy.detections[i].angle;
      out.tx_beamwidth[i]  = in.bathy.bahtymetric_header.tx_bw;
    }
    return;
  }

  void norbitWC2HydroWC(const norbit_msgs::WaterColumnStamped & in, acoustic_msgs::MultibeamWatercolumn & out){
    out.header=in.header;
    out.sound_speed=in.water_column.water_column_header.snd_velocity;
    out.sample_rate=in.water_column.water_column_header.sample_rate;

    auto n = in.water_column.water_column_header.N;// Number of beams
    auto m = in.water_column.water_column_header.M;// Number of samples in each beam

    out.num_beams = n;
    out.samples_per_beam = m;

    out.transmit_delay.resize(n);
    out.sample0.resize(n);
    out.tx_angle.resize(n);
    out.rx_angle.resize(n);
    out.tx_beamwidth.resize(n);

    for(size_t i = 0; i<n ; i++){
      out.transmit_delay[i] = 0;
      if(in.water_column.water_column_header.t0<0)
        out.sample0[i] = 0;
      else
        out.sample0[i] = uint32_t(in.water_column.water_column_header.t0);
      out.tx_angle[i] = in.water_column.water_column_header.tx_angle;
      out.tx_beamwidth[i] = in.water_column.water_column_header.tx_bw;
    }
    out.rx_angle = in.water_column.beam_directions;
    out.dtype = in.water_column.water_column_header.dtype;
    out.data = in.water_column.pixel_data;

    return;
  }


}

NS_FOOT
