#include "conversions.h"

NS_HEAD

namespace conversions {


  void bathymetric2SonarRanges(const norbit_msgs::BathymetricStamped & in,
                               acoustic_msgs::SonarRanges & out){
    auto num_beams = in.bathy.bahtymetric_header.N;

    out.header = in.header;

    out.ping_info.frequency = in.bathy.bahtymetric_header.tx_freq;
    // NOTE(lindzey): Kris -- do you want to correct spelling in the message field?
    double sos = in.bathy.bahtymetric_header.snd_velocity;
    out.ping_info.sound_speed = sos;
    out.ping_info.tx_beamwidths.resize(num_beams);
    out.ping_info.rx_beamwidths.resize(0); // not reported

    out.flags.resize(num_beams);
    out.transmit_delays.resize(num_beams);
    out.intensities.resize(num_beams);
    out.beam_unit_vec.resize(num_beams);
    out.ranges.resize(num_beams);

    for (size_t i = 0; i < num_beams; i++) {
      if (in.bathy.detections[i].quality_flag == 3) {
        out.flags[i].flag = acoustic_msgs::DetectionFlag::DETECT_OK;
      } else {
        out.flags[i].flag = acoustic_msgs::DetectionFlag::DETECT_BAD_SONAR;
      }
      out.transmit_delays[i] = 0;
      out.intensities[i] = in.bathy.detections[i].intensity;
      double el = in.bathy.bahtymetric_header.tx_angle;
      double az = in.bathy.detections[i].angle;
      out.beam_unit_vec[i].x = sin(el);
      out.beam_unit_vec[i].y = -1 * cos(el) * sin(az);
      out.beam_unit_vec[i].z = cos(el) * cos(az);
      double twtt = in.bathy.detections[i].sample_number / in.bathy.bahtymetric_header.sample_rate;
      out.ranges[i]  = 0.5 * twtt * sos;
      out.ping_info.tx_beamwidths[i]  = in.bathy.bahtymetric_header.tx_bw;
    }
    return;
  }

  void bathymetric2SonarDetections(const norbit_msgs::BathymetricStamped & in,
                                   acoustic_msgs::SonarDetections & out){
    auto num_beams = in.bathy.bahtymetric_header.N;

    out.header = in.header;

    out.ping_info.frequency = in.bathy.bahtymetric_header.tx_freq;
    out.ping_info.sound_speed = in.bathy.bahtymetric_header.snd_velocity;
    out.ping_info.tx_beamwidths.resize(num_beams);
    out.ping_info.rx_beamwidths.resize(num_beams); // not reported

    out.flags.resize(num_beams);
    out.two_way_travel_times.resize(num_beams);
    out.tx_delays.resize(num_beams);
    out.intensities.resize(num_beams);
    out.tx_angles.resize(num_beams);
    out.rx_angles.resize(num_beams);
    for (size_t i = 0; i < num_beams; i++) {
      if (in.bathy.detections[i].quality_flag == 3) {
        out.flags[i].flag = acoustic_msgs::DetectionFlag::DETECT_OK;
      } else {
        out.flags[i].flag = acoustic_msgs::DetectionFlag::DETECT_BAD_SONAR;
      }

      out.two_way_travel_times[i] = in.bathy.detections[i].sample_number / in.bathy.bahtymetric_header.sample_rate;
      out.tx_delays[i] = 0;
      out.intensities[i] = in.bathy.detections[i].intensity;
      out.tx_angles[i] = in.bathy.bahtymetric_header.tx_angle;
      out.rx_angles[i] = in.bathy.detections[i].angle;
      out.ping_info.tx_beamwidths[i] = in.bathy.bahtymetric_header.tx_bw;
    }
    return;
  }

  void norbitWC2RawSonarImage(const norbit_msgs::WaterColumnStamped & in,
                              acoustic_msgs::RawSonarImage & out) {
    auto num_beams = in.water_column.water_column_header.N;  // Number of beams
    auto num_samples = in.water_column.water_column_header.M;  // Number of samples in each beam

    out.header = in.header;

    out.ping_info.frequency = in.water_column.water_column_header.tx_freq;
    out.ping_info.sound_speed = in.water_column.water_column_header.snd_velocity;
    out.ping_info.tx_beamwidths.resize(num_beams);
    // out.ping_info.rx_beamwidths =

    out.sample_rate = in.water_column.water_column_header.sample_rate;
    out.samples_per_beam = num_samples;
    if (in.water_column.water_column_header.t0 < 0) {
      out.sample0 = 0;
    } else {
      out.sample0 = uint32_t(in.water_column.water_column_header.t0);
    }
    // NOTE(lindzey): RawSonarImage doesn't currently have a num_beams field;
    //     discussion in the merge request was leaning towards adding it.
    // out.num_beams = num_beams;

    out.tx_delays.resize(num_beams);
    out.tx_angles.resize(num_beams);
    out.rx_angles.resize(num_beams);
    for (size_t i = 0; i < num_beams ; i++) {
      out.tx_delays[i] = 0;
      out.tx_angles[i] = in.water_column.water_column_header.tx_angle;
      out.rx_angles[i] = in.water_column.beam_directions[i];
      out.ping_info.tx_beamwidths[i] = in.water_column.water_column_header.tx_bw;
    }

    // out.image.is_bigendian
    out.image.dtype = in.water_column.water_column_header.dtype;
    out.image.data = in.water_column.pixel_data;

    return;
  }


}

NS_FOOT
