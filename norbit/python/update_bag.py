#!/usr/bin/env python

import rosbag
import sys
import argparse
from norbit_msgs.msg import BathymetricStamped

bag = rosbag.Bag(sys.argv[1])

out_bag = rosbag.Bag(sys.argv[2], 'w')

for topic, msg, t in bag.read_messages():
  if msg._type == "norbit_msgs/BathymetricStamped":
    #print(msg.bathy.bahtymetric_header)
    new_msg = BathymetricStamped()
    new_msg.header = msg.header

    new_msg.bathy.common_header = msg.bathy.common_header

    new_msg.bathy.bathymetric_header.snd_velocity = msg.bathy.bahtymetric_header.snd_velocity
    new_msg.bathy.bathymetric_header.sample_rate = msg.bathy.bahtymetric_header.sample_rate
    new_msg.bathy.bathymetric_header.N = msg.bathy.bahtymetric_header.N
    new_msg.bathy.bathymetric_header.ping_number = msg.bathy.bahtymetric_header.ping_number
    new_msg.bathy.bathymetric_header.time = msg.bathy.bahtymetric_header.time
    new_msg.bathy.bathymetric_header.time_net = msg.bathy.bahtymetric_header.time_net
    new_msg.bathy.bathymetric_header.ping_rate = msg.bathy.bahtymetric_header.ping_rate
    new_msg.bathy.bathymetric_header.type = msg.bathy.bahtymetric_header.type
    new_msg.bathy.bathymetric_header.beam_dist_mode = msg.bathy.bahtymetric_header.beam_dist_mode
    new_msg.bathy.bathymetric_header.sonar_mode = msg.bathy.bahtymetric_header.sonar_mode
    new_msg.bathy.bathymetric_header.reserved1 = msg.bathy.bahtymetric_header.reserved1
    new_msg.bathy.bathymetric_header.tx_angle = msg.bathy.bahtymetric_header.tx_angle
    new_msg.bathy.bathymetric_header.gain = msg.bathy.bahtymetric_header.gain
    new_msg.bathy.bathymetric_header.tx_freq = msg.bathy.bahtymetric_header.tx_freq
    new_msg.bathy.bathymetric_header.tx_bw = msg.bathy.bahtymetric_header.tx_bw
    new_msg.bathy.bathymetric_header.tx_len = msg.bathy.bahtymetric_header.tx_len
    new_msg.bathy.bathymetric_header.reserved2 = msg.bathy.bahtymetric_header.reserved2
    new_msg.bathy.bathymetric_header.tx_voltage = msg.bathy.bahtymetric_header.tx_voltage
    new_msg.bathy.bathymetric_header.swath_dir = msg.bathy.bahtymetric_header.swath_dir
    new_msg.bathy.bathymetric_header.swath_open = msg.bathy.bahtymetric_header.swath_open
    new_msg.bathy.bathymetric_header.gate_tilt = msg.bathy.bahtymetric_header.gate_tilt

    new_msg.bathy.detections = msg.bathy.detections

    out_bag.write(topic, new_msg, t)
  else:
    out_bag.write(topic, msg, t)

    #print(t)

out_bag.close()
bag.close()
