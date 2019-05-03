#include <iostream>
#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "../include/norbit/s7k.h"
#include "../include/norbit/raw_detection_data.h"


using namespace boost::asio;
using ip::tcp;



class NorbitConnection{
public:
  NorbitConnection();
  void receive();
  void recHandler(
      const boost::system::error_code& error, // Result of operation.
      std::size_t bytes_transferred           // Number of bytes received.
    );
  void spin();
private:
  std::unique_ptr<tcp::socket> socket_;
  boost::asio::io_service io_service_;
  boost::array<char, sizeof(s7k::NetworkFrame)+sizeof(s7k::DataRecordFrameHead)> recv_buffer_;
};

NorbitConnection::NorbitConnection(){
  socket_ = std::unique_ptr<tcp::socket>(new tcp::socket(io_service_) );
  socket_->connect(tcp::endpoint( boost::asio::ip::address::from_string("192.168.53.2"), 7000 ));
  receive();
}

void NorbitConnection::receive(){
  recv_buffer_.assign(0);
  socket_->async_receive(boost::asio::buffer(recv_buffer_), 0,
                         boost::bind(&NorbitConnection::recHandler, this, boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
  //std::cout<< "setup complete" << std::endl;
}

void NorbitConnection::recHandler(const boost::system::error_code &error, std::size_t bytes_transferred){
  //std::cout<< "rec bytes: " << bytes_transferred << std::endl;
  const s7k::NetworkFrame * s7k_data = reinterpret_cast<const s7k::NetworkFrame*>( &recv_buffer_ );
  if(s7k_data->protocolVersion==5){
    std::cout<< std::endl << "rec bytes: " << bytes_transferred << std::endl;
    std::cout << "protocol version: " << s7k_data->protocolVersion << std::endl;
    std::cout << "Offset: " << s7k_data->offset << std::endl;
    std::cout << "totalPackets: " << s7k_data->totalPackets << std::endl;
    std::cout << "totalRecords: " << s7k_data->totalRecords << std::endl;
    std::cout << "sequenceNumber: " << s7k_data->sequenceNumber << std::endl;
    const s7k::DataRecordFrameHead * drf_data = reinterpret_cast<const s7k::DataRecordFrameHead*>( &recv_buffer_[sizeof(s7k::NetworkFrame)] );
    std::cout << "drf_protocolVersion: " << drf_data->protocolVersion << std::endl;
    if( drf_data->syncPattern == 0x0000FFFF ){
      std::cout << "drf_syncPattern correct " << std::endl;
    }else{
      std::cout << "drf_syncPattern incorrect " << std::endl;;
    }
    std::cout << "drf_syncPattern: " << drf_data->syncPattern << std::endl;
    std::cout << "drf size: " << drf_data->size <<std::endl;
    std::cout << "drf recordTypeIdentifier: " << drf_data->recordTypeIdentifier <<std::endl;
    std::cout << "totalRecordsInFragmentedData: " << drf_data->totalRecordsInFragmentedData <<std::endl;

    // read datagram
    const int dataSize = drf_data->size-sizeof(s7k::DataRecordFrameHead);
    boost::array<char, 50000> dataBuffer;
    boost::system::error_code error;
    size_t bytesRead = read(*socket_,boost::asio::buffer(dataBuffer,dataSize));
    std::cout << "bytesRead: " << bytesRead <<std::endl;
    if (error == boost::asio::error::eof)
      return; // Connection closed cleanly by peer.
    else if (error)
      throw boost::system::system_error(error); // Some other error.

    if(drf_data->recordTypeIdentifier==7027){
      const s7k::RawDetectionData::RTH * detection = reinterpret_cast<const s7k::RawDetectionData::RTH*>( &dataBuffer );
      std::cout << "sonarId: " << detection->sonarId << std::endl;
      std::cout << "pingNumber: " << detection->pingNumber << std::endl;
      std::cout << "multipingSequence: " << detection->multipingSequence << std::endl;
      std::cout << "N: " << detection->n << std::endl;
      std::cout << "dataFieldSize: " << detection->dataFieldSize << std::endl;
      std::cout << "detectionAlgorithm: " << int(detection->detectionAlgorithm) << std::endl;
      std::cout << "samplingRate: " << detection->samplingRate << std::endl;
      const s7k::RawDetectionData::RD * detectionPoints =
          reinterpret_cast<const s7k::RawDetectionData::RD*>(
            &dataBuffer[sizeof(s7k::RawDetectionData::RTH)] );
      std::cout << "size of RD " << sizeof(s7k::RawDetectionData::RD) << std::endl;
      for(size_t i = 0 ; i<detection->n; i++){
        std::cout << "   beamDescriptor: " <<detectionPoints[i].beamDescriptor << std::endl;
        std::cout << "   rx angle: " <<detectionPoints[i].rxAngle*180/3.14159 << std::endl;
        std::cout << "   detection point: " << detectionPoints[i].detectionPoint << std::endl;
      }
//      std::cout << "   beamDescriptor: " <<detectionPoints[0].beamDescriptor << std::endl;
//      std::cout << "   rx angle: " <<detectionPoints[0].rxAngle*180/3.14159 << std::endl;
//      std::cout << "   detection point: " << detectionPoints[0].detectionPoint << std::endl;
//      std::cout << "   beamDescriptor: " <<detectionPoints[detection->n-1].beamDescriptor << std::endl;
//      std::cout << "   rx angle: " <<detectionPoints[detection->n-1].rxAngle*180/3.14159 << std::endl;
    }


  }
  else{
    std::cout<< "rec bytes: " << bytes_transferred << std::endl;
  }
  receive();
  return;
}

void NorbitConnection::spin(){
  //io_service_.run();
  while(true)
    io_service_.run_one();
}

int main(int argc, char* argv[])
{
  //ros::init(argc, argv, "norbit_node");
  NorbitConnection con;
  con.spin();

}


////
//// client.cpp
//// ~~~~~~~~~~
////
//// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
////
//// Distributed under the Boost Software License, Version 1.0. (See accompanying
//// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
////

//#include <iostream>
//#include <boost/array.hpp>
//#include <boost/asio.hpp>

//using boost::asio::ip::tcp;

//int main(int argc, char* argv[])
//{
//  try
//  {
//    if (argc != 2)
//    {
//      std::cerr << "Usage: client <host>" << std::endl;
//      return 1;
//    }

//    boost::asio::io_service io_service;

//    tcp::resolver resolver(io_service);
//    tcp::resolver::query query(argv[1],"7000");
//    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

//    tcp::socket socket(io_service);
//    boost::asio::connect(socket, endpoint_iterator);

//    for (;;)
//    {
//      boost::array<char, 128> buf;
//      boost::system::error_code error;

//      size_t len = socket.read_some(boost::asio::buffer(buf), error);

//      if (error == boost::asio::error::eof)
//        break; // Connection closed cleanly by peer.
//      else if (error)
//        throw boost::system::system_error(error); // Some other error.

//      std::cout.write(buf.data(), len);
//    }
//  }
//  catch (std::exception& e)
//  {
//    std::cerr << e.what() << std::endl;
//  }

//  return 0;
//}
