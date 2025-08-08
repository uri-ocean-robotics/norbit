#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <norbit/parameters.hpp>
#include <norbit_msgs/msg/common_header.hpp>
#include <norbit_msgs/msg/cmd_resp.hpp>

#include "norbit/norbit_types/message.h"
#include "norbit/conversions.h"

class TCPSocketHandler {
private:
    static constexpr std::size_t header_size_ = sizeof(norbit_msgs::msg::CommonHeader);

    //! sockets for all the interface
    struct {
        std::unique_ptr<boost::asio::ip::tcp::socket> bathymetric;
        std::unique_ptr<boost::asio::ip::tcp::socket> water_column;
        std::unique_ptr<boost::asio::ip::tcp::socket> cmd;
    } sockets_;

    //! data header buffer
    struct{
        boost::array<char, header_size_> bathymetric;
        boost::array<char, header_size_> water_column;
    } hdr_buff_;
    
    //! boost io_service for data interface
    boost::asio::io_context io_context_;  

    //! thread
    std::thread worker_thread_;

    // Mutex to protect the command socket from concurrent access
    std::recursive_mutex cmd_buffer_mutex_;

    //! TCP related parameters
    //! TODO: make the param struct simpler: ros param and TCP param 
    ConnectionParams params_;

    //! Buffer for received data from CMD response
    boost::asio::streambuf cmd_resp_buffer_;
    
    //! The CMD response string
    std::deque<std::string> cmd_resp_queue_;

    /**
     * @brief Sets up a callback to receive the raw binary UDP socket data
     * @param[in] data norbit_types::BathymetricData type coming data
     *
     * This function allows the user to define a callback that will be ...
     */    
    std::function <void(norbit_types::BathymetricData)> bathyCallback_;     

    /**
     * @brief Sets up a callback to receive the raw binary UDP socket data
     * @param[in] data norbit_types::WaterColumnData type coming data
     *
     * This function allows the user to define a callback that will be ...
     */    
    std::function <void(norbit_types::WaterColumnData)> wcCallback_;     

    void handleConnect(
        const std::string& name, 
        const boost::system::error_code& ec, 
        std::function<void()> on_success);

    void startStreamReceive(
        std::unique_ptr<boost::asio::ip::tcp::socket>& socket, 
        boost::array<char, header_size_>& buffer);

    void startCmdReceive();

    void processHdrMsg(
        std::unique_ptr<boost::asio::ip::tcp::socket>& socket, 
        boost::array<char, header_size_>& hdr);

    void removeSubstrs(
        std::string &s, const std::string p);

public:
    /**
     * @brief Default constructor
     * @param[in] param the TCP related parameters
     */  
    TCPSocketHandler(
        const ConnectionParams& params);    

    /**
     * @brief Default deconstructor
     */ 
    ~TCPSocketHandler();    

    void setupConnection();

    void closeConnection();

    norbit_msgs::msg::CmdResp sendCmd(
        const std::string &cmd,
        const std::string &val);

    /**
     * @brief The registration function to setup the callback for bathymetric data
     * @param[in] bathyCallback_  the std::function to pass data
     */
    void setCallbackBathy(decltype(bathyCallback_) cb) { bathyCallback_  = cb;}

    /**
     * @brief The registration function to setup the callback for water column data
     * @param[in] wcCallback_  the std::function to pass data
     */
    void setCallbackWC(decltype(wcCallback_) cb) { wcCallback_  = cb;}

};