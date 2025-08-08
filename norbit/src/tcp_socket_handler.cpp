#include <norbit/tcp_socket_handler.hpp>

TCPSocketHandler::TCPSocketHandler(const ConnectionParams& params)
    :params_(params)
{
    std::cout<<"TCPSocketHandler called now\n";
}

TCPSocketHandler::~TCPSocketHandler()
{
    std::cout<<"~TCPSocketHandler called now\n";
    closeConnection();
}

void TCPSocketHandler::setupConnection()
{
    std::cout << "[setup] starting...." << std::endl;

    //! TODO: make it continus opening if open failed

    if (io_context_.stopped()) {
        io_context_.restart();
    }

    // ===================================================================== //
    // Bathymetric
    // ===================================================================== //

    sockets_.bathymetric = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_context_));

   boost::asio::ip::tcp::endpoint bathy_endpoint(
        boost::asio::ip::address::from_string(params_.ip), params_.bathy_port);

    sockets_.bathymetric->async_connect(bathy_endpoint, [this](const boost::system::error_code& ec) { 
        this->handleConnect("bathymetric", ec, [this]{ 
            this->startStreamReceive(sockets_.bathymetric, hdr_buff_.bathymetric); 
        }); 
    });

    // ===================================================================== //
    // Water Column
    // ===================================================================== //

    if(params_.pubWC()){
        std::cout<<"\n pubWC \n";

        sockets_.water_column = std::unique_ptr<boost::asio::ip::tcp::socket>(
            new boost::asio::ip::tcp::socket(io_context_));        

        boost::asio::ip::tcp::endpoint water_column_endpoint(
            boost::asio::ip::address::from_string(params_.ip), params_.water_column_port);

        sockets_.water_column->async_connect(water_column_endpoint, [this](const boost::system::error_code& ec) { 
            this->handleConnect("water_column", ec, [this]{ 
                this->startStreamReceive(sockets_.water_column, hdr_buff_.water_column); 
            }); 
        });
    }

    // ===================================================================== //
    // CMD
    // ===================================================================== //

    sockets_.cmd = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_context_)); 

    boost::asio::ip::tcp::endpoint cmd_endpoint(
        boost::asio::ip::address::from_string(params_.ip), params_.cmd_port);

    sockets_.cmd->async_connect(cmd_endpoint, [this](const boost::system::error_code& ec) {
        this->handleConnect("cmd", ec, [this]{
                this->startCmdReceive();
        });
    });

    std::cout << "[setup] Successfully connected." << std::endl;


    // The worker thread only runs the io_context for the async sockets.
    worker_thread_ = std::thread([this]() { io_context_.run(); });
}

void TCPSocketHandler::closeConnection()
{
    std::cout << "close start" << std::endl;

    std::cout << "stop the io..." << std::endl;
    // stop the io
    if (!io_context_.stopped()) {
        io_context_.stop();
    }

    std::cout << "stop the thread..." << std::endl;
    // stop the thread
    if (worker_thread_.joinable()) { 
        worker_thread_.join(); 
    }

    // close socket
    auto close_socket = [](const std::string& name, std::unique_ptr<boost::asio::ip::tcp::socket>& socket) {
        if (socket && socket->is_open()) {
            boost::system::error_code ec;
            // Shutdown both send and receive to be polite to the server
            socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
            socket->close(ec);
            if (ec) {
                std::cerr << "[close] Error closing " << name << " socket: " << ec.message() << std::endl;
            }
        }
    };

    close_socket("bathymetric", sockets_.bathymetric);
    close_socket("water_column", sockets_.water_column);
    close_socket("cmd", sockets_.cmd);

    std::cout << "close end" << std::endl;
}

void TCPSocketHandler::handleConnect(
    const std::string& name, const boost::system::error_code& ec, 
    std::function<void()> on_success) 
{
    if (!ec) {
        std::cout << "[" << name << "] Successfully connected." << std::endl;
        on_success();
    } else {
        std::cerr << "[" << name << "] Connect error: " << ec.message() << std::endl;
    }
}

void TCPSocketHandler::startStreamReceive(
    std::unique_ptr<boost::asio::ip::tcp::socket>& socket, 
    boost::array<char, header_size_>& buffer)
{
    buffer.assign(0);

    socket->async_receive(boost::asio::buffer(buffer),
        [this, &socket, &buffer](const boost::system::error_code& ec, std::size_t bytes) {
            if (!ec) {

                //! DEBUG:
                // std::ostringstream oss;
                // for (size_t i = 0; i < bytes; ++i) {
                //     oss << std::hex << std::setfill('0') << std::setw(2)
                //         << static_cast<int>(static_cast<unsigned char>(buffer[i])) << " ";
                // }
                // std::cout<< "Received raw data (hex): " << oss.str() <<std::endl;

                //! Parse the header
                processHdrMsg(socket, buffer);
                
                //! Continue loop
                startStreamReceive(socket, buffer); 
            } 
            else {
                if (ec != boost::asio::error::operation_aborted) {
                    std::cerr << "[socket] Read error: " << ec.message() << std::endl;
                }
            }
        });

}

void TCPSocketHandler::startCmdReceive()
{
    // Most commands are echoed with 0x0d 0x0a (\r\n), but
    // set_ntp_server is termianted with 0x0a only.

    // boost::asio::async_read_until(*sockets_.cmd, cmd_resp_buffer_, "\r\n",
    boost::asio::async_read_until(*sockets_.cmd, cmd_resp_buffer_, "\n",
        [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (!ec) {
                // A complete line is in the buffer. Extract and process it.
                std::istream is(&cmd_resp_buffer_);
                std::string line;
                std::getline(is, line);
                std::cout << "[cmd] Received command: \"" << line << "\"" << std::endl;

                // save
                cmd_buffer_mutex_.lock();
                cmd_resp_queue_.push_back(line);
                cmd_buffer_mutex_.unlock();
                
                // Continue listening for the next command.
                startCmdReceive();
            } else {
                if (ec != boost::asio::error::operation_aborted) {
                    std::cerr << "[socket] Read error: " << ec.message() << std::endl;
                }
            }
        });
}

void TCPSocketHandler::processHdrMsg(
    std::unique_ptr<boost::asio::ip::tcp::socket>& socket, 
    boost::array<char, header_size_>& hdr)
{
    try {
        norbit_types::Message msg;
        if (msg.fromBoostArray(hdr)) {
            const unsigned int dataSize = msg.commonHeader().size - sizeof(norbit_msgs::msg::CommonHeader);
            std::shared_ptr<char> dataPtr;
            dataPtr.reset(new char[dataSize]);
            size_t bytesRead =read(*socket,boost::asio::buffer(dataPtr.get(), dataSize));

            if(msg.setBits(dataPtr)){
                if ((msg.commonHeader().type == norbit_types::bathymetric) && bathyCallback_) {
                    // callback for bathymetric
                    bathyCallback_(msg.getBathy());
                }
                if ((msg.commonHeader().type == norbit_types::watercolum) && wcCallback_){
                    // callback for water column
                    wcCallback_(msg.getWC());
                }
            }
            else {
                //! TODO: why only report watercolumn? also the msg.setBits() return void
                std::cout<<"Watercolumn Message failed CRC check:  Ignoring\n";
            }
        }
        else{
            if(msg.commonHeader().version!=NORBIT_CURRENT_VERSION) {
                std::cout<<"Invalid version detected expected " << NORBIT_CURRENT_VERSION<<", got "<< msg.commonHeader().version <<std::endl;
            }
            if(msg.commonHeader().preable==norbit_msgs::msg::CommonHeader::NORBIT_PREAMBLE_KEY) {
                std::cout<<"Invalid header preable detected\n";
            }
        }
    } catch (...) {
        std::cout<<"An unhandled exception occured in NorbitConnection::recHandler()\n";
    }
}

void TCPSocketHandler::removeSubstrs(
    std::string &s, const std::string p) 
{
    std::string::size_type n = p.length();
    for (std::string::size_type i = s.find(p); i != std::string::npos;
        i = s.find(p))
        s.erase(i, n);
}


norbit_msgs::msg::CmdResp TCPSocketHandler::sendCmd(
    const std::string &cmd,
    const std::string &val) {

    // ===================================================================== //
    // prepare the message
    // ===================================================================== //
    std::string message = cmd + " " + val;
    std::string key = cmd;

    // some of the norbit responses don't echo back set_<cmd> so we need to strip
    // it
    removeSubstrs(key, "set_");
    removeSubstrs(key, " ");

    // ===================================================================== //
    // send the message
    // ===================================================================== //

    std::cout<<"[Norbit - sendCmd] command message sent: " << message.c_str() <<std::endl;

    boost::system::error_code ec;
    boost::asio::write(*sockets_.cmd, boost::asio::buffer(message), ec);
    if (ec) {
        throw std::runtime_error("[Norbit - sendCmd] Failed to write command: " + ec.message());
    }

    // ===================================================================== //
    // grab the received message
    // ===================================================================== //

    auto time_send = std::chrono::high_resolution_clock::now();

    // construct the msg
    norbit_msgs::msg::CmdResp out;
    out.success = false;
    out.ack = false;
    out.resp = "";

    bool running = true;
    do{
        cmd_buffer_mutex_.lock();

        if (cmd_resp_queue_.size() > 0) {
            out.resp = cmd_resp_queue_.front();

            // check if this the right message
            if (cmd_resp_queue_.front().find(key) != std::string::npos) {
                std::cout<<"[Norbit - sendCmd] ACK Received: " << cmd_resp_queue_.front() <<std::endl;
                cmd_resp_queue_.pop_front();
                out.success = true;
                out.ack = true;
                running = false;
            } else {
                cmd_resp_queue_.pop_front();
                std::cout<<"[Norbit - sendCmd] response not match to [" << key <<" ]\n";
            }
        } 
        else {
            // check how long we waited since message sent
            auto time_now = std::chrono::high_resolution_clock::now();
            auto time_wait = std::chrono::duration_cast<std::chrono::microseconds>(
                time_now - time_send).count() * 1e-6;

            if (time_wait > params_.cmd_timeout) {

                if (out.resp != "") {
                    std::cout<<"ERROR: [Norbit - sendCmd] received bad ACK: " << out.resp <<std::endl;
                    out.ack = true;
                } 
                else {
                    std::cout<<"ERROR: [Norbit - sendCmd] TIMEOUT -- no ACK received" <<std::endl;
                    out.ack = false;
                }

                running = false;
            }
        }

        cmd_buffer_mutex_.unlock();

    } while(running);

    return out;
}