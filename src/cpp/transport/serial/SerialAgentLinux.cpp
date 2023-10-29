#include <uxr/agent/transport/serial/SerialAgentLinux.hpp>
#include <uxr/agent/utils/Conversion.hpp>
#include <uxr/agent/logger/Logger.hpp>

#include <unistd.h>
#include <wiringPi.h>
#include <stdio.h> 

namespace eprosima {
namespace uxr {


SerialAgent::SerialAgent(
        uint8_t addr,
        Middleware::Kind middleware_kind)
    : Server<SerialEndPoint>{middleware_kind}
    , addr_{addr}
    , poll_fd_{}
    , buffer_{0}
    , framing_io_(
          addr,
          std::bind(&SerialAgent::write_data, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
          std::bind(&SerialAgent::read_data, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4))
    {
        wiringPiSetup () ;
        pinMode (GPIO_Read, OUTPUT) ;
        pinMode (GPIO_Debug, OUTPUT) ;
        pinMode (GPIO_Write, OUTPUT) ;
        digitalWrite (GPIO_Debug, 0) ;
    }



ssize_t SerialAgent::write_data(
        uint8_t* buf,
        size_t len,
        TransportRc& transport_rc)
{   
    size_t rv = 0;
    ssize_t bytes_written = ::write(poll_fd_.fd, buf, len);

    //XRCEメッセージの先頭を見つける処理
        //バッファ内探索
        for(int i=0; i<bytes_written; i++){
            if(buf[i]== 0x7E){
                from_7E = 0;
                found_7E_flag = true;
            }
        }
        
        //XRCEメッセージを読み込み中の処理(0x7E~末尾まで)
        if(found_7E_flag){  

            //読み込み開始のflag
            write_mtx_.lock();
                now_reading = 1;
            write_mtx_.unlock();

            //バッファ内探索
            for(int i=0; i<bytes_written; i++){
                //データ長の取得
                switch (from_7E){
                    case 3:
                        length_1st = buf[i];
                        break;
                    case 4:
                        length_2nd = buf[i];
                        if (buf[i]!=0x00) length_hex = buf[0] * 16 * 16 + length_1st;
                        else length_hex = length_1st;
                        break;
                }
                from_7E ++;


                //末尾到達時の処理
                if (from_7E>length_hex+5){
                    write_mtx_.lock();
                        now_reading = 0;
                    write_mtx_.unlock();
                    
                    //mutexの解放
                    cond_.notify_one();
                
                    found_7E_flag = false;
                    
                }
            }
        }

    if (0 < bytes_written)
    {
        rv = size_t(bytes_written);
    }
    else
    {
        transport_rc = TransportRc::server_error;
    }
    return rv;
}


ssize_t SerialAgent::read_data(
        uint8_t* buf,
        size_t len,
        int timeout,
        TransportRc& transport_rc)
{   
    FILE* fp_debug = fopen("debug.txt", "a");

    ssize_t bytes_read = 0;
    int poll_rv = poll(&poll_fd_, 1, timeout); 
    if(poll_fd_.revents & (POLLERR+POLLHUP))
    {
        transport_rc = TransportRc::server_error;;
    }
    else if (0 < poll_rv)
    {
        if (now_reading == 1){
        digitalWrite (GPIO_Debug, 1) ;
        std::unique_lock<std::mutex> lock(wait_mtx_);
        cond_.wait(lock, [this] { return !now_reading; });
        digitalWrite (GPIO_Debug, 0) ;
    }
        bytes_read = read(poll_fd_.fd, buf, len);

        if (0 > bytes_read)
        {
            transport_rc = TransportRc::server_error;
        }
    }
    else
    {
        transport_rc = (poll_rv == 0) ? TransportRc::timeout_error : TransportRc::server_error;
    }

    fclose(fp_debug);

    return bytes_read;
}

bool SerialAgent::recv_message(
        InputPacket<SerialEndPoint>& input_packet,
        int timeout,
        TransportRc& transport_rc)
{
    bool rv = false;
    uint8_t remote_addr = 0x00;
    ssize_t bytes_read = 0;

    do
    {
        bytes_read = framing_io_.read_framed_msg(
            buffer_, SERVER_BUFFER_SIZE, remote_addr, timeout, transport_rc);
    }
    while ((0 == bytes_read) && (0 < timeout));

    if (0 < bytes_read)
    {
        input_packet.message.reset(new InputMessage(buffer_, static_cast<size_t>(bytes_read)));
        input_packet.source = SerialEndPoint(remote_addr);
        rv = true;

        uint32_t raw_client_key;
        if (Server<SerialEndPoint>::get_client_key(input_packet.source, raw_client_key))
        {
            UXR_AGENT_LOG_MESSAGE(
                UXR_DECORATE_YELLOW("[==>> SER <<==]"),
                raw_client_key,
                input_packet.message->get_buf(),
                input_packet.message->get_len());
        }
    }
    return rv;

}

bool SerialAgent::send_message(
        OutputPacket<SerialEndPoint> output_packet,
        TransportRc& transport_rc)
{
    bool rv = false;
    ssize_t bytes_written =
            framing_io_.write_framed_msg(
                output_packet.message->get_buf(),
                output_packet.message->get_len(),
                output_packet.destination.get_addr(),
                transport_rc);
    if ((0 < bytes_written) && (
         static_cast<size_t>(bytes_written) == output_packet.message->get_len()))
    {
        rv = true;

        uint32_t raw_client_key;
        if (Server<SerialEndPoint>::get_client_key(output_packet.destination, raw_client_key))
        {
            UXR_AGENT_LOG_MESSAGE(
                UXR_DECORATE_YELLOW("[** <<SER>> **]"),
                raw_client_key,
                output_packet.message->get_buf(),
                output_packet.message->get_len());
        }
    }
    return rv;
}

} // namespace uxr
} // namespace eprosima