// Copyright 2017-present Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UXR_AGENT_TRANSPORT_SERIAL_SERIALAGENTLINUX_HPP_
#define UXR_AGENT_TRANSPORT_SERIAL_SERIALAGENTLINUX_HPP_

#include <uxr/agent/transport/Server.hpp>
#include <uxr/agent/transport/endpoint/SerialEndPoint.hpp>
#include <uxr/agent/transport/stream_framing/StreamFramingProtocol.hpp>

#include <cstdint>
#include <cstddef>
#include <sys/poll.h>

//GPIO出力用
#include <fcntl.h>
#include <unistd.h>
#include <wiringPi.h>

namespace eprosima {
namespace uxr {

class SerialAgent : public Server<SerialEndPoint>
{
public:
    SerialAgent(
            uint8_t addr,
            Middleware::Kind middleware_kind);

#ifdef UAGENT_DISCOVERY_PROFILE
    bool has_discovery() final { return false; }
#endif

#ifdef UAGENT_P2P_PROFILE
    bool has_p2p() final { return false; }
#endif

private:
    virtual bool init() = 0;

    virtual bool fini() = 0;

    std::mutex write_mtx_;
    std::mutex wait_mtx_;
    std::condition_variable cond_;
    bool dataReady = false;
    int now_reading = 0;
    
    
    int GPIO_Write = 4; // 物理ピン 16 (GPIO 22)
    int GPIO_Read = 5; // 物理ピン GPIO23
    int GPIO_Debug = 3; //物理ピン 15 
    int count_buffer = 1;
    
    
    int num_for_debug = 0;
    int from_7E = 0;
    bool found_7E_flag = 0;

    int length_1st = 0x00;
    int length_2nd = 0x00;
    int length_hex = 0x00;


    bool recv_message(
            InputPacket<SerialEndPoint>& input_packet,
            int timeout,
            TransportRc& transport_rc) final;

    bool send_message(
            OutputPacket<SerialEndPoint> output_packet,
            TransportRc& transport_rc) final;

    ssize_t write_data(
            uint8_t* buf,
            size_t len,
            TransportRc& transport_rc);

    ssize_t read_data(
            uint8_t* buf,
            size_t len,
            int timeout,
            TransportRc& transport_rc);

protected:
    const uint8_t addr_;
    struct pollfd poll_fd_;
    uint8_t buffer_[SERVER_BUFFER_SIZE];
    FramingIO framing_io_;


};

} // namespace uxr
} // namespace eprosima

#endif // UXR_AGENT_TRANSPORT_SERIAL_SERIALAGENTLINUX_HPP_
