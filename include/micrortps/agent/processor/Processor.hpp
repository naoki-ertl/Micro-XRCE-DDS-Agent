// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _MICRORTPS_AGENT_PROCESSOR_PROCESSOR_HPP_
#define _MICRORTPS_AGENT_PROCESSOR_PROCESSOR_HPP_

#include <micrortps/agent/types/XRCETypes.hpp>
#include <micrortps/agent/types/MessageHeader.hpp>
#include <micrortps/agent/types/SubMessageHeader.hpp>
#include <micrortps/agent/message/Message.hpp>
#include <micrortps/agent/utils/Serializer.hpp>

namespace eprosima {
namespace micrortps {

class Agent;
class ProxyClient;

class Processor
{
public:
    Processor(Agent& root) : root_(root) {}

    void process_input_message(const XrceMessage& input_message, uint32_t addr, uint16_t port);

private:
    void process_message(const dds::xrce::MessageHeader& header, Serializer& deserializer, ProxyClient& client);
    bool process_create_client_submessage(dds::xrce::MessageHeader& header, Serializer& deserializer, uint32_t addr, uint16_t port);
    bool process_delete_client_submessage(dds::xrce::MessageHeader& header, Serializer& deserializer, uint32_t addr, uint16_t port);
    bool process_create_submessage(const dds::xrce::MessageHeader& header, const dds::xrce::SubmessageHeader& sub_header, Serializer& deserializer, ProxyClient& client);
    bool process_delete_submessage(const dds::xrce::MessageHeader& header, const dds::xrce::SubmessageHeader& sub_header, Serializer& deserializer, ProxyClient& client);
    bool process_write_data_submessage(const dds::xrce::MessageHeader& /*header*/, const dds::xrce::SubmessageHeader& sub_header, Serializer& deserializer, ProxyClient& client);
    bool process_read_data_submessage(const dds::xrce::MessageHeader& header, const dds::xrce::SubmessageHeader& /*sub_header*/, Serializer& deserializer, ProxyClient& client);
    bool process_acknack_submessage(const dds::xrce::MessageHeader& header, const dds::xrce::SubmessageHeader& /*sub_header*/, Serializer& deserializer, ProxyClient& client);
    bool process_heartbeat_submessage(const dds::xrce::MessageHeader& header, const dds::xrce::SubmessageHeader& /*sub_header*/, Serializer& deserializer, ProxyClient& client);

private:
    Agent& root_;
};

} // namespace micrortps
} // namespace eprosima

#endif //_MICRORTPS_AGENT_PROCESSOR_PROCESSOR_HPP_
