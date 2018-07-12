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

#ifndef _MICRORTPS_AGENT_UTILS_XRCEFACTORY_HPP_
#define _MICRORTPS_AGENT_UTILS_XRCEFACTORY_HPP_

#include <micrortps/agent/types/XRCETypes.hpp>
#include <micrortps/agent/types/MessageHeader.hpp>
#include <micrortps/agent/utils/Serializer.hpp>

#include <stdint.h>

namespace eprosima {
namespace micrortps {

class XRCEFactory
{
public:
    XRCEFactory(char* buffer, size_t max_size) : serializer_(buffer, max_size) {}
    void header(const dds::xrce::MessageHeader& header);
    void status(const dds::xrce::STATUS_Payload& payload);
    void status_agent(const dds::xrce::STATUS_AGENT_Payload& payload);
    void acknack(const dds::xrce::ACKNACK_Payload& payload);
    void heartbeat(const dds::xrce::HEARTBEAT_Payload& payload);

    void data(const dds::xrce::DATA_Payload_Data& payload);
    void data(const dds::xrce::DATA_Payload_Sample& payload);
    void data(const dds::xrce::DATA_Payload_DataSeq& payload);
    void data(const dds::xrce::DATA_Payload_SampleSeq& payload);
    void data(const dds::xrce::DATA_Payload_PackedSamples& payload);

    size_t get_total_size();
private:

    void submessage_header(dds::xrce::SubmessageId submessage_id,
                           uint8_t flags,
                           uint16_t submessage_length);
    void reply(dds::xrce::SubmessageId submessage_id,
               const dds::xrce::BaseObjectRequest& object_reply,
               uint8_t data_flag);

    Serializer serializer_;
};

} // namespace micrortps
} // namespace eprosima

#endif //_MICRORTPS_AGENT_UTILS_XRCEFACTORY_HPP_
