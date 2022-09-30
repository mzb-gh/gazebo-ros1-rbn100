#pragma once

#include <mind_os/serialization/serialization.h>
#include <cstdint>

namespace base_msgs {

struct Header {
    std::uint32_t seq;
    std::uint64_t stamp;
}; /* message Header */

} /* namespace base_msgs */

namespace mind_os {
template<>
void serialize(const base_msgs::Header& data, BytesBuffer& buffer) {
    buffer << data.seq;
    buffer << data.stamp;
}; /* serialization of base_msgs::Header */

template<>
void deserialize(BytesBuffer& buffer, base_msgs::Header& data) {
    buffer >> data.seq;
    buffer >> data.stamp;
}; /* de-serialization of base_msgs::Header */

} /* namespace mind_os */

