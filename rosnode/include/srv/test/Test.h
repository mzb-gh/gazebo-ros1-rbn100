#pragma once

#include <mind_os/serialization/serialization.h>
#include <cstdint>


struct Test {
    struct Request {
        std::uint32_t time;
    } request; /* struct Request */
    struct Response {
        bool success;
    } response; /* struct Response */
}; /* service Test */


namespace mind_os {
template<>
void serialize(const ::Test::Request& data, BytesBuffer& buffer) {
    buffer << data.time;
}; /* serialization of ::Test Request */

template<>
void serialize(const ::Test::Response& data, BytesBuffer& buffer) {
    serialize(data.success, buffer);
}; /* serialization of ::Test Response */

template<>
void deserialize(BytesBuffer& buffer, ::Test::Request& data) {
    buffer >> data.time;
}; /* de-serialization of ::Test Request */
template<>
void deserialize(BytesBuffer& buffer, ::Test::Response& data) {
    deserialize(buffer, data.success);
}; /* de-serialization of ::Test Response */

} /* namespace mind_os */

