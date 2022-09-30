/*
 * MessageItem.h
 *
 *  Created on: Oct 15, 2021
 *      Author: ubuntu
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace base_msgs {

template <typename T>
class Data
{
private:
    T d;
    bool enable;
public:
    Data()
    {
        d = 0;
        enable = false;
    }

    Data& operator=(T data)
    {
        this->d = data;
        enable = true;
        return *this;
    }

    Data& operator=(const Data<T>& data)
    {
        if (data.enable)
        {
            this->d = data.d;
            enable = true;
        }
        return *this;
    }

    bool operator==(Data<T>& v)
    {
        return d == v.d;
    }

    bool operator!=(Data<T>& v)
    {
        return d != v.d;
    }

    explicit operator bool() const noexcept
    {
        return enable;
    }

    std::uint16_t size() const
    {
        return sizeof(T);
    }

    T data() const
    {
        return d;
    }

    const T* raw() const
    {
        return &d;
    }
};

template <>
class Data<std::string>
{
private:
    std::string d;
    bool enable = false;
public:
    Data& operator=(std::string& data)
    {
        this->d = data;
        enable = true;
        return *this;
    }

    Data& operator=(int data)
    {
        this->d = std::to_string(data);
        enable = true;
        return *this;
    }

    Data& operator=(const char* data)
    {
        this->d = std::string(data);
        enable = true;
        return *this;
    }

    Data& operator=(const Data<std::string>& data)
    {
        if (data.enable)
        {
            this->d = data.d;
            enable = true;
        }
        return *this;
    }

    bool operator==(Data<std::string>& v)
    {
        return d.compare(v.d) == 0;
    }

    bool operator!=(Data<std::string>& v)
    {
        return d.compare(v.d) != 0;
    }

    explicit operator bool() const noexcept
    {
        return enable;
    }

    std::uint16_t size() const
    {
        return d.size();
    }

    std::string data() const
    {
        return d;
    }

    const char* raw() const
    {
        return d.data();
    }
};

}
