#pragma once
#include <vector>
#include <algorithm>

template <typename T>
class LinearBuffer {
public:
    LinearBuffer() : capacity_(10){}

    LinearBuffer(size_t capacity) : capacity_(capacity){
        buffer_.reserve(capacity_);
    }

    void set_capacity(size_t capacity){
        this->capacity_ = capacity;
        buffer_.reserve(capacity_);
    }

    // Push an element to the back of the circular buffer
    void push_back(const T& value) {
        if (size_ < capacity_) {
            // If there's space available, add the element at the tail.
            buffer_.push_back(value);
            size_++;
        } else {
            // If the buffer is full, overwrite the first element and rotate.
            buffer_[0] = value;
            std::rotate(buffer_.begin(), buffer_.begin() + 1, buffer_.end());
        }
    }

    std::vector<T> buffer_;
private:

    size_t capacity_;
    size_t size_ = 0;
};