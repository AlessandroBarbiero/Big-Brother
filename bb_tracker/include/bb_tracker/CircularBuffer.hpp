#pragma once
#include <vector>

template <typename T>
class CircularBuffer {
public:
    CircularBuffer() : capacity_(10){}

    CircularBuffer(size_t capacity) : capacity_(capacity){
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
            // If the buffer is full, overwrite the oldest element.
            buffer_[head_] = value;
            head_ = (head_ + 1) % capacity_;
        }
    }

    std::vector<T> buffer_;
private:

    size_t capacity_;
    size_t head_ = 0;  // Start with head at position 0
    size_t size_ = 0;
};