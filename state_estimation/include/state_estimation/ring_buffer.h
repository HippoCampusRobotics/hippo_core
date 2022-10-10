#pragma once
#include <stdint.h>
#include <state_estimation/common.h>
#include <cstddef>

template <typename T>
class RingBuffer {
 public:
  explicit RingBuffer(std::size_t size) { Allocate(size); }
  RingBuffer() { Allocate(1); }
  ~RingBuffer() { delete[] buffer_; }
  RingBuffer(const RingBuffer &) = delete;
  RingBuffer &operator=(const RingBuffer &) = delete;
  RingBuffer(RingBuffer &&) = delete;
  RingBuffer &operator=(RingBuffer &&) = delete;

  bool Valid() const { return (buffer_ != nullptr) && (size_ > 0); }
  bool Allocate(int size) {
    // everything stays the same
    if (Valid() && (size == size_)) {
      return true;
    }

    // do not allow zero size buffer
    if (size == 0) {
      return false;
    }

    if (buffer_ != nullptr) {
      delete[] buffer_;
    }
    buffer_ = new T[size]{};
    if (buffer_ == nullptr) {
      return false;
    }
    size_ = size;
    head_ = 0;
    tail_ = 0;
    first_write_ = true;
    return true;
  }

  void Push(const T &sample) {
    if (!first_write_) {
      ++head_;
      if (head_ >= size_) {
        head_ = 0;
      }
    }
    buffer_[head_] = sample;
    if ((head_ == tail_) && !first_write_) {
      ++tail_;
      if (tail_ >= size_) {
        tail_ = 0;
      }
    } else {
      first_write_ = false;
    }
  }

  int Length() const { return size_; }

  T &operator[](const int index) { return buffer_[index]; }

  const T &Newest() const { return buffer_[head_]; }
  const T &Oldest() const { return buffer_[tail_]; }
  int OldestIndex() const { return tail_; }
  int NewestIndex() const { return head_; }

  bool PopFirstOlderThan(const uint64_t &timestamp_us, T *sample) {
    for (int i = 0; i < size_; ++i) {
      int index = head_ - i;

      // handle the wrap around
      index = index < 0 ? (index + size_) : index;

      if ((timestamp_us >= buffer_[index].time_us) &&
          (timestamp_us < buffer_[index].time_us + (uint64_t)1e6)) {
        *sample = buffer_[index];
        if (index == head_) {
          tail_ = head_;
          first_write_ = true;
        } else {
          // remove all older items (i.e. behin current index)
          tail_ = index + 1;
          if (tail_ >= size_) {
            tail_ = 0;
          }
        }

        buffer_[index].time_us = 0;
        return true;
      }

      // we have no match
      if (index == tail_) {
        return false;
      }
    }
    // only reason to reach this line is a zero buffer size
    return false;
  }

 private:
  T *buffer_{nullptr};
  int head_{0};
  int tail_{0};
  int size_{0};
  bool first_write_{true};
};
