#pragma once

#include <mutex>

namespace panda_controllers {

template<typename T>
class Buffered {
 public:
  Buffered() = default;
  Buffered(const T& t) {
    content_ = t;
  }
  T get() {
    std::lock_guard<std::recursive_mutex> lock(buf_mutex_);
    return buf_;
  }
  void setBuffer(const T& t) {
    std::lock_guard<std::recursive_mutex> lock(buf_mutex_);
    needs_update_ = true;
    buf_ = t;
  }
  void update() {
    std::lock_guard<std::recursive_mutex> lock(buf_mutex_);
    if (needs_update_) {
      needs_update_ = false;
      content_ = buf_;
    }
  }
  bool updatePending() {
    std::lock_guard<std::recursive_mutex> lock(buf_mutex_);
    return needs_update_;
  }
  operator const T&() {
    return content_;
  }
  const T* const operator->() {
    return &content_;
  }
 private:
  std::recursive_mutex buf_mutex_;
  T buf_, content_;
  bool needs_update_ = false;
};

template<typename T>
class Buffered<T*> {
 public:
  Buffered() = default;
  Buffered(T* const& t) {
    content_ = t;
  }
  T* get() {
    std::lock_guard<std::recursive_mutex> lock(buf_mutex_);
    return buf_;
  }
  void setBuffer(T* const& t) {
    std::lock_guard<std::recursive_mutex> lock(buf_mutex_);
    needs_update_ = true;
    buf_ = t;
  }
  void update() {
    std::lock_guard<std::recursive_mutex> lock(buf_mutex_);
    if (needs_update_) {
      needs_update_ = false;
      content_ = buf_;
    }
  }
  bool updatePending() {
    std::lock_guard<std::recursive_mutex> lock(buf_mutex_);
    return needs_update_;
  }
  operator T* const&() {
    return content_;
  }
  T* const operator->() {
    return content_;
  }
 private:
  std::recursive_mutex buf_mutex_;
  T* buf_;
  T* content_;
  bool needs_update_ = false;
};

}
