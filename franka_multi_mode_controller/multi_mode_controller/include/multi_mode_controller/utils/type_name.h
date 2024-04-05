#pragma once

#include <cxxabi.h>
#include <string>

namespace panda_controllers {

template <class Type>
static std::string typeName() {
  int status = -1;
  char* name = abi::__cxa_demangle(typeid(Type).name(), nullptr, nullptr,
                                   &status);
  std::string result = (name ? name : typeid(Type).name());
  free(name);
  return result;
}

template <class Object>
static std::string typeName(const Object& o) {
  int status = -1;
  char* name = abi::__cxa_demangle(typeid(o).name(), nullptr, nullptr,
                                   &status);
  std::string result = (name ? name : typeid(o).name());
  free(name);
  return result;
}

} // end of namespace panda_controllers
