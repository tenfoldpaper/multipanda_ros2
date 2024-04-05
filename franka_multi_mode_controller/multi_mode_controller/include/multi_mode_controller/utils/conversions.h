#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace {

void pushBackResources(std::vector<std::string>& out, const std::string& res) {
  if (res.empty()) {
    return;
  } else {
    size_t end_of_first_resource = res.find("&");
    if (end_of_first_resource == std::string::npos) {
      out.push_back(res);
      return;
    } else {
      out.push_back(res.substr(0, end_of_first_resource));
      return pushBackResources(out, res.substr(end_of_first_resource+1));
    }
  }
}

} // end of unnamed namespace

inline std::string vectorToResource(const std::vector<std::string>& vec) {
  if (vec.empty()) {
    return "";
  }
  std::stringstream ss;
  ss << vec[0];
  for (int i=1;i<vec.size();++i) {
    ss << "&" << vec[i];
  }
  return ss.str();
}

inline std::vector<std::string> resourceToVector(const std::string& res) {
  std::vector<std::string> out;
  pushBackResources(out, res);
  return out;
}
