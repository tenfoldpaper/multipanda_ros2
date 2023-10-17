#pragma once

#include <ostream>
#include <type_traits>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>

// A dump of functions that are used by both hardware and multi_hardware interfaces.
namespace franka_hardware {
bool all_of_element_has_string(std::vector<std::string> vec, std::string content);
int check_command_mode_type(std::vector<std::string> interfaces);
}