#pragma once

#include <multi_mode_controller/base/panda_controller_interface.h>
#include <multi_mode_controller/utils/factory.h>

namespace panda_controllers {

using ControllerFactory = Factory<PandaControllerInterface>;

} // namespace panda_controllers
