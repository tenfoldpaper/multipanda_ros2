#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <franka/model.h>
#include <franka/robot_state.h>

#include "franka_hardware/model.hpp"
#include "franka_hardware/model_base.hpp"

#include "semantic_components/semantic_component_interface.hpp"

namespace franka_semantic_components {
class FrankaRobotModel : public semantic_components::SemanticComponentInterface<franka_hardware::Model> {
    public:
    /**
     * Creates an instance of a FrankaRobotModel.
     * @param[in] name The name of robot model state interface.
     */
    FrankaRobotModel(const std::string& franka_model_interface_name,
                    const std::string& franka_state_interface_name);
    FrankaRobotModel() = delete;

    virtual ~FrankaRobotModel() = default;

    /**
     * Gets the 4x4 pose matrix for the given frame in base frame, calculated from the current
     * robot state.
     *
     * The pose is represented as a 4x4 matrix in column-major format.
     *
     * @param[in] frame The desired frame.
     *
     * @return Vectorized 4x4 pose matrix, column-major.
     *
     * @see franka::Model::pose
     */
    std::array<double, 16> getPose(const franka::Frame& frame) const {
        // probably due to namespace... 
        return robot_model->ModelBase::pose(frame, *robot_state);
    }

    /**
     * Gets the 6x7 Jacobian for the given frame, relative to that frame.
     *
     * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
     * the current robot state.
     *
     * @param[in] frame The desired frame.
     *
     * @return Vectorized 6x7 Jacobian, column-major.
     *
     * @see franka::Model::bodyJacobian
     */
    std::array<double, 42> getBodyJacobian(const franka::Frame& frame) const {
        return robot_model->ModelBase::bodyJacobian(frame, *robot_state);
    }

    /**
     * Gets the 6x7 Jacobian for the given joint relative to the base frame.
     *
     * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
     * the current robot state.
     *
     * @param[in] frame The desired frame.
     *
     * @return Vectorized 6x7 Jacobian, column-major.
     *
     * @see franka::Model::zeroJacobian
     */
    std::array<double, 42> getZeroJacobian(const franka::Frame& frame) const {
        return robot_model->ModelBase::zeroJacobian(frame, *robot_state);
    }

    /**
     * Calculates the 7x7 mass matrix from the current robot state. Unit: \f$[kg \times m^2]\f$.
     *
     * @return Vectorized 7x7 mass matrix, column-major.
     *
     * @see franka::Model::mass
     */
    std::array<double, 49> getMass() const { return robot_model->ModelBase::mass(*robot_state); }

    /**
     * Calculates the Coriolis force vector (state-space equation) from the current robot state:
     * \f$ c= C \times dq\f$, in \f$[Nm]\f$.
     *
     * @return Coriolis force vector.
     *
     * @see franka::Model::coriolis
     */
    std::array<double, 7> getCoriolis() const { return robot_model->ModelBase::coriolis(*robot_state); }

    /**
     * Calculates the gravity vector from the current robot state. Unit: \f$[Nm]\f$.
     *
     * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
     * Default to {0.0, 0.0, -9.81}.
     *
     * @return Gravity vector.
     *
     * @see franka::Model::gravity
     */
    std::array<double, 7> getGravity(const std::array<double, 3>& gravity_earth = {
                                        {0., 0., -9.81}}) const {
        return robot_model->ModelBase::gravity(*robot_state, gravity_earth);
    }

    // TODO: Add the override methods

    protected:
    /**
     * Retrieve the robot state and robot model pointers from the hardware state interface
     *
     * @throws Runtime error when state interfaces are not available.
     */
    void initialize();

    bool initialized{false};
    franka_hardware::Model* robot_model;
    franka::RobotState* robot_state;

    private:
    const std::string arm_id_{"panda"};

    const std::string robot_state_interface_name_{"robot_state"};
    const std::string robot_model_interface_name_{"robot_model"};
};
}