// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>
#include <string>

#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
   * @brief Set velocity/acceleration constraints used during rollout prediction
   */
  void setConstraints(const models::ControlConstraints & constraints, float model_dt)
  {
    constraints_ = constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    using namespace xt::placeholders;  // NOLINT

    if (model_dt_ <= 0.0f) {
      xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
        xt::view(state.cvx, xt::all(), xt::range(0, -1));

      xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
        xt::view(state.cwz, xt::all(), xt::range(0, -1));

      if (isHolonomic()) {
        xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
          xt::view(state.cvy, xt::all(), xt::range(0, -1));
      }
      return;
    }

    const float max_delta_vx = model_dt_ * constraints_.ax_max;
    const float min_delta_vx = model_dt_ * constraints_.ax_min;
    const float max_delta_vy = model_dt_ * constraints_.ay_max;
    const float min_delta_vy = model_dt_ * constraints_.ay_min;
    const float max_delta_wz = model_dt_ * constraints_.az_max;
    const bool holonomic = isHolonomic();
    const size_t n_cols = state.vx.shape()[1];

    for (size_t i = 1; i < n_cols; ++i) {
      const auto vx_prev = xt::view(state.vx, xt::all(), i - 1);
      const auto lower_vx = xt::where(
        vx_prev > 0.0f, vx_prev + min_delta_vx, vx_prev - max_delta_vx);
      const auto upper_vx = xt::where(
        vx_prev > 0.0f, vx_prev + max_delta_vx, vx_prev - min_delta_vx);
      xt::noalias(xt::view(state.vx, xt::all(), i)) = xt::clip(
        xt::view(state.cvx, xt::all(), i - 1), lower_vx, upper_vx);

      const auto wz_prev = xt::view(state.wz, xt::all(), i - 1);
      xt::noalias(xt::view(state.wz, xt::all(), i)) = xt::clip(
        xt::view(state.cwz, xt::all(), i - 1),
        wz_prev - max_delta_wz, wz_prev + max_delta_wz);

      if (holonomic) {
        const auto vy_prev = xt::view(state.vy, xt::all(), i - 1);
        const auto lower_vy = xt::where(
          vy_prev > 0.0f, vy_prev + min_delta_vy, vy_prev - max_delta_vy);
        const auto upper_vy = xt::where(
          vy_prev > 0.0f, vy_prev + max_delta_vy, vy_prev - min_delta_vy);
        xt::noalias(xt::view(state.vy, xt::all(), i)) = xt::clip(
          xt::view(state.cvy, xt::all(), i - 1), lower_vy, upper_vy);
      }
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}

protected:
  models::ControlConstraints constraints_{};
  float model_dt_{0.0f};
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;

    auto view = xt::masked_view(wz, (xt::fabs(vx) / xt::fabs(wz)) < min_turning_r_);
    view = xt::sign(wz) * xt::fabs(vx) / min_turning_r_;
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
