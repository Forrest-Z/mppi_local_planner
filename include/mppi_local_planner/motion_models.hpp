#ifndef MPPI_LOCAL_PLANNER_MOTION_MODELS_HPP
#define MPPI_LOCAL_PLANNER_MOTION_MODELS_HPP

#include <cstdint>

#include <mppi_local_planner/models/control_sequence.hpp>
#include <mppi_local_planner/models/state.hpp>

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
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
    virtual void predict(models::State & state)
    {
        const bool is_holo = isHolonomic();

        /**
        for (unsigned int i = 0; i != state.vx.shape(0); i++) {
            for (unsigned int j = 1; j != state.vx.shape(1); j++) {
                state.vx(i, j) = state.cvx(i, j - 1);
                state.wz(i, j) = state.cwz(i, j - 1);
                if (is_holo) {
                    state.vy(i, j) = state.cvy(i, j - 1);
                }
            }
        }**/


        for (unsigned int i = 0; i != state.arr_vx.rows(); ++i)
        {
            for (unsigned int j = 1; j != state.arr_vx.cols(); ++j)
            {
                state.arr_vx(i, j) = state.arr_cvx(i, j-1);
                state.arr_wz(i, j) = state.arr_cwz(i, j-1);
                if (is_holo)
                {
                    state.arr_vy(i, j) = state.arr_cvy(i, j-1);
                }
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
    explicit AckermannMotionModel(ros::NodeHandle* nh)
    {
        // TODO: getParam
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
        /**
        auto & vx = control_sequence.vx;
        auto & wz = control_sequence.wz;

        auto view = xt::masked_view(wz, xt::fabs(vx) / xt::fabs(wz) < min_turning_r_);
        view = xt::sign(wz) * vx / min_turning_r_;**/
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

    void applyConstraints(models::ControlSequence & control_sequence) override
    {
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

#endif // MPPI_LOCAL_PLANNER_MOTION_MODELS_HPP
