#include <memory>
#include <mppi_local_planner/tools/trajectory_visualizer.hpp>

namespace mppi
{

void TrajectoryVisualizer::on_configure(
    ros::NodeHandle* nh, const std::string & name,
    const std::string & frame_id)
{

    frame_id_ = frame_id;
    nh_ = nh;
    trajectories_publisher_ =
        nh_->advertise<visualization_msgs::MarkerArray>("/trajectories", 1);
    transformed_path_pub_ = nh_->advertise<nav_msgs::Path>("/transformed_global_plan", 1);

    // TODO: get param from nh, TrajectoryVisualizer
    trajectory_step_ = 5;
    time_step_ = 3;

    reset();
}

void TrajectoryVisualizer::on_cleanup()
{
//    trajectories_publisher_.reset();
//    transformed_path_pub_.reset();
}

void TrajectoryVisualizer::on_activate()
{
//    trajectories_publisher_->on_activate();
//    transformed_path_pub_->on_activate();
}

void TrajectoryVisualizer::on_deactivate()
{
//    trajectories_publisher_->on_deactivate();
//    transformed_path_pub_->on_deactivate();
}

/**
void TrajectoryVisualizer::add(
    const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace)
{
    auto & size = trajectory.shape()[0];
    if (!size) {
        return;
    }

    auto add_marker = [&](auto i) {
        float component = static_cast<float>(i) / static_cast<float>(size);

        auto pose = utils::createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
        auto scale =
            i != size - 1 ?
                          utils::createScale(0.03, 0.03, 0.07) :
                          utils::createScale(0.07, 0.07, 0.09);
        auto color = utils::createColor(0, component, component, 1);
        auto marker = utils::createMarker(
            marker_id_++, pose, scale, color, frame_id_, marker_namespace);
        points_->markers.push_back(marker);
    };

    for (size_t i = 0; i < size; i++) {
        add_marker(i);
    }
}**/

/******************************************************************************/
void TrajectoryVisualizer::add(
    const Eigen::ArrayXXf & trajectory, const std::string & marker_namespace)
{
    int size = trajectory.rows();
    if (!(size > 0)) {
        return;
    }

    auto add_marker = [&](auto i) {
        float component = static_cast<float>(i) / static_cast<float>(size);

        auto pose = utils::createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
        auto scale =
            i != size - 1 ?
                          utils::createScale(0.03, 0.03, 0.07) :
                          utils::createScale(0.07, 0.07, 0.09);
        auto color = utils::createColor(0, component, component, 1);
        auto marker = utils::createMarker(
            marker_id_++, pose, scale, color, frame_id_, marker_namespace);
        points_->markers.push_back(marker);
    };

    for (size_t i = 0; i < size; i++) {
        add_marker(i);
    }
}
/*******************************************************************************/

void TrajectoryVisualizer::add(
    const models::Trajectories & trajectories, const std::string & marker_namespace)
{
//    auto & shape = trajectories.x.shape();
//    const float shape_1 = static_cast<float>(shape[1]);
//    points_->markers.reserve(floor(shape[0] / trajectory_step_) * floor(shape[1] * time_step_));

//    for (size_t i = 0; i < shape[0]; i += trajectory_step_) {
//        for (size_t j = 0; j < shape[1]; j += time_step_) {
//            const float j_flt = static_cast<float>(j);
//            float blue_component = 1.0f - j_flt / shape_1;
//            float green_component = j_flt / shape_1;

//            auto pose = utils::createPose(trajectories.x(i, j), trajectories.y(i, j), 0.03);
//            auto scale = utils::createScale(0.03, 0.03, 0.03);
//            auto color = utils::createColor(0, green_component, blue_component, 1);
//            auto marker = utils::createMarker(
//                marker_id_++, pose, scale, color, frame_id_, marker_namespace);

//            points_->markers.push_back(marker);
//        }
//    }

    /********************************************************************************************/
    const unsigned int rows = trajectories.arr_x.rows();
    const unsigned int cols = trajectories.arr_x.cols();
    const float shape_1 = static_cast<float>(cols);
    points_->markers.reserve(floor(rows / trajectory_step_) * floor(cols * time_step_));

    for (size_t i = 0; i < rows; i += trajectory_step_) {
        for (size_t j = 0; j < cols; j += time_step_) {
            const float j_flt = static_cast<float>(j);
            float blue_component = 1.0f - j_flt / shape_1;
            float green_component = j_flt / shape_1;

            auto pose = utils::createPose(trajectories.arr_x(i, j), trajectories.arr_y(i, j), 0.03);
            auto scale = utils::createScale(0.03, 0.03, 0.03);
            auto color = utils::createColor(0, green_component, blue_component, 1);
            auto marker = utils::createMarker(
                marker_id_++, pose, scale, color, frame_id_, marker_namespace);

            points_->markers.push_back(marker);
        }
    }
    /*******************************************************************************************/
}

void TrajectoryVisualizer::reset()
{
    marker_id_ = 0;
    points_ = std::make_unique<visualization_msgs::MarkerArray>();
}

void TrajectoryVisualizer::visualize(const nav_msgs::Path & plan)
{
    if (trajectories_publisher_.getNumSubscribers() > 0)
    {
        visualization_msgs::MarkerArray v_mark_array = *points_;

        trajectories_publisher_.publish(*points_);
    }

    reset();

    if (transformed_path_pub_.getNumSubscribers() > 0) {
        transformed_path_pub_.publish(plan);
    }
}

}  // namespace mppi
