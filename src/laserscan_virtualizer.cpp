#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <string>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "ira_laser_tools/laserscan_virtualizer_parameter.hpp"

using std::placeholders::_1;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

namespace ira_laser_tools
{
class LaserscanVirtualizer : public rclcpp::Node
{
public:
    LaserscanVirtualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void pointcloud_to_laserscan(
        const PointCloud2::ConstSharedPtr& pcl_in,
        const tf2::Transform& transform,
        const std::string& output_frame,
        int pub_index);
    void pointCloudCallback(const PointCloud2::ConstSharedPtr& pcl_in);

private:
    std::shared_ptr<laserscan_virtualizer::ParamListener> param_listener_;
    laserscan_virtualizer::Params params_;

    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::vector<tf2::Transform> transform_;

    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> virtual_scan_publishers;

    std::shared_ptr<message_filters::Subscriber<PointCloud2>> cloud_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<PointCloud2>> filter_;
    void virtual_laser_scan_parser();

    std::string cloud_frame_;
};

void LaserscanVirtualizer::virtual_laser_scan_parser()
{
    const auto num_topics = params_.virtual_laser_scan.size();

    virtual_scan_publishers.clear();
    filter_.reset();

    virtual_scan_publishers.reserve(num_topics);

    cloud_sub_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(
        this, params_.cloud_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    filter_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
        *cloud_sub_, *tf_buffer_, params_.base_frame, 10,
        this->get_node_logging_interface(), this->get_node_clock_interface(), std::chrono::seconds(1));
    filter_->registerCallback(std::bind(&LaserscanVirtualizer::pointCloudCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Publishing: %ld virtual scans", params_.virtual_laser_scan.size());
    if (params_.output_laser_topic.empty()) {
        for (const auto& t : params_.virtual_laser_scan) {
            const auto pub = this->create_publisher<sensor_msgs::msg::LaserScan>(t, rclcpp::SensorDataQoS());
            virtual_scan_publishers.push_back(pub);
            RCLCPP_INFO(this->get_logger(), "%s on topic %s", t.c_str(), pub->get_topic_name());
        }
    } else {
        for (const auto& t : params_.virtual_laser_scan) {
            const auto pub = this->create_publisher<sensor_msgs::msg::LaserScan>(params_.output_laser_topic, rclcpp::SensorDataQoS());
            virtual_scan_publishers.push_back(
                pub);
            RCLCPP_INFO(this->get_logger(), "%s on topic %s", t.c_str(), pub->get_topic_name());
        }
    }
}

LaserscanVirtualizer::LaserscanVirtualizer(const rclcpp::NodeOptions& options) : Node("laserscan_virtualizer", options)
{
    param_listener_ = std::make_shared<laserscan_virtualizer::ParamListener>(get_node_parameters_interface(), get_logger());

    param_listener_->setUserCallback([this](const laserscan_virtualizer::Params& new_params) {
        params_ = new_params;
        // Re-parse virtual laser scan if changed
        this->virtual_laser_scan_parser();
    });

    params_ = param_listener_->get_params();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface()));
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->virtual_laser_scan_parser();

    cloud_frame_ = "";
}

void LaserscanVirtualizer::pointCloudCallback(const PointCloud2::ConstSharedPtr& pcl_in)
{
    const auto& output_frames = params_.virtual_laser_scan;
    if (cloud_frame_.empty()) {
        cloud_frame_ = pcl_in->header.frame_id;
        transform_.resize(output_frames.size());
        for (size_t i = 0; i < output_frames.size(); i++) {
            const auto tfGeom = tf_buffer_->lookupTransform(output_frames[i], cloud_frame_, pcl_in->header.stamp);
            tf2::fromMsg(tfGeom.transform, transform_[i]);
        }
    }

    for (size_t i = 0; i < output_frames.size(); i++) {
        pointcloud_to_laserscan(pcl_in, transform_[i], output_frames[i], i);
    }
}

void LaserscanVirtualizer::pointcloud_to_laserscan(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl_in,
    const tf2::Transform& transform,
    const std::string& output_frame,
    int pub_index)
{
    sensor_msgs::msg::LaserScan output;
    output.header = pcl_in->header;
    output.header.frame_id = output_frame;
    output.angle_min = params_.angle_min;
    output.angle_max = params_.angle_max;
    output.angle_increment = params_.angle_increment;
    output.time_increment = 0.0; // params_.time_increment; // Not used in original
    output.scan_time = params_.scan_time;
    output.range_min = params_.range_min;
    output.range_max = params_.range_max;

    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(ranges_size, output.range_max + 1.0);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pcl_in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pcl_in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pcl_in, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;

        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
            RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        tf2::Vector3 point(x, y, z);
        tf2::Vector3 transformed = transform * point;
        double tx = transformed.x();
        double ty = transformed.y();
        double tz = transformed.z();

        double range_sq = tx * tx + ty * ty;
        double range_min_sq_ = output.range_min * output.range_min;
        if (range_sq < range_min_sq_) {
            RCLCPP_DEBUG(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, tx, ty, tz);
            continue;
        }

        double angle = atan2(ty, tx);
        if (angle < output.angle_min || angle > output.angle_max) {
            RCLCPP_DEBUG(this->get_logger(), "rejected for angle %f not in range (%f, %f)", angle, output.angle_min, output.angle_max);
            continue;
        }

        int index = static_cast<int>((angle - output.angle_min) / output.angle_increment);
        if (index < 0 || static_cast<uint32_t>(index) >= ranges_size) {
            continue;
        }

        double existing_range_sq = output.ranges[index] * output.ranges[index];
        if (existing_range_sq > range_sq) {
            output.ranges[index] = std::sqrt(range_sq);
        }
    }

    virtual_scan_publishers[pub_index]->publish(output);
}
} // namespace ira_laser_tools
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ira_laser_tools::LaserscanVirtualizer)