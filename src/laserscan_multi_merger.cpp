#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

class LaserscanMerger : public rclcpp::Node
{
public:
    LaserscanMerger();

private:
    void scanFilterCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, size_t index);
    void publishMergedData();
    rcl_interfaces::msg::SetParametersResult reconfigureCallback(const std::vector<rclcpp::Parameter>& parameters);
    void laserscan_topic_parser();
    bool computeTransform(const std::string& source_frame, const rclcpp::Time& stamp, tf2::Transform& out_transform);
    void appendPointCloudPoint(std::vector<float>& points, const tf2::Vector3& point);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;

    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>> scan_subscribers_;
    std::vector<std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>> scan_filters_;
    std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> filtered_scans_;
    std::vector<bool> scan_ready_;
    std::vector<std::string> input_topics_;

    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double time_increment_;
    double scan_time_;
    double range_min_;
    double range_max_;

    std::string destination_frame_;
    std::string cloud_destination_topic_;
    std::string scan_destination_topic_;
    std::string laserscan_topics_;
};

LaserscanMerger::LaserscanMerger() : Node("laserscan_multi_merger")
{
    this->declare_parameter<std::string>("destination_frame", "cart_frame");
    this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
    this->declare_parameter<std::string>("scan_destination_topic", "/scan_multi");
    this->declare_parameter<std::string>("laserscan_topics", "");
    this->declare_parameter("angle_min", -3.14);
    this->declare_parameter("angle_max", 3.14);
    this->declare_parameter("angle_increment", 0.0058);
    this->declare_parameter("time_increment", 0.0);
    this->declare_parameter("scan_time", 0.0);
    this->declare_parameter("range_min", 0.0);
    this->declare_parameter("range_max", 25.0);

    this->get_parameter("destination_frame", destination_frame_);
    this->get_parameter("cloud_destination_topic", cloud_destination_topic_);
    this->get_parameter("scan_destination_topic", scan_destination_topic_);
    this->get_parameter("laserscan_topics", laserscan_topics_);
    this->get_parameter("angle_min", angle_min_);
    this->get_parameter("angle_max", angle_max_);
    this->get_parameter("angle_increment", angle_increment_);
    this->get_parameter("time_increment", time_increment_);
    this->get_parameter("scan_time", scan_time_);
    this->get_parameter("range_min", range_min_);
    this->get_parameter("range_max", range_max_);

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LaserscanMerger::reconfigureCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->laserscan_topic_parser();

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic_, rclcpp::SensorDataQoS());
    laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_destination_topic_, rclcpp::SensorDataQoS());
}

rcl_interfaces::msg::SetParametersResult LaserscanMerger::reconfigureCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& parameter : parameters) {
        const auto& type = parameter.get_type();
        const auto& name = parameter.get_name();

        if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
            double value = parameter.as_double();
            if (name == "angle_min") {
                angle_min_ = value;
            } else if (name == "angle_max") {
                angle_max_ = value;
            } else if (name == "angle_increment") {
                angle_increment_ = value;
            } else if (name == "time_increment") {
                time_increment_ = value;
            } else if (name == "scan_time") {
                scan_time_ = value;
            } else if (name == "range_min") {
                range_min_ = value;
            } else if (name == "range_max") {
                range_max_ = value;
            }
        }
    }

    return result;
}

void LaserscanMerger::laserscan_topic_parser()
{
    std::istringstream iss(laserscan_topics_);
    std::set<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::inserter(tokens, tokens.begin()));

    std::vector<std::string> tmp_input_topics(tokens.begin(), tokens.end());
    std::sort(tmp_input_topics.begin(), tmp_input_topics.end());
    tmp_input_topics.erase(std::unique(tmp_input_topics.begin(), tmp_input_topics.end()), tmp_input_topics.end());

    if (tmp_input_topics == input_topics_) {
        return;
    }

    input_topics_ = std::move(tmp_input_topics);
    filtered_scans_.assign(input_topics_.size(), nullptr);
    scan_ready_.assign(input_topics_.size(), false);
    scan_subscribers_.clear();
    scan_filters_.clear();

    if (input_topics_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No LaserScan topics configured for subscription.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Subscribing to %zu LaserScan topics with TF filtering", input_topics_.size());
    scan_subscribers_.reserve(input_topics_.size());
    scan_filters_.reserve(input_topics_.size());

    for (size_t i = 0; i < input_topics_.size(); ++i) {
        const std::string topic = input_topics_[i];

        auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
        scan_subscribers_.push_back(sub);

        auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
            *sub, *tf_buffer_, destination_frame_, 10, this->get_node_logging_interface(), this->get_clock(), rclcpp::Duration::from_seconds(2.0));

        auto callback = [this, i](const sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
            this->scanFilterCallback(scan, i);
        };
        filter->registerCallback(std::move(callback));

        scan_filters_.push_back(filter);
        RCLCPP_INFO(this->get_logger(), "  - %s (with TF filter to %s)", topic.c_str(), destination_frame_.c_str());
    }
}

bool LaserscanMerger::computeTransform(const std::string& source_frame, const rclcpp::Time& stamp, tf2::Transform& out_transform)
{
    try {
        auto tf_msg = tf_buffer_->lookupTransform(destination_frame_, source_frame, stamp, rclcpp::Duration::from_seconds(0.5));
        tf2::Quaternion quat(
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w);
        tf2::Vector3 trans(
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z);
        out_transform.setOrigin(trans);
        out_transform.setRotation(quat);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed for %s -> %s: %s",
                    source_frame.c_str(), destination_frame_.c_str(), ex.what());
        return false;
    }
}

void LaserscanMerger::appendPointCloudPoint(std::vector<float>& points, const tf2::Vector3& point)
{
    points.push_back(static_cast<float>(point.x()));
    points.push_back(static_cast<float>(point.y()));
    points.push_back(static_cast<float>(point.z()));
}

void LaserscanMerger::scanFilterCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, size_t index)
{
    if (index >= filtered_scans_.size()) {
        return;
    }

    filtered_scans_[index] = scan;
    scan_ready_[index] = true;

    bool all_ready = true;
    for (bool ready : scan_ready_) {
        if (!ready) {
            all_ready = false;
            break;
        }
    }

    if (!all_ready) {
        return;
    }

    publishMergedData();
    std::fill(scan_ready_.begin(), scan_ready_.end(), false);
}

void LaserscanMerger::publishMergedData()
{
    if (filtered_scans_.empty()) {
        return;
    }

    sensor_msgs::msg::LaserScan output;
    output.header.frame_id = destination_frame_;
    output.header.stamp = this->get_clock()->now();
    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = time_increment_;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    const uint32_t ranges_size = static_cast<uint32_t>(std::floor((output.angle_max - output.angle_min) / output.angle_increment)) + 1u;
    output.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    std::vector<float> cloud_points;
    cloud_points.reserve(1024);

    for (const auto& scan : filtered_scans_) {
        if (!scan) {
            continue;
        }

        tf2::Transform transform;
        if (!computeTransform(scan->header.frame_id, scan->header.stamp, transform)) {
            continue;
        }

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            const float range = scan->ranges[i];
            if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
                continue;
            }

            const double scan_angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
            tf2::Vector3 point_source(
                static_cast<double>(range) * std::cos(scan_angle),
                static_cast<double>(range) * std::sin(scan_angle),
                0.0);
            const tf2::Vector3 point_dest = transform * point_source;

            const double projected_range = std::hypot(point_dest.x(), point_dest.y());
            if (projected_range < range_min_ || projected_range > range_max_) {
                continue;
            }

            const double projected_angle = std::atan2(point_dest.y(), point_dest.x());
            if (projected_angle < angle_min_ || projected_angle > angle_max_) {
                continue;
            }

            const int bin = static_cast<int>((projected_angle - angle_min_) / angle_increment_);
            if (bin < 0 || static_cast<size_t>(bin) >= ranges_size) {
                continue;
            }

            if (projected_range < output.ranges[bin]) {
                output.ranges[bin] = static_cast<float>(projected_range);
            }
            appendPointCloudPoint(cloud_points, point_dest);
        }
    }

    // Use the latest scan timestamp for the merged messages.
    for (const auto& scan : filtered_scans_) {
        if (scan && scan->header.stamp > output.header.stamp) {
            output.header.stamp = scan->header.stamp;
        }
    }

    laser_scan_publisher_->publish(output);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = destination_frame_;
    cloud_msg.header.stamp = output.header.stamp;
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(cloud_points.size() / 3);
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;
    cloud_msg.point_step = static_cast<uint32_t>(3 * sizeof(float));
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = sizeof(float);
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 2 * sizeof(float);
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.data.resize(cloud_points.size() * sizeof(float));
    std::memcpy(cloud_msg.data.data(), cloud_points.data(), cloud_msg.data.size());

    point_cloud_publisher_->publish(cloud_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserscanMerger>());
    rclcpp::shutdown();
    return 0;
}
