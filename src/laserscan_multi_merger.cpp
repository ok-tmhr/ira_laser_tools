#include <chrono>
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

#include "ira_laser_tools/laserscan_multi_merger_parameter.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using LaserScan = sensor_msgs::msg::LaserScan;
using Policy2 = message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan>;
using Policy3 = message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan, LaserScan>;
using Policy4 = message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan, LaserScan, LaserScan>;
using Synchronizer2 = message_filters::Synchronizer<Policy2>;
using Synchronizer3 = message_filters::Synchronizer<Policy3>;
using Synchronizer4 = message_filters::Synchronizer<Policy4>;

class LaserscanMerger : public rclcpp::Node
{
public:
    LaserscanMerger();

private:
    void on_sync(const LaserScan::ConstSharedPtr& scan1, const LaserScan::ConstSharedPtr& scan2);
    void on_sync3(const LaserScan::ConstSharedPtr& scan1, const LaserScan::ConstSharedPtr& scan2, const LaserScan::ConstSharedPtr& scan3);
    void on_sync4(const LaserScan::ConstSharedPtr& scan1, const LaserScan::ConstSharedPtr& scan2, const LaserScan::ConstSharedPtr& scan3, const LaserScan::ConstSharedPtr& scan4);
    void publishMergedData(const std::vector<LaserScan::ConstSharedPtr>& scans);
    void laserscan_topic_parser();
    void addScanToMerged(const LaserScan::ConstSharedPtr& scan, LaserScan& output, std::vector<float>* point_cloud_points);
    void appendPointCloudPoint(std::vector<float>& points, const tf2::Vector3& point);
    void publishPointCloud(const std::vector<float>& points, const rclcpp::Time& stamp);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<LaserScan>::SharedPtr laser_scan_publisher_;

    std::vector<std::shared_ptr<message_filters::Subscriber<LaserScan>>> scan_subscribers_;
    std::vector<std::shared_ptr<tf2_ros::MessageFilter<LaserScan>>> scan_filters_;
    std::shared_ptr<Synchronizer2> sync_;
    std::shared_ptr<Synchronizer3> sync3_;
    std::shared_ptr<Synchronizer4> sync4_;
    std::shared_ptr<laserscan_multi_merger::ParamListener> param_listener_;
    laserscan_multi_merger::Params params_;
};

LaserscanMerger::LaserscanMerger() : Node("laserscan_multi_merger")
{
    param_listener_ = std::make_shared<laserscan_multi_merger::ParamListener>(get_node_parameters_interface(), get_logger());
    params_ = param_listener_->get_params();
    param_listener_->setUserCallback([this](const laserscan_multi_merger::Params& new_params) {
        params_ = new_params;
    });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->laserscan_topic_parser();

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.cloud_destination_topic, rclcpp::SensorDataQoS());
    laser_scan_publisher_ = this->create_publisher<LaserScan>(params_.scan_destination_topic, rclcpp::SensorDataQoS());
}

void LaserscanMerger::laserscan_topic_parser()
{
    const size_t num_topics = params_.laserscan_topics.size();
    RCLCPP_INFO(this->get_logger(), "Subscribing to %zu LaserScan topics with TF filtering", num_topics);

    scan_subscribers_.reserve(num_topics);
    scan_filters_.reserve(num_topics);

    for (const auto& topic : params_.laserscan_topics) {
        const auto sub = std::make_shared<message_filters::Subscriber<LaserScan>>(this, topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
        scan_subscribers_.push_back(sub);

        const auto filter = std::make_shared<tf2_ros::MessageFilter<LaserScan>>(
            *sub, *tf_buffer_, params_.destination_frame, params_.queue_size, this->get_node_logging_interface(), this->get_node_clock_interface(), std::chrono::seconds(2));
        scan_filters_.push_back(filter);

        RCLCPP_INFO(this->get_logger(), "  - %s (with TF filter to %s)", topic.c_str(), params_.destination_frame.c_str());
    }

    switch (num_topics) {
    case 2:
        sync_ = std::make_shared<Synchronizer2>(Policy2(params_.queue_size), *scan_filters_[0], *scan_filters_[1]);
        sync_->registerCallback(std::bind(&LaserscanMerger::on_sync, this, _1, _2));
        break;
    case 3:
        sync3_ = std::make_shared<Synchronizer3>(Policy3(params_.queue_size), *scan_filters_[0], *scan_filters_[1], *scan_filters_[2]);
        sync3_->registerCallback(std::bind(&LaserscanMerger::on_sync3, this, _1, _2, _3));
        break;
    case 4:
        sync4_ = std::make_shared<Synchronizer4>(Policy4(params_.queue_size), *scan_filters_[0], *scan_filters_[1], *scan_filters_[2], *scan_filters_[3]);
        sync4_->registerCallback(std::bind(&LaserscanMerger::on_sync4, this, _1, _2, _3, _4));
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "laserscan_topics must contain 1, 2, 3, or 4 topics for synchronization.");
    }
}

void LaserscanMerger::appendPointCloudPoint(std::vector<float>& points, const tf2::Vector3& point)
{
    points.push_back(static_cast<float>(point.x()));
    points.push_back(static_cast<float>(point.y()));
    points.push_back(static_cast<float>(point.z()));
}

void LaserscanMerger::on_sync(const LaserScan::ConstSharedPtr& scan1, const LaserScan::ConstSharedPtr& scan2)
{
    publishMergedData({scan1, scan2});
}

void LaserscanMerger::on_sync3(const LaserScan::ConstSharedPtr& scan1, const LaserScan::ConstSharedPtr& scan2, const LaserScan::ConstSharedPtr& scan3)
{
    publishMergedData({scan1, scan2, scan3});
}

void LaserscanMerger::on_sync4(const LaserScan::ConstSharedPtr& scan1, const LaserScan::ConstSharedPtr& scan2, const LaserScan::ConstSharedPtr& scan3, const LaserScan::ConstSharedPtr& scan4)
{
    publishMergedData({scan1, scan2, scan3, scan4});
}

void LaserscanMerger::publishMergedData(const std::vector<LaserScan::ConstSharedPtr>& scans)
{
    LaserScan output;
    output.header.frame_id = params_.destination_frame;
    output.header.stamp = this->get_clock()->now();
    output.angle_min = params_.angle_min;
    output.angle_max = params_.angle_max;
    output.angle_increment = params_.angle_increment;
    output.time_increment = params_.time_increment;
    output.scan_time = params_.scan_time;
    output.range_min = params_.range_min;
    output.range_max = params_.range_max;

    const uint32_t ranges_size = static_cast<uint32_t>(std::floor((output.angle_max - output.angle_min) / output.angle_increment)) + 1u;
    output.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    std::vector<float> cloud_points;
    std::vector<float>* cloud_points_ptr = nullptr;
    if (params_.publish_pointcloud) {
        cloud_points.reserve(1024);
        cloud_points_ptr = &cloud_points;
    }

    for (const auto& scan : scans) {
        addScanToMerged(scan, output, cloud_points_ptr);
        // Use the latest scan timestamp for the merged messages.
        if (scan && rclcpp::Time(scan->header.stamp) > output.header.stamp) {
            output.header.stamp = scan->header.stamp;
        }
    }

    laser_scan_publisher_->publish(output);
    if (params_.publish_pointcloud) {
        publishPointCloud(cloud_points, output.header.stamp);
    }
}

void LaserscanMerger::addScanToMerged(const LaserScan::ConstSharedPtr& scan, LaserScan& output, std::vector<float>* point_cloud_points)
{
    tf2::Transform transform;
    const auto tf_msg = tf_buffer_->lookupTransform(params_.destination_frame, scan->header.frame_id, scan->header.stamp);
    tf2::fromMsg(tf_msg.transform, transform);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const float range = scan->ranges[i];
        if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
            continue;
        }

        const double scan_angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
        const tf2::Vector3 point_source(
            static_cast<double>(range) * std::cos(scan_angle),
            static_cast<double>(range) * std::sin(scan_angle),
            0.0);
        const tf2::Vector3 point_dest = transform * point_source;

        const double projected_range = std::hypot(point_dest.x(), point_dest.y());
        if (projected_range < output.range_min || projected_range > output.range_max) {
            continue;
        }

        const double projected_angle = std::atan2(point_dest.y(), point_dest.x());
        if (projected_angle < output.angle_min || projected_angle > output.angle_max) {
            continue;
        }

        const int bin = static_cast<int>(std::floor((projected_angle - output.angle_min) / output.angle_increment));
        if (bin < 0 || static_cast<size_t>(bin) >= output.ranges.size()) {
            continue;
        }

        if (projected_range < output.ranges[bin]) {
            output.ranges[bin] = static_cast<float>(projected_range);
        }

        if (point_cloud_points) {
            appendPointCloudPoint(*point_cloud_points, point_dest);
        }
    }
}

void LaserscanMerger::publishPointCloud(const std::vector<float>& points, const rclcpp::Time& stamp)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = params_.destination_frame;
    cloud_msg.header.stamp = stamp;
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(points.size() / 3);
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;
    cloud_msg.point_step = static_cast<uint32_t>(3 * sizeof(float));
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    auto make_field = [](const std::string& name, uint32_t offset) {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        return field;
    };

    cloud_msg.fields = {
        make_field("x", 0),
        make_field("y", sizeof(float)),
        make_field("z", 2 * sizeof(float))};

    cloud_msg.data.resize(points.size() * sizeof(float));
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (size_t i = 0; i + 2 < points.size(); i += 3) {
        *iter_x = points[i];
        *iter_y = points[i + 1];
        *iter_z = points[i + 2];
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    point_cloud_publisher_->publish(cloud_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserscanMerger>());
    rclcpp::shutdown();
    return 0;
}
