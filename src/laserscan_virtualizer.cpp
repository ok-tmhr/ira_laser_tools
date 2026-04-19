#include "pcl_ros/transforms.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "ira_laser_tools/laserscan_virtualizer_parameter.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> myPointCloud;

using namespace std;
using namespace pcl;

class LaserscanVirtualizer : public rclcpp::Node
{
public:
    LaserscanVirtualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index);
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr pcl_in);

private:
    std::shared_ptr<laserscan_virtualizer::ParamListener> param_listener_;
    laserscan_virtualizer::Params params_;

    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::vector<tf2::Stamped<tf2::Transform>> transform_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> virtual_scan_publishers;
    std::vector<string> output_frames;

    void virtual_laser_scan_parser();

    string cloud_frame;
};

void LaserscanVirtualizer::virtual_laser_scan_parser()
{
    // LaserScan frames to use for virtualization
    istringstream iss(params_.virtual_laser_scan);
    std::vector<string> tokens;
    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<std::vector<string>>(tokens));

    std::vector<string> tmp_output_frames;

    for (std::vector<int>::size_type i = 0; i < tokens.size(); i++) {
        auto beg = this->get_clock()->now();
        if (tf_buffer_->canTransform(params_.base_frame, tokens[i], rclcpp::Time(0), rclcpp::Duration(1, 0))) // Check if TF knows the transform from this frame reference to base_frame reference
        {
            cout << "Elapsed: " << (this->get_clock()->now() - beg).nanoseconds() / 1e9 << endl;
            cout << "Adding: " << tokens[i] << endl;
            tmp_output_frames.push_back(tokens[i]);
        } else {
            cout << "Can't transform: '" << tokens[i] + "' to '" << params_.base_frame << "'" << endl;
        }
    }

    // Sort and remove duplicates
    sort(tmp_output_frames.begin(), tmp_output_frames.end());
    std::vector<string>::iterator last = std::unique(tmp_output_frames.begin(), tmp_output_frames.end());
    tmp_output_frames.erase(last, tmp_output_frames.end());

    // Do not re-advertize if the topics are the same
    if ((tmp_output_frames.size() != output_frames.size()) || !equal(tmp_output_frames.begin(), tmp_output_frames.end(), output_frames.begin())) {
        cloud_frame = "";

        output_frames = tmp_output_frames;
        if (output_frames.size() > 0) {
            virtual_scan_publishers.resize(output_frames.size());
            RCLCPP_INFO(this->get_logger(), "Publishing: %ld virtual scans", virtual_scan_publishers.size());
            cout << "Advertising topics: " << endl;
            for (std::vector<int>::size_type i = 0; i < output_frames.size(); ++i) {
                if (params_.output_laser_topic.empty()) {
                    virtual_scan_publishers[i] = this->create_publisher<sensor_msgs::msg::LaserScan>(output_frames[i].c_str(), rclcpp::SensorDataQoS());
                    cout << "\t\t" << output_frames[i] << " on topic " << output_frames[i].c_str() << endl;
                } else {
                    virtual_scan_publishers[i] = this->create_publisher<sensor_msgs::msg::LaserScan>(params_.output_laser_topic.c_str(), rclcpp::SensorDataQoS());
                    cout << "\t\t" << output_frames[i] << " on topic " << params_.output_laser_topic.c_str() << endl;
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Not publishing to any topic.");
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
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->virtual_laser_scan_parser();

    point_cloud_subscription_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(params_.cloud_topic.c_str(), rclcpp::SensorDataQoS(), std::bind(&LaserscanVirtualizer::pointCloudCallback, this, std::placeholders::_1));
    cloud_frame = "";
}

void LaserscanVirtualizer::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr pcl_in)
{
    if (cloud_frame.empty()) {
        cloud_frame = (*pcl_in).header.frame_id;
        transform_.resize(output_frames.size());
        for (std::vector<int>::size_type i = 0; i < output_frames.size(); i++) {
            if (tf_buffer_->canTransform(output_frames[i], cloud_frame, rclcpp::Time(0), rclcpp::Duration(2, 0))) {
                geometry_msgs::msg::TransformStamped tfGeom = tf_buffer_->lookupTransform(output_frames[i], cloud_frame, rclcpp::Time(0));

                tf2::convert(tfGeom, transform_[i]);
            }
        }
    }

    for (std::vector<int>::size_type i = 0; i < output_frames.size(); i++) {
        myPointCloud pcl_out, tmpPcl;
        pcl::PCLPointCloud2 tmpPcl2;

        pcl_conversions::toPCL(*pcl_in, tmpPcl2);
        pcl::fromPCLPointCloud2(tmpPcl2, tmpPcl);

        // Initialize the header of the temporary pointcloud, needed to rototranslate the three points that define our plane
        // It shall be equal to the input point cloud's one, changing only the 'frame_id'
        string tmpFrame = output_frames[i];
        pcl_out.header = tmpPcl.header;

        // Ask tf to rototranslate the velodyne pointcloud to base_frame reference
        pcl_ros::transformPointCloud(tmpPcl, pcl_out, transform_[i]);
        pcl_out.header.frame_id = tmpFrame;

        // Transform the pcl into eigen matrix
        Eigen::MatrixXf tmpEigenMatrix;
        pcl::toPCLPointCloud2(pcl_out, tmpPcl2);
        pcl::getPointCloudAsEigen(tmpPcl2, tmpEigenMatrix);

        // Extract the points close to the z=0 plane, convert them into a laser-scan message and publish it
        pointcloud_to_laserscan(tmpEigenMatrix, pcl_out.header, i);
    }
}

void LaserscanVirtualizer::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index) // pcl::PCLPointCloud2 *merged_cloud)
{
    sensor_msgs::msg::LaserScan output;
    output.header = pcl_conversions::fromPCL(scan_header);
    output.angle_min = params_.angle_min;
    output.angle_max = params_.angle_max;
    output.angle_increment = params_.angle_increment;
    output.time_increment = 0.0; // params_.time_increment; // Not used in original
    output.scan_time = params_.scan_time;
    output.range_min = params_.range_min;
    output.range_max = params_.range_max;

    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(ranges_size, output.range_max + 1.0);

    for (int i = 0; i < points.cols(); i++) {
        const float& x = points(0, i);
        const float& y = points(1, i);
        const float& z = points(2, i);

        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
            RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = pow(y, 2) + pow(x, 2);
        double range_min_sq_ = output.range_min * output.range_min;
        if (range_sq < range_min_sq_) {
            RCLCPP_DEBUG(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }

        double angle = atan2(y, x);
        if (angle < output.angle_min || angle > output.angle_max) {
            RCLCPP_DEBUG(this->get_logger(), "rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
            continue;
        }

        int index = (angle - output.angle_min) / output.angle_increment;
        if (output.ranges[index] * output.ranges[index] > range_sq) {
            output.ranges[index] = sqrt(range_sq);
        }
    }

    virtual_scan_publishers[pub_index]->publish(output);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LaserscanVirtualizer>());

    rclcpp::shutdown();

    return 0;
}
