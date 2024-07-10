#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

class PointCloudFilter : public rclcpp::Node
{
public:
    PointCloudFilter() : Node("point_cloud_filter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PointCloudFilter::filter_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/Filtered/scan", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/filtered_point_marker", 10);

        // 初期化
        last_median_x_ = 0.0;
        last_median_y_ = 0.0;
        has_last_point_ = false;
        has_last_scan_ = false;  // 前回のスキャンが存在するかどうか
        marker_id_ = 0;  // 線のIDを初期化
    }

private:
    void filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_msg = sensor_msgs::msg::LaserScan(*msg);
        float sum_x = 0.0;
        float sum_y = 0.0;
        int count = 0;

        std::vector<geometry_msgs::msg::Point> points;
        std::vector<float> x_values;
        std::vector<float> y_values;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {

                float angle = msg->angle_min + i * msg->angle_increment;
                float x = msg->ranges[i] * cos(angle);
                float y = msg->ranges[i] * sin(angle);

                geometry_msgs::msg::Point point;
                point.x = x;
                point.y = y;
                point.z = 0.0;
                points.push_back(point);

                if (has_last_scan_)
                {
                    // 前回の点群データと現在の点群データの距離を計算
                    float last_angle = last_scan_.angle_min + i * last_scan_.angle_increment;
                    float last_x = last_scan_.ranges[i] * cos(last_angle);
                    float last_y = last_scan_.ranges[i] * sin(last_angle);
                    float distance = sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));
                    if (std::isnan(distance) || std::isinf(distance) || distance < 0.1)
                    {
                        filtered_msg.ranges[i] = std::numeric_limits<float>::infinity();
                    }
                    else
                    {
                        int neighbor_count = 0;
                        for (const auto& other_point : points)
                        {
                            if (&point != &other_point)
                            {
                                float neighbor_distance = sqrt(pow(point.x - other_point.x, 2) + pow(point.y - other_point.y, 2));
                                if (neighbor_distance <= 0.30)
                                {
                                    neighbor_count++;
                                }
                            }
                        }

                        if (neighbor_count < 30)
                        {
                            filtered_msg.ranges[i] = std::numeric_limits<float>::infinity();
                        }
                        else
                        {
                            //RCLCPP_INFO(this->get_logger(), "Distance: %f", distance);
                            sum_x += x;
                            sum_y += y;
                            count++;
                            x_values.push_back(x);
                            y_values.push_back(y);
                        }
                    }
                }
            
        }

        if (count > 0)
        {
            
            std::nth_element(x_values.begin(), x_values.begin() + x_values.size() / 2, x_values.end());
            std::nth_element(y_values.begin(), y_values.begin() + y_values.size() / 2, y_values.end());
            float median_x = x_values[x_values.size() / 2];
            float median_y = y_values[y_values.size() / 2];
            /*
            float median_x = sum_x / count;
            float median_y = sum_y / count;
            */
            float marker_distance = sqrt(pow(median_x - last_median_x_, 2) + pow(median_y - last_median_y_, 2));
            
            if (last_median_x_ != 0 && last_median_y_ != 0 && marker_distance > 0.7){
                RCLCPP_INFO(this->get_logger(), "Distance: %f", marker_distance);
                median_x = last_median_x_;
                median_y = last_median_y_;
            }

            if (has_last_point_)
            {
                visualization_msgs::msg::Marker line;
                line.header.frame_id = "laser";
                line.header.stamp = this->get_clock()->now();
                line.ns = "trajectory";
                line.id = marker_id_++;
                line.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line.action = visualization_msgs::msg::Marker::ADD;
                line.points.push_back(createPoint(last_median_x_, last_median_y_));
                line.points.push_back(createPoint(median_x, median_y));
                line.scale.x = 0.01;
                line.color.a = 1.0;
                line.color.r = 0.0;
                line.color.g = 1.0;
                line.color.b = 0.0;
                line.lifetime = rclcpp::Duration(3, 0);
                marker_publisher_->publish(line);
            }
            publish_circle_marker(median_x, median_y);
            last_median_x_ = median_x;
            last_median_y_ = median_y;
            has_last_point_ = true;
        }
        last_scan_ = *msg;
        has_last_scan_ = true;

        publisher_->publish(filtered_msg);
    }

    void publish_circle_marker(float x, float y)
    {
        visualization_msgs::msg::Marker circle;
        circle.header.frame_id = "laser";
        circle.header.stamp = this->get_clock()->now();
        circle.ns = "circle";
        circle.id = marker_id_++;
        circle.type = visualization_msgs::msg::Marker::CYLINDER;
        circle.action = visualization_msgs::msg::Marker::ADD;
        circle.pose.position.x = x;
        circle.pose.position.y = y;
        circle.pose.position.z = 0.0;
        circle.pose.orientation.x = 0.0;
        circle.pose.orientation.y = 0.0;
        circle.pose.orientation.z = 0.0;
        circle.pose.orientation.w = 1.0;
        circle.scale.x = 0.5;  // 円の直径
        circle.scale.y = 0.5;  // 円の直径
        circle.scale.z = 0.01; // 円の厚み
        circle.color.a = 0.5;  // 透明度
        circle.color.r = 1.0;  // 赤色
        circle.color.g = 0.0;
        circle.color.b = 0.0;
        circle.lifetime = rclcpp::Duration(3, 0);
        marker_publisher_->publish(circle);
    }

    geometry_msgs::msg::Point createPoint(float x, float y)
    {
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;
        return point;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    float last_median_x_;
    float last_median_y_;
    bool has_last_point_;
    sensor_msgs::msg::LaserScan last_scan_; // 前回のスキャンデータ
    bool has_last_scan_;  // 前回のスキャンが存在するかどうか
    int marker_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilter>());
    rclcpp::shutdown();
    return 0;
}
