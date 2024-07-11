#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <limits>
#include <cmath>

class PointCloudFilter : public rclcpp::Node
{
public:
    PointCloudFilter() : Node("point_cloud_filter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PointCloudFilter::filter_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/Filtered/scan", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/filtered_point_marker", 10);

        //初期化
        last_avg_x_ = 0.0; // 平均座標
        last_avg_y_ = 0.0;
        has_last_point_ = false;
        marker_id_ = 0;
        initial_scan_done_ = false; // 最初のフィルタリングのみtrue
        threshold_ = 0.3; // 足の追跡時に使う閾値

        // n秒後に軌跡を描く
        drawing_timer_ = this->create_wall_timer(
            std::chrono::seconds(100),
            [this]() { start_drawing_ = true; });
    }

private:
    void filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_msg = sensor_msgs::msg::LaserScan(*msg);

        if (!initial_scan_done_)
        {
            // x[m]以上離れている点をフィルタリングする処理(初回のみ)
            float sum_x = 0.0;
            float sum_y = 0.0;
            int count = 0;
            int label = 0;

            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                if (msg->ranges[i] < 0.2)
                {
                    float angle = msg->angle_min + i * msg->angle_increment; // 角度計算
                    float x = msg->ranges[i] * cos(angle); // 座標計算
                    float y = msg->ranges[i] * sin(angle);
                    remembered_points_[label].emplace_back(x, y);
                    sum_x += x;
                    sum_y += y;
                    count++;
                }
                else
                {
                    filtered_msg.ranges[i] = std::numeric_limits<float>::infinity(); // フィルタリング
                }
            }

            if (count > 0)
            {
                last_avg_x_ = sum_x / count;
                last_avg_y_ = sum_y / count;
                initial_scan_done_ = true; // 初回以降はスキップ
            }
        }
        // 追跡する処理(初回以降)
        else
        {
            // 記憶した点群のラベルを追跡してフィルタリング
            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                float angle = msg->angle_min + i * msg->angle_increment;
                float x = msg->ranges[i] * cos(angle);
                float y = msg->ranges[i] * sin(angle);
                bool found = false;

                for (const auto& [label, points] : remembered_points_)
                {
                    for (const auto& point : points)
                    {
                        if (std::hypot(point.first - x, point.second - y) < threshold_) // 記憶と現在(x, y)の距離<thresholdなら同値
                        {
                            found = true;
                            break;
                        }
                    }
                    if (found) break;
                }

                if (!found)
                {
                    filtered_msg.ranges[i] = std::numeric_limits<float>::infinity(); // 記憶と現在(x, y)の距離>thresholdならフィルタリング
                }
            }

            // 平均座標を計算し直す
            float sum_x = 0.0;
            float sum_y = 0.0;
            int count = 0;
            for (size_t i = 0; i < filtered_msg.ranges.size(); ++i)
            {
                if (filtered_msg.ranges[i] < std::numeric_limits<float>::infinity()) // 生き残った点群に限定
                {
                    float angle = filtered_msg.angle_min + i * filtered_msg.angle_increment;
                    float x = filtered_msg.ranges[i] * cos(angle);
                    float y = filtered_msg.ranges[i] * sin(angle);
                    sum_x += x;
                    sum_y += y;
                    count++;
                }
            }

            if (count > 0)
            {
                float avg_x = sum_x / count;
                float avg_y = sum_y / count;

                // 記憶した点群の平均座標を更新
                remembered_points_.clear();
                remembered_points_[0].emplace_back(avg_x, avg_y);

                // 平均座標を中心とした半径threshold_の円を描く
                visualization_msgs::msg::Marker circle;
                circle.header.frame_id = "laser";
                circle.header.stamp = this->get_clock()->now();
                circle.ns = "circle";
                circle.id = 0;
                circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
                circle.action = visualization_msgs::msg::Marker::ADD;
                circle.scale.x = 0.01;
                circle.color.a = 1.0;
                circle.color.r = 1.0;
                circle.color.g = 0.0;
                circle.color.b = 0.0;

                //円の外枠を描くための点を追加
                const int num_points = 100; // 円を構成する点の数
                for (int i = 0; i <= num_points; ++i)
                {
                    float angle = 2.0 * M_PI * i / num_points;
                    geometry_msgs::msg::Point p;
                    p.x = avg_x + threshold_ * cos(angle);
                    p.y = avg_y + threshold_ * sin(angle);
                    p.z = 0.0;
                    circle.points.push_back(p);
                }

                marker_publisher_->publish(circle);

                // 平均座標を繋いでいくことで軌跡を描く
                if (has_last_point_ && start_drawing_)
                {
                    visualization_msgs::msg::Marker line;
                    line.header.frame_id = "laser";
                    line.header.stamp = this->get_clock()->now();
                    line.ns = "trajectory";
                    line.id = marker_id_++;
                    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    line.action = visualization_msgs::msg::Marker::ADD;
                    line.points.push_back(createPoint(last_avg_x_, last_avg_y_));
                    line.points.push_back(createPoint(avg_x, avg_y));
                    line.scale.x = 0.01; // 太さ
                    line.color.a = 1.0; // 色
                    line.color.r = 1.0;
                    line.color.g = 0.0;
                    line.color.b = 0.0;
                    line.lifetime = rclcpp::Duration::from_seconds(5); // n秒後に消える
                    marker_publisher_->publish(line);
                }
                last_avg_x_ = avg_x;
                last_avg_y_ = avg_y;
                has_last_point_ = true;
            }
        }

        publisher_->publish(filtered_msg); // 処理したデータをpublish
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
    rclcpp::TimerBase::SharedPtr drawing_timer_;
    float last_avg_x_;
    float last_avg_y_;
    bool has_last_point_;
    bool start_drawing_ = false;
    int marker_id_;
    bool initial_scan_done_;
    float threshold_;
    std::unordered_map<int, std::vector<std::pair<float, float>>> remembered_points_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilter>());
    rclcpp::shutdown();
    return 0;
}

