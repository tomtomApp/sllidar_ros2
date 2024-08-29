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
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/closest_point_marker", 10);
        closest_point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/closest_point", 10);
        fov_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fov_marker", 10);

        // 初期化
        threshold_ = 1.0; // 円の半径
        forward_angle_min_ = -M_PI_2; // 前方の最小角度 (-90度)
        forward_angle_max_ = M_PI_2;  // 前方の最大角度 (+90度)
        num_points_ = 30; // 扇形を構成する頂点の数
    }

private:
    void filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_msg = sensor_msgs::msg::LaserScan(*msg);
        float closest_distance = std::numeric_limits<float>::infinity();
        float closest_x = 0.0;
        float closest_y = 0.0;

        // 円の中にある前方の点群のみ表示
        float center_x = 0.0; // 中心のX座標 (円の中心)
        float center_y = 0.0; // 中心のY座標 (円の中心)

        // フィルタリング処理と最も近い点の検索
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float angle = msg->angle_min + i * msg->angle_increment;
            float x = msg->ranges[i] * cos(angle);
            float y = msg->ranges[i] * sin(angle);

            // 前方の角度範囲内かつ閾値以内の点群のみ残す
            if (angle >= forward_angle_min_ && angle <= forward_angle_max_)
            {
                float distance = std::hypot(x - center_x, y - center_y);
                if (distance > threshold_)
                {
                    filtered_msg.ranges[i] = std::numeric_limits<float>::infinity(); // 閾値外ならフィルタリング
                }
                else if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_x = x;
                    closest_y = y;
                }
            }
            else
            {
                filtered_msg.ranges[i] = std::numeric_limits<float>::infinity(); // 後方の点群をフィルタリング
            }
        }

        // 最も近い点のマーカーを生成
        visualization_msgs::msg::Marker closest_point_marker;
        closest_point_marker.header.frame_id = "base_link"; // フレームIDは適切なものを指定
        closest_point_marker.header.stamp = this->get_clock()->now();
        closest_point_marker.ns = "closest_point";
        closest_point_marker.id = 0;
        closest_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        closest_point_marker.action = visualization_msgs::msg::Marker::ADD;
        closest_point_marker.pose.position.x = closest_x;
        closest_point_marker.pose.position.y = closest_y;
        closest_point_marker.pose.position.z = 0.0;
        closest_point_marker.pose.orientation.w = 1.0;
        closest_point_marker.scale.x = 0.1; // 球の大きさ
        closest_point_marker.scale.y = 0.1;
        closest_point_marker.scale.z = 0.1;
        closest_point_marker.color.a = 1.0;
        closest_point_marker.color.r = 0.0;
        closest_point_marker.color.g = 0.0;
        closest_point_marker.color.b = 1.0; // 青色

        // 最も近い点の座標をgeometry_msgs::msg::Pointとしてパブリッシュ
        geometry_msgs::msg::Point closest_point;
        closest_point.x = closest_x;
        closest_point.y = closest_y;
        closest_point.z = 0.0;
        closest_point_publisher_->publish(closest_point);

        // 視野を表すマーカーを生成
        visualization_msgs::msg::Marker fov_marker;
        fov_marker.header.frame_id = "base_link"; // フレームIDは適切なものを指定
        fov_marker.header.stamp = this->get_clock()->now();
        fov_marker.ns = "fov";
        fov_marker.id = 1;
        fov_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // 視野を三角形リストとして定義
        fov_marker.action = visualization_msgs::msg::Marker::ADD;
        fov_marker.pose.orientation.w = 1.0;
        fov_marker.scale.x = 1.0;
        fov_marker.scale.y = 1.0;
        fov_marker.scale.z = 0.1;
        fov_marker.color.a = 0.5;
        fov_marker.color.r = 1.0;
        fov_marker.color.g = 1.0;
        fov_marker.color.b = 0.0; // 半透明な黄色

        // 視野マーカーの頂点を計算
        geometry_msgs::msg::Point p0;
        p0.x = 0.0;
        p0.y = 0.0;
        p0.z = 0.0;

        std::vector<geometry_msgs::msg::Point> points;
        float max_range = 1.0;
        float angle_step = (forward_angle_max_ - forward_angle_min_) / num_points_;

        for (size_t i = 0; i <= num_points_; ++i)
        {
            float angle = forward_angle_min_ + i * angle_step;
            geometry_msgs::msg::Point p;
            p.x = max_range * cos(angle);
            p.y = max_range * sin(angle);
            p.z = 0.0;
            points.push_back(p);
        }

        // 三角形リストに頂点を追加
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            fov_marker.points.push_back(p0);        // 基点
            fov_marker.points.push_back(points[i]); // 扇形の左端
            fov_marker.points.push_back(points[i + 1]); // 扇形の右端
        }

        // マーカーとフィルタリング結果をpublish
        marker_publisher_->publish(closest_point_marker);
        fov_marker_publisher_->publish(fov_marker);
        publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr closest_point_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_;
    float threshold_;     // 円の半径として使用
    float forward_angle_min_; // 前方角度の最小値 (-90度)
    float forward_angle_max_; // 前方角度の最大値 (+90度)
    size_t num_points_; // 扇形の頂点数
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilter>());
    rclcpp::shutdown();
    return 0;
}

