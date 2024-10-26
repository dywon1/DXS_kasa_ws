// 필요한 헤더 파일 포함
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/segmentation/extract_clusters.h"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// spline.h 파일 포함
#include "spline.h"

#include <vector>
#include <Eigen/Dense>
#include <algorithm>

using std::placeholders::_1;

struct ClusterData {
    Eigen::Vector3f center;
    int point_count;
    std::vector<Eigen::Vector3f> points;
    Eigen::Vector3f min_point;
    Eigen::Vector3f max_point;
};

class ClusterNode : public rclcpp::Node
{
public:
    ClusterNode()
        : Node("path_planning")
    {
        // 파라미터 초기화
        this->declare_parameter<double>("eps", 0.3); // eps 값 증가
        this->declare_parameter<int>("min_samples", 2); // min_samples 값 감소
        this->declare_parameter<int>("max_cones", 25);

        // 퍼블리셔 초기화
        midpoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/midpoints_marker", 10);
        cluster_points_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_points_marker", 10);
        midpoint_float_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/midpoints_float", 10);
        clustered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/clustered_points", 10);
        spline_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/spline_path_marker", 10);

        // 구독자 초기화
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ransac_points", 10, std::bind(&ClusterNode::cluster_callback, this, _1));
    }

private:
    void cluster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // PointCloud2 메시지를 PCL 포인트 클라우드로 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 클러스터링 수행
        double eps = this->get_parameter("eps").as_double();
        int min_samples = this->get_parameter("min_samples").as_int();

        std::vector<int> cluster_labels = perform_clustering(cloud, eps, min_samples);

        // 클러스터 수 계산
        int num_clusters = *std::max_element(cluster_labels.begin(), cluster_labels.end()) + 1;

        // 클러스터 센터와 포인트 수, 포인트들, 크기를 계산
        std::vector<ClusterData> cluster_centers_and_points;
        for (int cluster_id = 0; cluster_id < num_clusters; ++cluster_id)
        {
            std::vector<Eigen::Vector3f> cluster_points;
            for (size_t i = 0; i < cluster_labels.size(); ++i)
            {
                if (cluster_labels[i] == cluster_id)
                {
                    cluster_points.push_back(cloud->points[i].getVector3fMap());
                }
            }
            if (!cluster_points.empty())
            {
                Eigen::Vector3f cluster_center = Eigen::Vector3f::Zero();
                Eigen::Vector3f min_point = cluster_points[0];
                Eigen::Vector3f max_point = cluster_points[0];

                for (const auto& pt : cluster_points)
                {
                    cluster_center += pt;

                    // 최소값 및 최대값 업데이트
                    min_point = min_point.cwiseMin(pt);
                    max_point = max_point.cwiseMax(pt);
                }
                cluster_center /= cluster_points.size();

                cluster_centers_and_points.push_back({cluster_center, static_cast<int>(cluster_points.size()), cluster_points, min_point, max_point});
            }
        }

        // 클러스터링 결과를 퍼블리시하여 시각화
        publish_clustered_point_cloud(cloud, cluster_labels);

        // 콘 분류 및 퍼블리시
        classify_and_publish_cones(cluster_centers_and_points);
    }

    std::vector<int> perform_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps, int min_samples)
    {
        // KdTree 생성
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // 클러스터 인덱스 저장 벡터
        std::vector<pcl::PointIndices> cluster_indices;

        // 클러스터링 설정
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(eps);
        ec.setMinClusterSize(min_samples);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 각 포인트에 대한 클러스터 레이블 할당
        std::vector<int> labels(cloud->points.size(), -1);
        int cluster_id = 0;
        for (const auto& indices : cluster_indices)
        {
            for (const auto& index : indices.indices)
            {
                labels[index] = cluster_id;
            }
            ++cluster_id;
        }

        return labels;
    }

    bool size_in_range(const Eigen::Vector3f& size)
    {
        float min_x = 0.01f;
        float max_x = 0.40f;
        float min_y = 0.01f;
        float max_y = 0.40f;
        float min_z = 0.01f;
        float max_z = 0.75f;

        return (size.x() >= min_x && size.x() <= max_x) &&
               (size.y() >= min_y && size.y() <= max_y) &&
               (size.z() >= min_z && size.z() <= max_z);
    }

    void classify_and_publish_cones(const std::vector<ClusterData>& cluster_centers_and_points)
    {
        int max_cones = this->get_parameter("max_cones").as_int();

        // 크기 범위에 맞는 클러스터만 선택
        std::vector<ClusterData> cones;
        for (const auto& cluster : cluster_centers_and_points)
        {
            // 클러스터 크기 계산
            Eigen::Vector3f size = cluster.max_point - cluster.min_point;

            // 크기 범위 조건 확인
            if (size_in_range(size))
            {
                cones.push_back(cluster);
            }
        }

        // 필터링된 콘들을 거리 순으로 정렬
        std::sort(cones.begin(), cones.end(),
                  [](const ClusterData& a, const ClusterData& b)
                  {
                      return a.center.head<2>().norm() < b.center.head<2>().norm();
                  });

        if (cones.size() > static_cast<size_t>(max_cones))
        {
            cones.resize(max_cones);
        }

        visualization_msgs::msg::MarkerArray cluster_points_markers;
        visualization_msgs::msg::MarkerArray midpoint_markers;
        std_msgs::msg::Float32MultiArray midpoint_array_msg;

        // 중점 좌표를 저장할 벡터 (X, Y 쌍으로 저장)
        std::vector<std::pair<double, double>> midpoints;

        size_t i = 0;
        while (i + 1 < cones.size())
        {
            const ClusterData& cone_0 = cones[i];
            const ClusterData& cone_1 = cones[i + 1];

            double dist = (cone_0.center.head<2>() - cone_1.center.head<2>()).norm();

            // 거리 조건 확인
            if (dist > 4.0 && dist < 8.0)
            {
                Eigen::Vector3f mid_point = (cone_0.center + cone_1.center) / 2.0f;

                double ccw_val = ccw(Eigen::Vector2f(0.0f, 0.0f), cone_0.center.head<2>(), cone_1.center.head<2>());

                std::string color_0;
                std::string color_1;

                if (ccw_val > 0)
                {
                    color_0 = "yellow";   // 왼쪽 콘
                    color_1 = "blue"; // 오른쪽 콘
                }
                else
                {
                    color_0 = "blue"; // 오른쪽 콘
                    color_1 = "yellow";   // 왼쪽 콘
                }

                // 각 콘의 클러스터 포인트를 마커로 추가
                publish_cluster_points(cone_0.points, static_cast<int>(i), color_0, cluster_points_markers);
                publish_cluster_points(cone_1.points, static_cast<int>(i + 1), color_1, cluster_points_markers);

                // 중점 마커 생성 및 추가
                visualization_msgs::msg::Marker midpoint_marker = create_midpoint_marker(mid_point, static_cast<int>(i));
                midpoint_markers.markers.push_back(midpoint_marker);

                // 중점 데이터를 Float32MultiArray에 추가
                midpoint_array_msg.data.push_back(static_cast<float>(i));
                midpoint_array_msg.data.push_back(-mid_point[0]);
                midpoint_array_msg.data.push_back(-mid_point[1]);

                // 중점 좌표를 벡터에 추가 (부호 반전하여 저장)
                midpoints.emplace_back(-mid_point[0], -mid_point[1]);

                i += 2; // 다음 두 개의 콘으로 이동
            }
            else
            {
                i++;
            }
        }

        // 모든 클러스터 포인트 마커를 퍼블리시
        cluster_points_pub_->publish(cluster_points_markers);

        // 중점 마커를 한 번에 퍼블리시
        midpoint_marker_pub_->publish(midpoint_markers);

        // 중점 데이터를 한 번에 퍼블리시
        midpoint_float_pub_->publish(midpoint_array_msg);

        // 중점 데이터가 충분한 경우 스플라인 보간 적용
        if (midpoints.size() >= 3)
        {
            // 중점 좌표를 X 좌표에 따라 오름차순으로 정렬
            std::sort(midpoints.begin(), midpoints.end(),
                      [](const std::pair<double, double>& a, const std::pair<double, double>& b)
                      {
                          return a.first < b.first;
                      });

            // 동일한 X 좌표를 가지는 포인트 제거 (X 좌표가 동일하면 스플라인 보간이 불가능)
            auto last = std::unique(midpoints.begin(), midpoints.end(),
                                    [](const std::pair<double, double>& a, const std::pair<double, double>& b)
                                    {
                                        return a.first == b.first;
                                    });
            midpoints.erase(last, midpoints.end());

            // 정렬된 X, Y 좌표를 각각 벡터로 분리
            std::vector<double> mid_xs(midpoints.size());
            std::vector<double> mid_ys(midpoints.size());
            for (size_t i = 0; i < midpoints.size(); ++i)
            {
                mid_xs[i] = midpoints[i].first;
                mid_ys[i] = midpoints[i].second;
            }

            // 스플라인 객체 생성
            tk::spline s;
            s.set_points(mid_xs, mid_ys);

            // 스플라인 경로를 생성할 포인트 개수 설정
            int num_points = 100;
            double x_start = mid_xs.front();
            double x_end = mid_xs.back();
            double x_step = (x_end - x_start) / (num_points - 1);

            visualization_msgs::msg::Marker spline_marker;
            spline_marker.header.frame_id = "os_sensor";
            spline_marker.header.stamp = this->now();
            spline_marker.ns = "spline_path";
            spline_marker.id = 0;
            spline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            spline_marker.action = visualization_msgs::msg::Marker::ADD;
            spline_marker.scale.x = 0.1; // 선의 두께
            spline_marker.color.a = 1.0;
            spline_marker.color.g = 1.0; // 초록색

            for (int i = 0; i < num_points; ++i)
            {
                double x = x_start + i * x_step;
                double y = s(x);

                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = y;
                p.z = 0.0;
                spline_marker.points.push_back(p);
            }

            // 스플라인 경로 마커 퍼블리시
            visualization_msgs::msg::MarkerArray spline_marker_array;
            spline_marker_array.markers.push_back(spline_marker);
            spline_marker_pub_->publish(spline_marker_array);

            // 스플라인 경로의 좌표들을 Float32MultiArray로 퍼블리시 (필요한 경우)
            // std_msgs::msg::Float32MultiArray spline_array_msg;
            // for (int i = 0; i < num_points; ++i)
            // {
            //     double x = x_start + i * x_step;
            //     double y = s(x);

            //     spline_array_msg.data.push_back(x);
            //     spline_array_msg.data.push_back(y);
            // }
            // spline_float_pub_->publish(spline_array_msg);
        }
    }

    double ccw(const Eigen::Vector2f& a, const Eigen::Vector2f& b, const Eigen::Vector2f& c)
    {
        return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
    }

    void publish_cluster_points(const std::vector<Eigen::Vector3f>& points, int id, const std::string& color, visualization_msgs::msg::MarkerArray& marker_array)
    {
        visualization_msgs::msg::Marker points_marker;
        points_marker.header.frame_id = "os_sensor";
        points_marker.header.stamp = this->now();
        points_marker.ns = "cluster_points";
        points_marker.id = id;
        points_marker.type = visualization_msgs::msg::Marker::POINTS;
        points_marker.action = visualization_msgs::msg::Marker::ADD;

        points_marker.scale.x = 0.1; // 포인트 크기
        points_marker.scale.y = 0.1;

        // 색상 설정
        if (color == "blue")
        {
            points_marker.color.r = 0.0f;
            points_marker.color.g = 0.0f;
            points_marker.color.b = 1.0f;
            points_marker.color.a = 1.0f;
        }
        else if (color == "yellow")
        {
            points_marker.color.r = 1.0f;
            points_marker.color.g = 1.0f;
            points_marker.color.b = 0.0f;
            points_marker.color.a = 1.0f;
        }

        // 포인트 추가
        for (const auto& pt : points)
        {
            geometry_msgs::msg::Point p;
            p.x = -pt[0]; // 부호 반전
            p.y = -pt[1]; // 부호 반전
            p.z = pt[2];
            points_marker.points.push_back(p);
        }

        // 왼쪽, 오른쪽 콘 마커의 수명 설정
        points_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_array.markers.push_back(points_marker);
    }

    visualization_msgs::msg::Marker create_midpoint_marker(const Eigen::Vector3f& midpoint, int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "os_sensor";
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = marker.scale.y = marker.scale.z = 0.5;

        marker.color.a = 1.0;
        marker.color.r = 1.0; // 빨간색

        // X축, Y축 부호 반전
        marker.pose.position.x = -midpoint[0];
        marker.pose.position.y = -midpoint[1];
        marker.pose.position.z = midpoint[2];

        marker.id = id;

        // 중점 경로 마커 시간
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        return marker;
    }

    void publish_clustered_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& labels)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored_cloud->points.resize(cloud->points.size());

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZRGB point;
            point.x = -cloud->points[i].x; // 부호 반전
            point.y = -cloud->points[i].y; // 부호 반전
            point.z = cloud->points[i].z;

            int label = labels[i];
            if (label == -1)
            {
                // 노이즈 포인트 (검정색)
                point.r = 0;
                point.g = 0;
                point.b = 0;
            }
            else
            {
                // 클러스터별로 색상 지정
                uint8_t r = static_cast<uint8_t>(label * 53) % 255;
                uint8_t g = static_cast<uint8_t>(label * 97) % 255;
                uint8_t b = static_cast<uint8_t>(label * 151) % 255;
                point.r = r;
                point.g = g;
                point.b = b;
            }
            colored_cloud->points[i] = point;
        }

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*colored_cloud, output_msg);
        output_msg.header.frame_id = "os_sensor";
        output_msg.header.stamp = this->now();

        clustered_cloud_pub_->publish(output_msg);
    }

    // 멤버 변수 선언
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr midpoint_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_points_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr midpoint_float_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr spline_marker_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterNode>());
    rclcpp::shutdown();
    return 0;
}
