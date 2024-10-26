#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

class samplingNode : public rclcpp::Node
{
public:
    samplingNode()
    : Node("sampling")
    {
        this->declare_parameter<float>("leaf_size"); 
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sampling_points", 10);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", qos, std::bind(&samplingNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *cloud);

        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
       
        float leaf_size;
        this->get_parameter("leaf_size", leaf_size); 
        sor.setLeafSize(leaf_size, leaf_size, leaf_size); 

        sor.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(*cloud_filtered, output);
        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<samplingNode>());
    rclcpp::shutdown();
    return 0;
}
