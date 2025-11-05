#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class IconPublisher : public rclcpp::Node
{
public:
    IconPublisher() : Node("icon_publisher")
    {
        auto qos = rclcpp::QoS(1).transient_local().reliable();
        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("icons", qos);

        sub_human_ = create_subscription<PoseArray>(
            "/human_pose", 10,
            std::bind(&IconPublisher::humanCallback, this, std::placeholders::_1));

        sub_hunter_ = create_subscription<PoseArray>(
            "/hunter_pose", 10,
            std::bind(&IconPublisher::hunterCallback, this, std::placeholders::_1));

        publishIcons();
    }

private:
    void publishIcons()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        marker_array.markers.push_back(produceIcon(
            "base", 0, "package://visual_icons/meshes/home_green.glb",
            0.0, 0.0, 0.0, 3.0, "map"));

        int id = 1;
        for (const auto &pose : last_human_msg_.poses)
        {
            marker_array.markers.push_back(produceIcon(
                "human", id++, "package://visual_icons/meshes/person.glb",
                pose.position.x, pose.position.y, 0.0, 3.0, "map"));
        }

        // id = 1001;
        for (const auto &pose : last_hunter_msg_.poses)
        {
            marker_array.markers.push_back(produceIcon(
                "hunter", id++, "package://visual_icons/meshes/hunter_eye.glb",
                pose.position.x, pose.position.y, 0.0, 3.0, "map"));
        }

        pub_->publish(marker_array);
    }

    void humanCallback(const PoseArray::SharedPtr msg)
    {
        last_human_msg_ = *msg;
        publishIcons();
    }

    void hunterCallback(const PoseArray::SharedPtr msg)
    {
        last_hunter_msg_ = *msg;
        publishIcons();
    }

    Marker produceIcon(const std::string &ns, int id, const std::string &mesh_uri,
                       double x, double y, double z, double scale,
                       const std::string &frame_id)
    {
        Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = Marker::MESH_RESOURCE;
        m.action = Marker::ADD;
        m.mesh_resource = mesh_uri;
        m.mesh_use_embedded_materials = true;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.x = -0.5;
        m.pose.orientation.y = 0.5;
        m.pose.orientation.z = 0.5;
        m.pose.orientation.w = -0.5;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.lifetime = rclcpp::Duration(0, 0); // persistent until deleted
        return m;
    }

    rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
    rclcpp::Subscription<PoseArray>::SharedPtr sub_human_;
    rclcpp::Subscription<PoseArray>::SharedPtr sub_hunter_;
    rclcpp::TimerBase::SharedPtr timer_;

    PoseArray last_human_msg_;
    PoseArray last_hunter_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IconPublisher>());
    rclcpp::shutdown();
    return 0;
}