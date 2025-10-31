#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using visualization_msgs::msg::Marker;

class IconPublisher : public rclcpp::Node
{
public:
    IconPublisher() : Node("icon_publisher")
    {
        auto qos = rclcpp::QoS(1).transient_local().reliable();
        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("icons", qos);

        // Subscribe to human pose from Python node
        sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/human_pose", 10,
            std::bind(&IconPublisher::humanCallback, this, std::placeholders::_1));

        publishHomeIcon();
    }

private:
    void publishHomeIcon()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(produceIcon(
            "base", 0, "package://visual_icons/meshes/home_green.glb",
            0.0, 0.0, 0.0, 3.0));
        pub_->publish(marker_array);
    }

    void humanCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(produceIcon(
            "human", 1, "package://visual_icons/meshes/person.glb",
            msg->point.x, // X world
            msg->point.y, // Y World
            0, // Ignore Z
            3.0));
        pub_->publish(marker_array);
        
        RCLCPP_INFO(this->get_logger(), "🧍 Human icon added at (%.2f, %.2f, %.2f)",
                    msg->point.x, msg->point.y, 0);
    }

    Marker produceIcon(const std::string &ns, int id, const std::string &mesh_uri,
                       double x, double y, double z, double scale)
    {
        Marker m;
        m.header.frame_id = "map";  // or "odom" if your TF tree uses that
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
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.pose.orientation.x = -0.5;
        m.pose.orientation.y = 0.5;
        m.pose.orientation.z = 0.5;
        m.pose.orientation.w = -0.5;
        m.lifetime = rclcpp::Duration(0, 0);  // forever
        return m;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IconPublisher>());
    rclcpp::shutdown();
    return 0;
}
