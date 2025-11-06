#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
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

        pub_patrol_ = create_publisher<visualization_msgs::msg::Marker>("patrol_marker", qos);

        sub_goal_ = create_subscription<PoseStamped>(

            "/behavior/traverse_goal", 10,

            [this](const PoseStamped::SharedPtr msg)

            {
                inspect_goal_ = *msg; // keep full header (frame_id)

                goal_received_ = true;

                publishPatrolArea(); // update RViz immediately
            });

        sub_len_ = create_subscription<std_msgs::msg::Float64>(

            "/behavior/inspect_length", 10,

            [this](const std_msgs::msg::Float64::SharedPtr msg)

            {
                inspect_length_ = std::max(1.0, msg->data); // clamp to non-negative

                length_received_ = true;

                publishPatrolArea();
            });

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

    void publishPatrolArea()

    {

        if (goal_received_)

        {

            Marker patrol_area = producePatrolArea(0);

            pub_patrol_->publish(patrol_area);

            if (length_received_)

            {

                inspect_length_ = 1.0; // reset to default after use

                length_received_ = false;

                goal_received_ = false;
            }
        }
    }

    Marker producePatrolArea(int id)

    {

        if (inspect_goal_.header.frame_id.empty())

            return Marker(); // no goal yet

        Marker cube;

        cube.header = inspect_goal_.header; // keep goal's frame_id & time

        cube.header.stamp = now(); // refresh stamp

        cube.ns = "inspect";

        // cube.id = INSPECT_ID;

        cube.id = id;

        cube.type = Marker::CUBE;

        cube.action = Marker::ADD;

        // Pose from goal

        cube.pose = inspect_goal_.pose;

        // Edge size = inspect_length_ (fallback to 1.0 if unset/zero)

        const double L = (inspect_length_ > 1.0) ? inspect_length_ : 1.0;

        cube.scale.x = L;

        cube.scale.y = L;

        cube.scale.z = 0.1;

        // Color (cyan)

        cube.color.a = 0.3;

        cube.color.r = 1.0;

        cube.color.g = 0.65;

        cube.color.b = 0.0;

        cube.lifetime = rclcpp::Duration(0, 0);

        return cube;
    }

    rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
    rclcpp::Subscription<PoseArray>::SharedPtr sub_human_;
    rclcpp::Subscription<PoseArray>::SharedPtr sub_hunter_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Marker>::SharedPtr pub_patrol_;
    rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_len_;

    PoseArray last_human_msg_;
    PoseArray last_hunter_msg_;
    geometry_msgs::msg::PoseStamped inspect_goal_;
    double inspect_length_{1.0};
    bool length_received_{false};
    bool goal_received_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IconPublisher>());
    rclcpp::shutdown();
    return 0;
}