#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

using geometry_msgs::msg::PoseStamped;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class IconPublisher : public rclcpp::Node
{
public:
    IconPublisher() : Node("icon_publisher")
    {

        // Transient local keeps last message for late subscribers (RViz)
        auto qos = rclcpp::QoS(1).transient_local().reliable();
        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("icons", qos);
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

        populate_people();
        populate_hunters();

        publishMarkerArray();
    }

    void publishMarkerArray()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        marker_array.markers.push_back(
            produceIcon("base", 0,
                        "package://visual_icons/meshes/home_green.glb",
                        0.0, 0.0, 0.0,
                        3.0));

        id++;

        for (size_t i = 0; i < detected_people_.poses.size(); ++i)
        {
            const auto &pose = detected_people_.poses[i];
            marker_array.markers.push_back(
                produceIcon("person", id++,
                            "package://visual_icons/meshes/person.glb",
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                            3.0));
        }

        for (size_t i = 0; i < detected_hunters_.poses.size(); ++i)
        {
            const auto &pose = detected_hunters_.poses[i];
            marker_array.markers.push_back(
                produceIcon("person", id++,
                            "package://visual_icons/meshes/hunter_eye.glb",
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                            3.0));
        }

        pub_->publish(marker_array);
    }

    Marker produceIcon(const std::string &ns, int id, const std::string &mesh_uri,
                       double x, double y, double z, double scale)
    {
        Marker m;
        m.header.frame_id = "map";
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = Marker::MESH_RESOURCE;
        m.action = Marker::ADD;
        m.mesh_resource = mesh_uri;
        m.mesh_use_embedded_materials = true; // use texture/material from DAE
        // m.texture_resource = "file://home/greese/41068_ws/src/visual_icons/meshes/textures/circle.png"; // not used with embedded materials
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;

        // Set scale to resize the mesh (xyz all set)
        m.scale.x = m.scale.y = m.scale.z = scale;

        m.pose.orientation.x = -0.5; // 90 deg around X axis
        m.pose.orientation.y = 0.5;  // to
        m.pose.orientation.z = 0.5;
        m.pose.orientation.w = -0.5;

        m.lifetime = rclcpp::Duration(0, 0); // forever

        // Reset marker params after use
        // TODO

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
        cube.header.stamp = now();          // refresh stamp
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

    void populate_hunters()
    {
        geometry_msgs::msg::Pose hunter1;
        hunter1.position.x = -5.0;
        hunter1.position.y = -2.0;
        hunter1.position.z = 0.0;

        geometry_msgs::msg::Pose hunter2;
        hunter2.position.x = 2.0;
        hunter2.position.y = 4.0;
        hunter2.position.z = 0.0;

        detected_hunters_.poses.push_back(hunter1);
        detected_hunters_.poses.push_back(hunter2);
    }

    void populate_people()
    {
        geometry_msgs::msg::Pose person1;
        person1.position.x = 5.0;
        person1.position.y = 0.0;
        person1.position.z = 0.0;

        geometry_msgs::msg::Pose person2;
        person2.position.x = -3.0;
        person2.position.y = 3.0;
        person2.position.z = 0.0;

        detected_people_.poses.push_back(person1);
        detected_people_.poses.push_back(person2);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_patrol_;
    geometry_msgs::msg::PoseArray detected_people_;
    geometry_msgs::msg::PoseArray detected_hunters_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_len_;
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
