#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

using visualization_msgs::msg::Marker;

class IconPublisher : public rclcpp::Node
{
public:
    IconPublisher() : Node("icon_publisher")
    {

        // Transient local keeps last message for late subscribers (RViz)
        auto qos = rclcpp::QoS(1).transient_local().reliable();
        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("icons", qos);
        // markers_ = visualization_msgs::msg::MarkerArray();

        // publishBaseIcon();
        //  Example: publish more icons later (detections, etc.)
        //  publishIcon("hotspot", 1, "package://my_visual_icons/meshes/detection_hotspot.dae", 12.3, 4.5, 0.06, 0.8);

        // publishPersonIcon();

        geometry_msgs::msg::Pose person1;
        person1.position.x = 5.0;
        person1.position.y = 0.0;
        person1.position.z = 0.0;

        geometry_msgs::msg::Pose person2;
        person2.position.x = -3.0;
        person2.position.y = 3.0;
        person2.position.z = 0.0;

        detected_people.poses.push_back(person1);
        detected_people.poses.push_back(person2);

        produceMarkerArray();
    }

    // void publishBaseIcon()
    // {
    //     publishIcon("base", 0,
    //                 "package://visual_icons/meshes/home_green.glb",
    //                 0.0, 0.0, 0.0, // x,y,z (slightly above ground to avoid z-fighting)
    //                 3.0            // scale in meters (uniform)
    //     );
    // }

    // void publishPersonIcon()
    // {
    //     publishIcon("person", 1,
    //                 "package://visual_icons/meshes/person.glb",
    //                 5.0, 0.0, 0.0, // x,y,z (slightly above ground to avoid z-fighting)
    //                 3.0            // scale in meters (uniform)
    //     );
    // }

    void produceMarkerArray()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        marker_array.markers.push_back(
            produceIcon("base", 0,
                        "package://visual_icons/meshes/home_green.glb",
                        0.0, 0.0, 0.0,
                        3.0));

        for (size_t i = 0; i < detected_people.poses.size(); ++i)
        {
            const auto &pose = detected_people.poses[i];
            marker_array.markers.push_back(
                produceIcon("person", i + 1,
                            "package://visual_icons/meshes/person.glb",
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
        // m.texture_resource = "fiel://home/greese/41068_ws/src/visual_icons/meshes/textures/circle.png"; // not used with embedded materials
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;

        // Set scale to resize the mesh (xyz all set)
        m.scale.x = m.scale.y = m.scale.z = scale;

        // If your DAE uses embedded material/texture, alpha still must be >0
        // m.color.a = 1.0;
        // m.color.r = m.color.g = m.color.b = 1.0; // ignored if embedded materials
        // m.color.r = 1.0;
        // m.color.g = 0.0;
        // m.color.b = 0.0;

        m.pose.orientation.x = -0.5; // 90 deg around X axis
        m.pose.orientation.y = 0.5;  // to
        m.pose.orientation.z = 0.5;
        m.pose.orientation.w = -0.5;

        m.lifetime = rclcpp::Duration(0, 0); // forever

        return m;
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    // visualization_msgs::msg::MarkerArray markers_;
    geometry_msgs::msg::PoseArray detected_people;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IconPublisher>());
    rclcpp::shutdown();
    return 0;
}
