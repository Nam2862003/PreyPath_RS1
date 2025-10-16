#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using visualization_msgs::msg::Marker;

class IconPublisher : public rclcpp::Node
{
public:
    IconPublisher() : Node("icon_publisher")
    {

        // Transient local keeps last message for late subscribers (RViz)
        auto qos = rclcpp::QoS(1).transient_local().reliable();
        pub_ = create_publisher<Marker>("icons", qos);

        publishBaseIcon();
        // Example: publish more icons later (detections, etc.)
        // publishIcon("hotspot", 1, "package://my_visual_icons/meshes/detection_hotspot.dae", 12.3, 4.5, 0.06, 0.8);
    }

    void publishBaseIcon()
    {
        publishIcon("base", 0,
                    "package://visual_icons/meshes/blend1.glb",
                    0.0, 0.0, 0.1, // x,y,z (slightly above ground to avoid z-fighting)
                    3.0            // scale in meters (uniform)
        );

        // publishIcon("base", 0,
        //             "file:///home/greese/41068_ws/src/visual_icons/meshes/home1.glb",
        //             5.0, 5.0, 0.06, // x,y,z (slightly above ground to avoid z-fighting)
        //             1               // scale in meters (uniform)
        // );
    }

    void publishIcon(const std::string &ns, int id, const std::string &mesh_uri,
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
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;

        // Set scale to resize the mesh (xyz all set)
        m.scale.x = m.scale.y = m.scale.z = scale;

        // If your DAE uses embedded material/texture, alpha still must be >0
        m.color.a = 1.0;
        m.color.r = m.color.g = m.color.b = 1.0; // ignored if embedded materials
        // m.color.r = 1.0;
        // m.color.g = 0.0;
        // m.color.b = 0.0;

        m.pose.orientation.x = -0.5; // 90 deg around X axis
        m.pose.orientation.y = 0.5;  // to
        m.pose.orientation.z = 0.5;
        m.pose.orientation.w = -0.5;

        m.lifetime = rclcpp::Duration(0, 0); // forever
        pub_->publish(m);
    }

private:
    rclcpp::Publisher<Marker>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IconPublisher>());
    rclcpp::shutdown();
    return 0;
}
