#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>

using visualization_msgs::msg::Marker;

class IconPublisher : public rclcpp::Node
{
public:
    IconPublisher() : Node("icon_publisher")
    {
        auto qos = rclcpp::QoS(1).transient_local().reliable();
        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("icons", qos);

        sub_human_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/human_pose", 10,
            std::bind(&IconPublisher::humanCallback, this, std::placeholders::_1));

        sub_hunter_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/hunter_pose", 10,
            std::bind(&IconPublisher::hunterCallback, this, std::placeholders::_1));

        // Timer to check if human detection timed out
        timer_ = this->create_wall_timer(std::chrono::seconds(2),
            std::bind(&IconPublisher::checkTimeout, this));

        publishHomeIcon();
        proximity_thresh_ = 0.6; // meters (kept but no longer required for suppression)
        suppress_window_sec_ = 1.0; // while hunter is recent, suppress human icon
    }

private:
    void publishHomeIcon()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(produceIcon(
            "base", 0, "package://visual_icons/meshes/home_green.glb",
            0.0, 0.0, 0.0, 3.0, "map"));
        pub_->publish(marker_array);
    }

    void humanCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        last_human_time_ = this->now();
        last_human_pose_ = *msg;

        // If hunter is currently in sight (recent), suppress human icon entirely
        bool hunter_recent = hunter_present_ && (this->now() - last_hunter_time_).seconds() <= suppress_window_sec_;
        if (hunter_recent)
        {
            // Ensure any previous human icon is removed
            if (human_present_)
            {
                visualization_msgs::msg::MarkerArray del_array;
                Marker del;
                del.header.frame_id = "map";
                del.header.stamp = this->now();
                del.ns = "human";
                del.id = 1;
                del.action = Marker::DELETE;
                del_array.markers.push_back(del);
                pub_->publish(del_array);
                human_present_ = false;
            }
            return; // do not publish human while hunter is present
        }

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(produceIcon(
            "human", 1, "package://visual_icons/meshes/person.glb",
            msg->point.x, msg->point.y, 0.0, 3.0, "map"));
        pub_->publish(marker_array);

        // RCLCPP_INFO(this->get_logger(), "Human icon added at (%.2f, %.2f)", msg->point.x, msg->point.y);
        human_present_ = true;
    }

    void hunterCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        last_hunter_time_ = this->now();

        visualization_msgs::msg::MarkerArray marker_array;
        // If a human icon is present, remove it immediately when hunter is in sight
        if (human_present_)
        {
            Marker del;
            del.header.frame_id = "map";
            del.header.stamp = this->now();
            del.ns = "human";
            del.id = 1;
            del.action = Marker::DELETE;
            marker_array.markers.push_back(del);
            human_present_ = false;
        }

        marker_array.markers.push_back(produceIcon(
            "hunter", 2, "package://visual_icons/meshes/hunter_eye.glb",
            msg->point.x, msg->point.y, 0.0, 3.0, "map"));

        // Save the first-seen hunter as a permanent marker
        if (!hunter_saved_)
        {
            saved_hunter_pose_ = *msg;
            hunter_saved_ = true;
            marker_array.markers.push_back(produceIcon(
                "hunter_saved", 100, "package://visual_icons/meshes/hunter_eye.glb",
                msg->point.x, msg->point.y, 0.0, 3.0, "map"));
        }
        pub_->publish(marker_array);

        hunter_present_ = true;
    }

    void checkTimeout()
    {
        if (human_present_ && (this->now() - last_human_time_).seconds() > 1.0)
        {
            // No human detected recently → delete marker
            visualization_msgs::msg::MarkerArray marker_array;
            Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();
            m.ns = "human";
            m.id = 1;
            m.action = Marker::DELETE;
            marker_array.markers.push_back(m);
            pub_->publish(marker_array);
            // RCLCPP_INFO(this->get_logger(), "Human icon removed (timeout)");
            human_present_ = false;
        }

        if (hunter_present_ && (this->now() - last_hunter_time_).seconds() > 1.0)
        {
            // No hunter detected recently → delete marker
            visualization_msgs::msg::MarkerArray marker_array;
            Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();
            m.ns = "hunter";
            m.id = 2;
            m.action = Marker::DELETE;
            marker_array.markers.push_back(m);
            pub_->publish(marker_array);
            hunter_present_ = false;
        }
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
        m.lifetime = rclcpp::Duration(0, 0);  // persistent until deleted
        return m;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_human_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_hunter_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_human_time_;
    bool human_present_ = false;
    rclcpp::Time last_hunter_time_;
    bool hunter_present_ = false;
    geometry_msgs::msg::PointStamped last_human_pose_;
    double proximity_thresh_;
    double suppress_window_sec_;
    // Persistent first-hunter storage
    bool hunter_saved_ = false;
    geometry_msgs::msg::PointStamped saved_hunter_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IconPublisher>());
    rclcpp::shutdown();
    return 0;
}