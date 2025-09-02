#include "panel.hpp"

#include <QHBoxLayout>
#include <QGridLayout>
#include <QVariant>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rviz_control_panel
{

    ControlPanel::ControlPanel(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        auto *root = new QWidget(this);
        auto *v = new QVBoxLayout(root);

        status_ = new QLabel("Status: idle");
        btn_estop_ = new QPushButton("E-stop");
        btn_rtb_ = new QPushButton("Return to Base");

        v->addWidget(status_);
        v->addWidget(btn_estop_);
        v->addWidget(btn_rtb_);
        v->addStretch(1);
        setLayout(v);

        connect(btn_estop_, &QPushButton::clicked, this, &ControlPanel::onEstopClicked);
        connect(btn_rtb_, &QPushButton::clicked, this, &ControlPanel::onReturnBaseClicked);
    }

    void ControlPanel::onInitialize()
    {
        // Get RViz’s shared rclcpp::Node
        auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
        auto raw_node = ros_node_abstraction->get_raw_node();

        // Publisher for E-stop
        estop_pub_ = raw_node->create_publisher<std_msgs::msg::Bool>("/estop", 10);

        // Nav2 NavigateToPose action client
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(raw_node, "navigate_to_pose");
    }

    void ControlPanel::onEstopClicked()
    {
        if (!estop_pub_)
            return;
        std_msgs::msg::Bool msg;
        msg.data = true;
        estop_pub_->publish(msg);
        status_->setText("Status: E-stop sent");
    }

    void ControlPanel::onReturnBaseClicked()
    {
        if (!nav_client_)
            return;

        if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(50)))
        {
            status_->setText("Status: Nav2 action server not ready");
            return;
        }

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->get_clock()->now();
        goal.pose.pose.position.x = home_x_;
        goal.pose.pose.position.y = home_y_;
        // yaw->quat (Z-only)
        double yaw_rad = home_yaw_deg_ * M_PI / 180.0;
        goal.pose.pose.orientation.z = std::sin(yaw_rad * 0.5);
        goal.pose.pose.orientation.w = std::cos(yaw_rad * 0.5);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto)
        {
            QMetaObject::invokeMethod(status_, [this]()
                                      { status_->setText("Status: RTB goal done"); });
        };
        nav_client_->async_send_goal(goal, send_goal_options);

        status_->setText("Status: RTB goal sent");
    }

    void ControlPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("home_x", home_x_);
        config.mapSetValue("home_y", home_y_);
        config.mapSetValue("home_yaw_deg", home_yaw_deg_);
    }

    void ControlPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
        float v;
        if (config.mapGetFloat("home_x", &v))
            home_x_ = v;
        if (config.mapGetFloat("home_y", &v))
            home_y_ = v;
        if (config.mapGetFloat("home_yaw_deg", &v))
            home_yaw_deg_ = v;
    }

} // namespace rviz_control_panel

PLUGINLIB_EXPORT_CLASS(rviz_control_panel::ControlPanel, rviz_common::Panel)
