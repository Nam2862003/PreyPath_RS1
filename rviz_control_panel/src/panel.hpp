#pragma once

#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QTextEdit>
#include <QStyleFactory>
#include <QCheckBox>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/display_context.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rviz_control_panel
{

    class ControlPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        ControlPanel(QWidget *parent = nullptr);
        void onInitialize() override; // called when RViz has a context/ROS node
        void save(rviz_common::Config config) const override;
        void load(const rviz_common::Config &config) override;

    private Q_SLOTS:
        void onEstopClicked();
        void onReturnBaseClicked();
        void onManualControlToggled(bool enabled);

    private:
        // UI
        QPushButton *btn_estop_{nullptr};
        QPushButton *btn_rtb_{nullptr};
        QLabel *status_{nullptr};

        QPushButton *btn_forward_{nullptr};
        QPushButton *btn_backward_{nullptr};
        QPushButton *btn_left_{nullptr};
        QPushButton *btn_right_{nullptr};
        QPushButton *btn_stop_{nullptr};
        QPushButton *btn_f_left_{nullptr};
        QPushButton *btn_f_right_{nullptr};
        QPushButton *btn_b_left_{nullptr};
        QPushButton *btn_b_right_{nullptr};

        QCheckBox *cb_manual_control_{nullptr};

        // ROS
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

        // Settings (persisted to .rviz)
        double home_x_{0.0}, home_y_{0.0}, home_yaw_deg_{0.0};
    };

} // namespace rviz_control_panel
