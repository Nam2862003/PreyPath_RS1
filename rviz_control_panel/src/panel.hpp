#pragma once

#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QCheckBox>
#include <QLineEdit>
#include <QDoubleValidator>
#include <QGroupBox>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

// (Keep the pluginlib include in the .cpp where you EXPORT_CLASS)
#include <pluginlib/class_list_macros.hpp>

namespace rviz_control_panel
{

  class ControlPanel : public rviz_common::Panel
  {
    Q_OBJECT

  public:
    explicit ControlPanel(QWidget *parent = nullptr);
    void onInitialize() override; // called when RViz has a context/ROS node
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config &config) override;

  private Q_SLOTS:
    void onEstopClicked();
    void onReturnBaseClicked();
    void onManualControlToggled(bool enabled);

    // New: execute "Inspect location" (send Nav2 goal to typed X/Y)
    void onInspectExecute();

  private:
    // -------- UI: status + main controls --------
    QLabel *status_{nullptr};
    QLabel *comms_{nullptr};
    QLabel *detection_{nullptr};
    QPushButton *btn_estop_{nullptr};
    QPushButton *btn_rtb_{nullptr};

    // -------- UI: Inspect location (X/Y + Execute) --------
    QLineEdit *edit_x_{nullptr};
    QLineEdit *edit_y_{nullptr};
    QLineEdit *edit_r_{nullptr};
    QPushButton *btn_inspect_exec_{nullptr};
    QDoubleValidator *num_validator_{nullptr};

    // -------- UI: Manual control grid --------
    QCheckBox *cb_manual_control_{nullptr};
    QWidget *manual_control_group_{nullptr};
    QPushButton *btn_forward_{nullptr};
    QPushButton *btn_backward_{nullptr};
    QPushButton *btn_left_{nullptr};
    QPushButton *btn_right_{nullptr};
    QPushButton *btn_stop_{nullptr};
    QPushButton *btn_f_left_{nullptr};
    QPushButton *btn_f_right_{nullptr};
    QPushButton *btn_b_left_{nullptr};
    QPushButton *btn_b_right_{nullptr};

    // -------- ROS interfaces --------
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // Behavior controller interface (new): publish traverse goals & receive status
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traverse_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_status_sub_;
  // New: Return-to-base publisher (PoseStamped) handled by behavior controller
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rtb_pub_;
  // Manual control publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_pub_;

  // Helper to publish a manual velocity command
  void publishManualCmd(double lin_x, double ang_z);

    // -------- Settings (persisted to RViz config) --------
    double home_x_{0.0};
    double home_y_{0.0};
    double home_yaw_deg_{0.0};
  };

} // namespace rviz_control_panel
