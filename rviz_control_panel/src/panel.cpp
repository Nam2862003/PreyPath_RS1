#include "panel.hpp"

namespace rviz_control_panel
{

  ControlPanel::ControlPanel(QWidget *parent)
      : rviz_common::Panel(parent)
  {
    // Give the panel a unique object name for scoping
    this->setObjectName("ControlPanelRoot");

    // Build layout
    auto *vbox = new QVBoxLayout(this);

    auto *hbox = new QHBoxLayout();
    auto *status_label = new QLabel("Status: ");
    status_label->setObjectName("StatusLabel");

    status_ = new QLabel("-");
    status_->setMinimumWidth(150);

    hbox->addWidget(status_label);
    hbox->addWidget(status_);
    hbox->addStretch(1);
    vbox->addLayout(hbox);

    // Buttons
    btn_estop_ = new QPushButton("E-STOP");
    btn_estop_->setObjectName("Estop"); // special styling
    btn_estop_->setMinimumHeight(100);
    btn_estop_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    btn_rtb_ = new QPushButton("Return to Base");
    btn_rtb_->setMinimumHeight(50);

    vbox->addSpacing(20);
    vbox->addWidget(btn_estop_);
    vbox->addSpacing(5);
    vbox->addWidget(btn_rtb_);
    vbox->addSpacing(20);

    // Manual control checkbox
    cb_manual_control_ = new QCheckBox("Enable Manual Control");
    cb_manual_control_->setChecked(false);
    vbox->addWidget(cb_manual_control_);
    vbox->addSpacing(20);

    // Manual control grid
    manual_control_group_ = new QWidget(this);
    auto *grid = new QGridLayout(manual_control_group_);
    btn_forward_ = new QPushButton("↑");
    btn_backward_ = new QPushButton("↓");
    btn_left_ = new QPushButton("←");
    btn_right_ = new QPushButton("→");
    btn_stop_ = new QPushButton("■");
    btn_f_left_ = new QPushButton("↖");
    btn_f_right_ = new QPushButton("↗");
    btn_b_left_ = new QPushButton("↙");
    btn_b_right_ = new QPushButton("↘");

    for (auto btn : {btn_forward_, btn_backward_, btn_left_, btn_right_,
                     btn_stop_, btn_f_left_, btn_f_right_, btn_b_left_, btn_b_right_})
    {
      btn->setMinimumHeight(50);
      btn->setMinimumWidth(50);
      btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
      btn->setEnabled(false);
    }
    manual_control_group_->setVisible(false);

    grid->addWidget(btn_forward_, 0, 1);
    grid->addWidget(btn_left_, 1, 0);
    grid->addWidget(btn_stop_, 1, 1);
    grid->addWidget(btn_right_, 1, 2);
    grid->addWidget(btn_backward_, 2, 1);
    grid->addWidget(btn_f_left_, 0, 0);
    grid->addWidget(btn_f_right_, 0, 2);
    grid->addWidget(btn_b_left_, 2, 0);
    grid->addWidget(btn_b_right_, 2, 2);

    vbox->addWidget(manual_control_group_);
    vbox->addStretch(1);

    setLayout(vbox);

    // Connectors
    connect(btn_estop_, &QPushButton::clicked, this, &ControlPanel::onEstopClicked);
    connect(btn_rtb_, &QPushButton::clicked, this, &ControlPanel::onReturnBaseClicked);
    connect(cb_manual_control_, &QCheckBox::toggled, this, &ControlPanel::onManualControlToggled);

    // --- Scoped stylesheet applied to this panel only ---
    this->setStyleSheet(R"(
    #ControlPanelRoot QPushButton {
        background-color: #3a86ff;
        color: white;
        font-size: 16px;          /* 32 → 16 */
        font-weight: 600;
        border-radius: 6px;       /* 12 → 6 */
        padding: 4px;             /* 8 → 4 */
    }
    #ControlPanelRoot QPushButton:hover {
        background-color: #2e6dcc;
    }
    #ControlPanelRoot QPushButton:disabled {
        background-color:rgb(136, 174, 231);
    }
    #ControlPanelRoot QPushButton:pressed {
        background-color: #244f99;
    }
    #ControlPanelRoot QPushButton#Estop {
        background-color: #d00000;
    }
    #ControlPanelRoot QPushButton#Estop:hover {
        background-color: #a60000;
    }

    #ControlPanelRoot QCheckBox {
        font-size: 20px;          /* 40 → 20 */
        font-weight: bold;
    }
    #ControlPanelRoot QCheckBox::indicator {
        width: 14px;              /* 28 → 14 */
        height: 14px;             /* 28 → 14 */
        border-radius: 3px;       /* 6 → 3 */
        border: 1px solid gray;   /* 2 → 1 */
        background: white;
    }
    #ControlPanelRoot QCheckBox::indicator:checked {
        background-color: #3a86ff;
        border-color: #3a86ff;
    }

    #ControlPanelRoot QLabel {
        color:rgb(0, 0, 0);
        font-size: 14px;          /* 28 → 14 */
    }
    #ControlPanelRoot QLabel#StatusLabel {
        font-weight: 700;
    }
)");
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
    status_->setText("E-stop sent");
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
                                { status_->setText("RTB goal reached"); });
    };
    nav_client_->async_send_goal(goal, send_goal_options);

    status_->setText("RTB goal sent");
  }

  void ControlPanel::onManualControlToggled(bool enabled)
  {
    for (auto btn : {btn_forward_, btn_backward_, btn_left_, btn_right_, btn_stop_, btn_f_left_, btn_f_right_, btn_b_left_, btn_b_right_})
    {
      btn->setEnabled(enabled);
    }

    manual_control_group_->setVisible(enabled);

    if (!estop_pub_)
      return;
    std_msgs::msg::Bool msg;
    msg.data = true;
    estop_pub_->publish(msg);
    status_->setText("Manual control activated");

    // TODO: Notify ROS about manual control, e.g. send cmd vel to stop autonomous navigation
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
