#include "panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace rviz_control_panel
{
  ControlPanel::ControlPanel(QWidget *parent)
      : rviz_common::Panel(parent)
  {
    // Scope styles to this panel only
    this->setObjectName("ControlPanelRoot");

    auto *vbox = new QVBoxLayout(this);

    // ---- Status line ----
    {
      auto *hbox = new QHBoxLayout();
      auto *status_label = new QLabel("Status: ");
      status_label->setObjectName("StatusLabel");
      status_ = new QLabel("-");
      status_->setMinimumWidth(200);
      hbox->addWidget(status_label);
      hbox->addWidget(status_);
      hbox->addStretch(1);
      vbox->addLayout(hbox);
    }

    // ---- Inspect location (X/Y + Execute) ----
    // This matches the mockup: label, two inputs, and a button underneath.
    {
      auto *title = new QLabel("Send for patrol to X,Y in map frame (meters):");
      vbox->addSpacing(8);
      vbox->addWidget(title);

      // numeric validator (allows “-” and “.”, no sci notation)
      xy_validator_ = new QDoubleValidator(this);
      xy_validator_->setNotation(QDoubleValidator::StandardNotation);

      auto *grid = new QGridLayout();
      auto *lbl_x = new QLabel("X:");
      auto *lbl_y = new QLabel("Y:");
      edit_x_ = new QLineEdit();
      edit_y_ = new QLineEdit();
      edit_x_->setPlaceholderText("global coordiantes");
      edit_y_->setPlaceholderText("global coordiantes");
      edit_x_->setValidator(xy_validator_);
      edit_y_->setValidator(xy_validator_);
      edit_x_->setMinimumWidth(200);
      edit_y_->setMinimumWidth(200);

      grid->addWidget(lbl_x, 0, 0);
      grid->addWidget(edit_x_, 0, 1);
      grid->addItem(new QSpacerItem(30, 1), 0, 2); // spacing between columns
      grid->addWidget(lbl_y, 0, 3);
      grid->addWidget(edit_y_, 0, 4);

      vbox->addLayout(grid);

      btn_inspect_exec_ = new QPushButton("Execute");
      btn_inspect_exec_->setMinimumHeight(60);
      vbox->addSpacing(10);
      vbox->addWidget(btn_inspect_exec_, /*stretch*/ 0, Qt::AlignHCenter);

      // UX: pressing Enter in either box triggers execute
      connect(edit_x_, &QLineEdit::returnPressed, this, &ControlPanel::onInspectExecute);
      connect(edit_y_, &QLineEdit::returnPressed, this, &ControlPanel::onInspectExecute);
      connect(btn_inspect_exec_, &QPushButton::clicked, this, &ControlPanel::onInspectExecute);
    }

    // ---- Big buttons ----
    btn_estop_ = new QPushButton("E-STOP");
    btn_estop_->setObjectName("Estop");
    btn_estop_->setMinimumHeight(200);
    btn_estop_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    btn_rtb_ = new QPushButton("Return to Base");
    btn_rtb_->setMinimumHeight(100);

    vbox->addSpacing(40);
    vbox->addWidget(btn_estop_);
    vbox->addSpacing(20);
    vbox->addWidget(btn_rtb_);
    vbox->addSpacing(40);

    // ---- Manual control block (unchanged) ----
    cb_manual_control_ = new QCheckBox("Enable Manual Control");
    cb_manual_control_->setChecked(false);
    vbox->addWidget(cb_manual_control_);
    vbox->addSpacing(10);

    manual_control_group_ = new QWidget(this);
    auto *mgrid = new QGridLayout(manual_control_group_);
    btn_forward_ = new QPushButton("↑");
    btn_backward_ = new QPushButton("↓");
    btn_left_ = new QPushButton("←");
    btn_right_ = new QPushButton("→");
    btn_stop_ = new QPushButton("■");
    btn_f_left_ = new QPushButton("↖");
    btn_f_right_ = new QPushButton("↗");
    btn_b_left_ = new QPushButton("↙");
    btn_b_right_ = new QPushButton("↘");
    for (auto btn : {btn_forward_, btn_backward_, btn_left_, btn_right_, btn_stop_,
                     btn_f_left_, btn_f_right_, btn_b_left_, btn_b_right_})
    {
      btn->setMinimumHeight(150);
      btn->setMinimumWidth(150);
      btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
      btn->setEnabled(false);
    }
    manual_control_group_->setVisible(false);

    mgrid->addWidget(btn_forward_, 0, 1);
    mgrid->addWidget(btn_left_, 1, 0);
    mgrid->addWidget(btn_stop_, 1, 1);
    mgrid->addWidget(btn_right_, 1, 2);
    mgrid->addWidget(btn_backward_, 2, 1);
    mgrid->addWidget(btn_f_left_, 0, 0);
    mgrid->addWidget(btn_f_right_, 0, 2);
    mgrid->addWidget(btn_b_left_, 2, 0);
    mgrid->addWidget(btn_b_right_, 2, 2);

    vbox->addWidget(manual_control_group_);
    vbox->addStretch(1);
    setLayout(vbox);

    // --- Signals ---
    connect(btn_estop_, &QPushButton::clicked, this, &ControlPanel::onEstopClicked);
    connect(btn_rtb_, &QPushButton::clicked, this, &ControlPanel::onReturnBaseClicked);
    connect(cb_manual_control_, &QCheckBox::toggled, this, &ControlPanel::onManualControlToggled);

    // --- Styles (add inputs + keep your existing look) ---
    this->setStyleSheet(R"(
    #ControlPanelRoot QPushButton {
      background-color: #3a86ff;
      color: white;
      font-size: 32px;
      font-weight: 600;
      border-radius: 12px;
      padding: 8px;
    }
    #ControlPanelRoot QPushButton:hover { background-color: #2e6dcc; }
    #ControlPanelRoot QPushButton:disabled { background-color: rgb(136,174,231); }
    #ControlPanelRoot QPushButton:pressed { background-color: #244f99; }
    #ControlPanelRoot QPushButton#Estop { background-color: #d00000; }
    #ControlPanelRoot QPushButton#Estop:hover { background-color: #a60000; }

    #ControlPanelRoot QLineEdit {
      font-size: 28px;
      padding: 8px 12px;
      border: 2px solid #222;
      border-radius: 10px;
      min-height: 44px;
    }
    #ControlPanelRoot QLabel {
      color: black;
      font-size: 28px;
    }
    #ControlPanelRoot QLabel#StatusLabel { font-weight: 700; }
    #ControlPanelRoot QCheckBox {
      font-size: 40px;
      font-weight: bold;
    }
    #ControlPanelRoot QCheckBox::indicator {
      width: 28px; height: 28px; border-radius: 6px; border: 2px solid gray; background: white;
    }
    #ControlPanelRoot QCheckBox::indicator:checked {
      background-color: #3a86ff; border-color: #3a86ff;
    }
  )");
  }

  void ControlPanel::onInitialize()
  {
    // Use RViz's shared node so we don't spin our own
    auto node_abs = getDisplayContext()->getRosNodeAbstraction().lock();
    auto node = node_abs->get_raw_node();

    estop_pub_ = node->create_publisher<std_msgs::msg::Bool>("/estop", 10);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");
  }

  // --- E-Stop: publish Bool(true) ---
  void ControlPanel::onEstopClicked()
  {
    if (!estop_pub_)
      return;
    std_msgs::msg::Bool msg;
    msg.data = true;
    estop_pub_->publish(msg);
    status_->setText("E-stop sent");
  }

  // --- Return-to-Base: sends stored home_x_, home_y_, home_yaw_deg_ ---
  void ControlPanel::onReturnBaseClicked()
  {
    if (!nav_client_ || !nav_client_->wait_for_action_server(std::chrono::milliseconds(50)))
    {
      status_->setText("Nav2 action server not ready");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp =
        getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->get_clock()->now();
    goal.pose.pose.position.x = home_x_;
    goal.pose.pose.position.y = home_y_;
    double yaw = home_yaw_deg_ * M_PI / 180.0;
    goal.pose.pose.orientation.z = std::sin(yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(yaw * 0.5);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.result_callback = [this](auto)
    {
      QMetaObject::invokeMethod(status_, [this]
                                { status_->setText("RTB goal reached"); });
    };
    nav_client_->async_send_goal(goal, opts);
    status_->setText("RTB goal sent");
  }

  // --- Manual control toggle (unchanged) ---
  void ControlPanel::onManualControlToggled(bool enabled)
  {
    for (auto btn : {btn_forward_, btn_backward_, btn_left_, btn_right_,
                     btn_stop_, btn_f_left_, btn_f_right_, btn_b_left_, btn_b_right_})
      btn->setEnabled(enabled);

    manual_control_group_->setVisible(enabled);

    if (estop_pub_)
    {
      std_msgs::msg::Bool msg;
      msg.data = true;
      estop_pub_->publish(msg);
    }
    status_->setText("Manual control " + QString(enabled ? "activated" : "disabled"));
  }

  // --- New: Send Nav2 goal to (x,y) from the input boxes ---
  void ControlPanel::onInspectExecute()
  {
    if (!nav_client_)
    {
      status_->setText("Nav2 client not initialized");
      return;
    }
    if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(100)))
    {
      status_->setText("Nav2 action server not ready");
      return;
    }

    bool okx = false, oky = false;
    const double x = edit_x_->text().toDouble(&okx);
    const double y = edit_y_->text().toDouble(&oky);
    if (!okx || !oky)
    {
      status_->setText("Enter valid X/Y (meters in map)");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map"; // Assumes inputs are in map frame
    goal.pose.header.stamp =
        getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->get_clock()->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    // Face forward (yaw=0). You can add a third input later if you want yaw.
    goal.pose.pose.orientation.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.result_callback = [this](auto)
    {
      QMetaObject::invokeMethod(status_, [this]
                                { status_->setText("Inspect goal reached"); });
    };
    nav_client_->async_send_goal(goal, opts);

    status_->setText(QString("Navigating to (%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
  }

  void ControlPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("home_x", home_x_);
    config.mapSetValue("home_y", home_y_);
    config.mapSetValue("home_yaw_deg", home_yaw_deg_);

    // Persist last typed inspect coordinates for convenience
    config.mapSetValue("inspect_x", edit_x_ ? edit_x_->text() : "");
    config.mapSetValue("inspect_y", edit_y_ ? edit_y_->text() : "");
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

    QString sx, sy;
    if (config.mapGetString("inspect_x", &sx) && edit_x_)
      edit_x_->setText(sx);
    if (config.mapGetString("inspect_y", &sy) && edit_y_)
      edit_y_->setText(sy);
  }
} // namespace rviz_control_panel

PLUGINLIB_EXPORT_CLASS(rviz_control_panel::ControlPanel, rviz_common::Panel)
