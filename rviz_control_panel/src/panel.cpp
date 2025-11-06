#include "panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace rviz_control_panel
{
  ControlPanel::ControlPanel(QWidget *parent)
      : rviz_common::Panel(parent)
  {
    // initialize detection timer reference time
    last_detection_time_ = std::chrono::steady_clock::now();
    // Scope styles to this panel only
    this->setObjectName("ControlPanelRoot");

    auto *vbox_total = new QVBoxLayout(this);

    // ---- Status reports ----
    {
      auto *grid = new QGridLayout();
      // auto *hbox = new QHBoxLayout();
      auto *status_label = new QLabel("Status: ");
      status_label->setObjectName("StatusLabel");
      status_ = new QLabel("-");
      status_->setMinimumWidth(200);

      auto *comms_label = new QLabel("Comms: ");
      comms_label->setObjectName("StatusLabel");
      comms_ = new QLabel("Simulation started.");
      comms_->setMinimumWidth(200);

      auto *detection_label = new QLabel("Detection: ");
      detection_label->setObjectName("StatusLabel");
      detection_ = new QLabel("No people detected.");
      detection_->setMinimumWidth(200);

      grid->addWidget(status_label, 0, 0);
      grid->addWidget(status_, 0, 1);
      grid->addWidget(comms_label, 1, 0);
      grid->addWidget(comms_, 1, 1);
      grid->addWidget(detection_label, 2, 0);
      grid->addWidget(detection_, 2, 1);

      grid->setColumnStretch(1, 1); // stretch second column

      auto *group = new QGroupBox("Status reports"); // title on the frame
      group->setLayout(grid);
      vbox_total->addWidget(group);
      vbox_total->addSpacing(10);
    }

    // ---- Patrol settings ----
    {

      auto *vbox = new QVBoxLayout();
      auto *title = new QLabel("Send for patrol to X,Y in map frame (meters):");
      vbox->addWidget(title);

      auto *hbox_patrol = new QHBoxLayout();

      // numeric validator (allows “-” and “.”, no sci notation)
      num_validator_ = new QDoubleValidator(this);
      num_validator_->setNotation(QDoubleValidator::StandardNotation);

      auto *grid = new QGridLayout();
      auto *lbl_x = new QLabel("X:");
      auto *lbl_y = new QLabel("Y:");
      auto *lbl_r = new QLabel("L:");
      lbl_x->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      lbl_y->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      lbl_r->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      edit_x_ = new QLineEdit();
      edit_y_ = new QLineEdit();
      edit_r_ = new QLineEdit();
      edit_x_->setPlaceholderText("global coordiantes");
      edit_y_->setPlaceholderText("global coordiantes");
      edit_r_->setPlaceholderText("patrol area");
      edit_x_->setValidator(num_validator_);
      edit_y_->setValidator(num_validator_);
      edit_r_->setValidator(num_validator_);
      edit_x_->setMinimumWidth(150);
      edit_y_->setMinimumWidth(150);
      edit_r_->setMinimumWidth(150);

      grid->addWidget(lbl_x, 0, 0);
      grid->addWidget(edit_x_, 0, 1);
      grid->addWidget(lbl_y, 1, 0);
      grid->addWidget(edit_y_, 1, 1);
      grid->addWidget(lbl_r, 2, 0);
      grid->addWidget(edit_r_, 2, 1);

      hbox_patrol->addLayout(grid);
      hbox_patrol->addSpacing(10);

      btn_inspect_exec_ = new QPushButton("Execute");
      btn_inspect_exec_->setMinimumHeight(40);
      btn_inspect_exec_->setMinimumWidth(120);
      btn_inspect_exec_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

      vbox->addSpacing(10);
      // hbox_patrol->addWidget(btn_inspect_exec_, /*stretch*/ 0, Qt::AlignHCenter);
      hbox_patrol->addWidget(btn_inspect_exec_);

      vbox->addLayout(hbox_patrol);

      // UX: pressing Enter in either box triggers execute
      connect(edit_x_, &QLineEdit::returnPressed, this, &ControlPanel::onInspectExecute);
      connect(edit_y_, &QLineEdit::returnPressed, this, &ControlPanel::onInspectExecute);
      connect(btn_inspect_exec_, &QPushButton::clicked, this, &ControlPanel::onInspectExecute);

      auto *group = new QGroupBox("Patrol Settings"); // title on the frame
      group->setLayout(vbox);
      vbox_total->addWidget(group);
      vbox_total->addSpacing(10);
    }

    // ---- Override ----

    {

      auto *vbox = new QVBoxLayout();

      auto *hbox = new QHBoxLayout();
      btn_estop_ = new QPushButton("E-STOP");
      btn_estop_->setObjectName("Estop");
      btn_estop_->setMinimumHeight(80);
      btn_estop_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

      btn_rtb_ = new QPushButton("Return to Base");
      // btn_rtb_->setMinimumHeight(50);
      btn_rtb_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

      hbox->addWidget(btn_rtb_);
      hbox->addSpacing(10);
      hbox->addWidget(btn_estop_);
      vbox->addLayout(hbox);
      vbox->addSpacing(20);

      // ---- Manual control block (unchanged) ----
      cb_manual_control_ = new QCheckBox("Enable Manual Control");
      cb_manual_control_->setChecked(false);
      vbox->addWidget(cb_manual_control_);

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
        btn->setMinimumHeight(80);
        btn->setMinimumWidth(80);
        btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        btn->setEnabled(false);
      }

      mgrid->addWidget(btn_forward_, 0, 1);
      mgrid->addWidget(btn_left_, 1, 0);
      mgrid->addWidget(btn_stop_, 1, 1);
      mgrid->addWidget(btn_right_, 1, 2);
      mgrid->addWidget(btn_backward_, 2, 1);
      mgrid->addWidget(btn_f_left_, 0, 0);
      mgrid->addWidget(btn_f_right_, 0, 2);
      mgrid->addWidget(btn_b_left_, 2, 0);
      mgrid->addWidget(btn_b_right_, 2, 2);

      mgrid->setContentsMargins(0, 0, 0, 0);
      mgrid->setHorizontalSpacing(2); // or 0 if you want them touching
      mgrid->setVerticalSpacing(2);
      mgrid->setSizeConstraint(QLayout::SetFixedSize);

      manual_control_group_->setVisible(false);
      manual_control_group_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

      vbox->addWidget(manual_control_group_, /*stretch*/ 0, Qt::AlignHCenter);
      vbox->addStretch(1);
      setLayout(vbox);

      auto *group = new QGroupBox("User override"); // title on the frame
      group->setLayout(vbox);

      vbox_total->addWidget(group);
      vbox_total->addSpacing(10);

      // --- Signals ---
      connect(btn_estop_, &QPushButton::clicked, this, &ControlPanel::onEstopClicked);
      connect(btn_rtb_, &QPushButton::clicked, this, &ControlPanel::onReturnBaseClicked);
      connect(cb_manual_control_, &QCheckBox::toggled, this, &ControlPanel::onManualControlToggled);

      // Manual movement button bindings (single-shot commands)
      const double V = 0.4;      // linear speed m/s
      const double W = 1.0;      // angular speed rad/s
      const double VF = 0.7 * V; // diagonal blend
      const double WF = 0.7 * W;
      connect(btn_forward_, &QPushButton::clicked, this, [this, V]
              { publishManualCmd(V, 0.0); });
      connect(btn_backward_, &QPushButton::clicked, this, [this, V]
              { publishManualCmd(-V, 0.0); });
      connect(btn_left_, &QPushButton::clicked, this, [this, W]
              { publishManualCmd(0.0, +W); });
      connect(btn_right_, &QPushButton::clicked, this, [this, W]
              { publishManualCmd(0.0, -W); });
      connect(btn_stop_, &QPushButton::clicked, this, [this]
              { publishManualCmd(0.0, 0.0); });
      connect(btn_f_left_, &QPushButton::clicked, this, [this, VF, WF]
              { publishManualCmd(VF, +WF); });
      connect(btn_f_right_, &QPushButton::clicked, this, [this, VF, WF]
              { publishManualCmd(VF, -WF); });
      connect(btn_b_left_, &QPushButton::clicked, this, [this, VF, WF]
              { publishManualCmd(-VF, -WF); });
      connect(btn_b_right_, &QPushButton::clicked, this, [this, VF, WF]
              { publishManualCmd(-VF, +WF); });
    }

    // --- Styles (add inputs + keep your existing look) ---
    this->setStyleSheet(R"(
#ControlPanelRoot QPushButton {
  background-color: #3a86ff;
  color: white;
  font-size: 16px;          /* 32 → 16 */
  font-weight: 600;
  border-radius: 6px;       /* 12 → 6 */
  padding: 4px;             /* 8 → 4 */
}
#ControlPanelRoot QPushButton:hover { background-color: #2e6dcc; }
#ControlPanelRoot QPushButton:disabled { background-color: rgb(136,174,231); }
#ControlPanelRoot QPushButton:pressed { background-color: #244f99; }
#ControlPanelRoot QPushButton#Estop { background-color: #d00000; }
#ControlPanelRoot QPushButton#Estop:hover { background-color: #a60000; }

#ControlPanelRoot QLineEdit {
  font-size: 14px;          /* 28 → 14 */
  padding: 4px 6px;         /* 8x12 → 4x6 */
  border: 1px solid #222;   /* 2 → 1 */
  border-radius: 5px;       /* 10 → 5 */
  min-height: 22px;         /* 44 → 22 */
}
#ControlPanelRoot QLabel {
  color: black;
  font-size: 14px;          /* 28 → 14 */
}
#ControlPanelRoot QLabel#StatusLabel { font-weight: 700; }
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

#ControlPanelRoot QGroupBox {
  font-size: 14px;          /* 32 → 16 */
  font-weight: bold;
  border: 2px solid gray;   /* 4 → 2 */
  border-radius: 6px;       /* 12 → 6 */
  margin-top: 10px;
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
    // New: publisher to behavior controller for traverse goals
    traverse_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/behavior/traverse_goal", 10);
    rtb_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/behavior/return_to_base", 10);
    manual_enable_pub_ = node->create_publisher<std_msgs::msg::Bool>("/behavior/manual_enable", 10);
    manual_cmd_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/behavior/manual_cmd", 10);
    // Publish inspection length (L) to behavior controller
    inspect_len_pub_ = node->create_publisher<std_msgs::msg::Float64>("/behavior/inspect_length", 10);
    // Subscribe to behavior status (mode only) and comms (human-readable comments)
    behavior_status_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/behavior/status", 10, [this](std_msgs::msg::String::ConstSharedPtr msg)
        {
          const QString mode = QString::fromStdString(msg->data).trimmed();
          QMetaObject::invokeMethod(status_, [this, mode] { status_->setText(mode); }); });

    behavior_comms_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/behavior/comms", 10, [this](std_msgs::msg::String::ConstSharedPtr msg)
        {
          const QString text = QString::fromStdString(msg->data);
          QMetaObject::invokeMethod(comms_, [this, text] { comms_->setText(text); }); });

    // Human detection
    // --- Timer to reset detection label if no updates recently ---
    detection_reset_timer_ = node->create_wall_timer(
        std::chrono::seconds(2),
        [this]()
        {
          const auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::seconds>(
                  now - last_detection_time_)
                  .count() > 2)
          {
            QMetaObject::invokeMethod(detection_, [this]
                                      { detection_->setText("No people detected."); });
          }
        });

    human_pose_sub_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/human_pose", 10,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
        {
          // Format nicely for UI
          const QString info = QString("Human detected at (%1, %2)")
                                   .arg(msg->point.x, 0, 'f', 2)
                                   .arg(msg->point.y, 0, 'f', 2);

          QMetaObject::invokeMethod(detection_, [this, info]
                                    { detection_->setText(info); });
        });
  }

  // --- E-Stop: publish Bool(true) ---
  void ControlPanel::onEstopClicked()
  {
    if (!estop_pub_)
      return;
    std_msgs::msg::Bool msg;
    msg.data = true;
    estop_pub_->publish(msg);
    comms_->setText("E-stop sent");

    resetInspectPlaceholders(edit_x_, edit_y_, edit_r_);
  }

  // --- Return-to-Base: sends stored home_x_, home_y_, home_yaw_deg_ ---
  void ControlPanel::onReturnBaseClicked()
  {
    // Guard: do not allow RTB while manual is enabled
    if (cb_manual_control_ && cb_manual_control_->isChecked())
    {
      comms_->setText("RTB disabled in Manual mode");
      return;
    }
    if (!rtb_pub_)
    {
      comms_->setText("RTB publisher not ready");
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp =
        getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->get_clock()->now();
    pose.pose.position.x = home_x_;
    pose.pose.position.y = home_y_;
    double yaw = home_yaw_deg_ * M_PI / 180.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    rtb_pub_->publish(pose);
    comms_->setText("RTB goal published to controller");

    resetInspectPlaceholders(edit_x_, edit_y_, edit_r_);
  }

  // --- Manual control toggle (unchanged) ---
  void ControlPanel::onManualControlToggled(bool enabled)
  {
    for (auto btn : {btn_forward_, btn_backward_, btn_left_, btn_right_,
                     btn_stop_, btn_f_left_, btn_f_right_, btn_b_left_, btn_b_right_})
      btn->setEnabled(enabled);

    manual_control_group_->setVisible(enabled);
    // Publish manual enable flag
    if (manual_enable_pub_)
    {
      std_msgs::msg::Bool b;
      b.data = enabled;
      manual_enable_pub_->publish(b);
    }
    // UI policy: While Manual is ON, disable autonomous goal inputs and RTB
    const char *tooltip = "Disabled while Manual Control is enabled";
    if (edit_x_)
    {
      edit_x_->setEnabled(!enabled);
      edit_x_->setToolTip(enabled ? tooltip : "");
    }
    if (edit_y_)
    {
      edit_y_->setEnabled(!enabled);
      edit_y_->setToolTip(enabled ? tooltip : "");
    }
    if (edit_r_)
    {
      edit_r_->setEnabled(!enabled);
      edit_r_->setToolTip(enabled ? tooltip : "");
    }
    if (btn_inspect_exec_)
    {
      btn_inspect_exec_->setEnabled(!enabled);
      btn_inspect_exec_->setToolTip(enabled ? tooltip : "");
    }
    if (btn_rtb_)
    {
      btn_rtb_->setEnabled(!enabled);
      btn_rtb_->setToolTip(enabled ? tooltip : "");
    }
    // Safety: send zero twist when disabling
    if (!enabled)
      publishManualCmd(0.0, 0.0);
    // Visible feedback about traverse gating
    if (enabled)
    {
      comms_->setText("Manual control activated. Traverse/RTB disabled");
      if (status_)
        status_->setText("MANUAL");
    }
    else
    {
      comms_->setText("Manual control disabled. You can set traverse/RTB goals");
    }

    resetInspectPlaceholders(edit_x_, edit_y_, edit_r_);
  }

  void ControlPanel::publishManualCmd(double lin, double ang)
  {
    if (!manual_cmd_pub_ || !cb_manual_control_ || !cb_manual_control_->isChecked())
      return; // Only send if manual enabled
    geometry_msgs::msg::Twist t;
    t.linear.x = lin;
    t.angular.z = ang;
    manual_cmd_pub_->publish(t);
  }

  // --- New: Send Nav2 goal to (x,y) from the input boxes ---
  void ControlPanel::onInspectExecute()
  {

    // Guard: do not allow traverse while manual is enabled
    if (cb_manual_control_ && cb_manual_control_->isChecked())
    {
      comms_->setText("Traverse disabled in Manual mode");
      return;
    }
    bool okx = false, oky = false;
    const double x = edit_x_->text().toDouble(&okx);
    const double y = edit_y_->text().toDouble(&oky);
    bool okl = false;
    const double L = edit_r_ ? edit_r_->text().toDouble(&okl) : 0.0;
    if (!okx || !oky)
    {
      comms_->setText("Enter valid X/Y (meters in map)");
      return;
    }
    if (!traverse_pub_)
    {
      comms_->setText("Traverse publisher not ready");
      return;
    }
    // If L provided and positive, publish it so controller plans patrol after arrival
    if (inspect_len_pub_ && okl && L > 0.0)
    {
      std_msgs::msg::Float64 lmsg;
      lmsg.data = L;
      inspect_len_pub_->publish(lmsg);
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp =
        getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->get_clock()->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    traverse_pub_->publish(pose);
    if (okl && L > 0.0)
    {
      comms_->setText(QString("Traverse goal published (%1, %2), then patrol L=%3 m")
                          .arg(x, 0, 'f', 2)
                          .arg(y, 0, 'f', 2)
                          .arg(L, 0, 'f', 2));
    }
    else
    {
      comms_->setText(QString("Traverse goal published (%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
    }

    resetInspectPlaceholders(edit_x_, edit_y_, edit_r_);
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

  void resetInspectPlaceholders(QLineEdit *edit_x_, QLineEdit *edit_y_, QLineEdit *edit_r_)
  {
    edit_x_->clear();
    edit_y_->clear();
    edit_r_->clear();
    edit_x_->setPlaceholderText("global coordiantes");
    edit_y_->setPlaceholderText("global coordiantes");
    edit_r_->setPlaceholderText("patrol area");
  }
} // namespace rviz_control_panel

PLUGINLIB_EXPORT_CLASS(rviz_control_panel::ControlPanel, rviz_common::Panel)
