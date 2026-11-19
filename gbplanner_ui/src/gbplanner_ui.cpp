#include "gbplanner_ui.h"
#include <QVBoxLayout>
#include <QLabel>

namespace gbplanner_ui {

gbplanner_panel::gbplanner_panel(QWidget* parent) : rviz::Panel(parent) {
    QVBoxLayout* layout = new QVBoxLayout;

    // Namespace selector
    QComboBox* namespace_selector = new QComboBox();
    namespace_selector->addItem("red");
    namespace_selector->addItem("blue");
    layout->addWidget(new QLabel("Select Namespace:"));
    layout->addWidget(namespace_selector);

    // Initialize buttons and connect signals to slots
    button_start_planner = new QPushButton("Start Planner");
    button_stop_planner = new QPushButton("Stop Planner");
    button_homing = new QPushButton("Go Home");
    button_init_motion = new QPushButton("Initialization");
    button_plan_to_waypoint = new QPushButton("Plan to Waypoint");
    button_global_planner = new QPushButton("Run Global");
    global_id_line_edit = new QLineEdit();

    layout->addWidget(button_start_planner);
    layout->addWidget(button_stop_planner);
    layout->addWidget(button_homing);
    layout->addWidget(button_init_motion);
    layout->addWidget(button_plan_to_waypoint);
    layout->addWidget(button_global_planner);
    layout->addWidget(global_id_line_edit);

    setLayout(layout);

    connect(button_start_planner, &QPushButton::clicked, this, &gbplanner_panel::on_start_planner_click);
    connect(button_stop_planner, &QPushButton::clicked, this, &gbplanner_panel::on_stop_planner_click);
    connect(button_homing, &QPushButton::clicked, this, &gbplanner_panel::on_homing_click);
    connect(button_init_motion, &QPushButton::clicked, this, &gbplanner_panel::on_init_motion_click);
    connect(button_plan_to_waypoint, &QPushButton::clicked, this, &gbplanner_panel::on_plan_to_waypoint_click);
    connect(button_global_planner, &QPushButton::clicked, this, &gbplanner_panel::on_global_planner_click);

    connect(namespace_selector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, namespace_selector](int index) {
                this->updateServiceClients(namespace_selector->currentText());
            });

    // Bind clients for the default selection so the buttons work immediately.
    updateServiceClients(namespace_selector->currentText());
}

void gbplanner_panel::updateServiceClients(const QString& ns) {
    std::string base_service_path = "/" + ns.toStdString() + "/planner_control_interface/";
    planner_client_start_planner = nh.serviceClient<std_srvs::Trigger>(base_service_path + "std_srvs/automatic_planning");
    planner_client_stop_planner = nh.serviceClient<std_srvs::Trigger>(base_service_path + "std_srvs/stop");
    planner_client_homing = nh.serviceClient<std_srvs::Trigger>(base_service_path + "std_srvs/homing_trigger");
    planner_client_init_motion = nh.serviceClient<planner_msgs::pci_initialization>("/" + ns.toStdString() + "/pci_initialization_trigger");
    planner_client_plan_to_waypoint = nh.serviceClient<std_srvs::Trigger>(base_service_path + "std_srvs/go_to_waypoint");
    planner_client_global_planner = nh.serviceClient<planner_msgs::pci_global>("/" + ns.toStdString() + "/pci_global");
}

void gbplanner_panel::on_start_planner_click() {
    std_srvs::Trigger srv;
    if (!planner_client_start_planner.call(srv)) {
        ROS_ERROR("Failed to call start planner service");
    }
}

void gbplanner_panel::on_stop_planner_click() {
    std_srvs::Trigger srv;
    if (!planner_client_stop_planner.call(srv)) {
        ROS_ERROR("Failed to call stop planner service");
    }
}

void gbplanner_panel::on_homing_click() {
    std_srvs::Trigger srv;
    if (!planner_client_homing.call(srv)) {
        ROS_ERROR("Failed to call homing service");
    }
}

void gbplanner_panel::on_init_motion_click() {
    planner_msgs::pci_initialization srv;
    if (!planner_client_init_motion.call(srv)) {
        ROS_ERROR("Failed to call initialization service");
    }
}

void gbplanner_panel::on_plan_to_waypoint_click() {
    std_srvs::Trigger srv;
    if (!planner_client_plan_to_waypoint.call(srv)) {
        ROS_ERROR("Failed to call plan to waypoint service");
    }
}

void gbplanner_panel::on_global_planner_click() {
    planner_msgs::pci_global srv;
    srv.request.id = global_id_line_edit->text().toInt();
    if (!planner_client_global_planner.call(srv)) {
        ROS_ERROR("Failed to call global planner service");
    }
}

void gbplanner_panel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
}

void gbplanner_panel::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

}  // namespace gbplanner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gbplanner_ui::gbplanner_panel, rviz::Panel)