#ifndef GBPLANNER_UI_H
#define GBPLANNER_UI_H

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#ifndef Q_MOC_RUN
#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <planner_msgs/pci_global.h>
#include <planner_msgs/pci_initialization.h>
#endif

namespace gbplanner_ui {

class gbplanner_panel : public rviz::Panel {
    Q_OBJECT

public:
    explicit gbplanner_panel(QWidget* parent = nullptr);
    virtual void load(const rviz::Config& config) override;
    virtual void save(rviz::Config config) const override;

protected Q_SLOTS:
    void on_start_planner_click();
    void on_stop_planner_click();
    void on_homing_click();
    void on_init_motion_click();
    void on_plan_to_waypoint_click();
    void on_global_planner_click();
    void updateServiceClients(const QString& ns);  // Declaration of new slot

protected:
    QPushButton* button_start_planner;
    QPushButton* button_stop_planner;
    QPushButton* button_homing;
    QPushButton* button_init_motion;
    QPushButton* button_plan_to_waypoint;
    QPushButton* button_global_planner;
    QLineEdit* global_id_line_edit;

    ros::ServiceClient planner_client_start_planner;
    ros::ServiceClient planner_client_stop_planner;
    ros::ServiceClient planner_client_homing;
    ros::ServiceClient planner_client_init_motion;
    ros::ServiceClient planner_client_plan_to_waypoint;
    ros::ServiceClient planner_client_global_planner;

    ros::NodeHandle nh;
};

}  // namespace gbplanner_ui

#endif  // GBPLANNER_UI_H
