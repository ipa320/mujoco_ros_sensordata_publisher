#pragma once

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>

namespace mujoco_ros::sensordata_publisher {

class SensordataPublisherPlugin : public mujoco_ros::MujocoPlugin
{
public:
	virtual ~SensordataPublisherPlugin();

	bool load(const mjModel *m, mjData *d) override;

	void reset() override;

    void lastStageCallback(const mjModel *model, mjData *data) override;

protected:
    std_msgs::MultiArrayLayout layout_;

    ros::Publisher publisher_;

    // Mujoco model and data pointers
    mjModel *m_;
    mjModel *d_;

    // Timing
    ros::Duration publish_period_;
    ros::Time last_update_sim_time_ros_;
};

} // namespace mujoco_ros_sensordata_publisher
