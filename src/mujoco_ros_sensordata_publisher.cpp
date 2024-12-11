#include <mujoco_ros_sensordata_publisher/mujoco_ros_sensordata_publisher.h>

#include <pluginlib/class_list_macros.h>

namespace mujoco_ros::sensordata_publisher {

SensordataPublisherPlugin::~SensordataPublisherPlugin() {}

bool SensordataPublisherPlugin::load(const mjModel *m, mjData *d)
{
	ROS_INFO_STREAM_NAMED("mujoco_ros_sensordata_publisher", "Loading mujoco_ros_sensordata_publisher plugin ...");

	// Check that ROS has been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_sensordata_publisher",
		                       "A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}

	ROS_ASSERT(rosparam_config_.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	// Check rosparam sanity
	if (rosparam_config_.hasMember("publish_period")) {
		if (rosparam_config_["publish_period"].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
			ROS_ERROR_NAMED("mujoco_ros_sensordata_publisher", "The 'negate_force_torque' param for MujocoRosSensorsPlugin must define a boolean if given");
			return false;
		}
		publish_period_ = ros::Duration((double)rosparam_config_["publish_period"]);
	}
	else
	{
		publish_period_ = ros::Duration(0.0);
	}

	layout_ = std_msgs::MultiArrayLayout();
	layout_.data_offset = 0;

	layout_.dim.push_back(std_msgs::MultiArrayDimension());
	layout_.dim[0].label = "sensordata";
	layout_.dim[0].size = m->nsensordata + 1;
	layout_.dim[0].stride = layout_.dim[0].size;

	publisher_ = node_handle_.advertise<std_msgs::Float64MultiArray>("sensordata", 1, true);

	ROS_INFO("Loaded mujoco_ros_sensordata_publisher");
	return true;
}

void SensordataPublisherPlugin::lastStageCallback(const mjModel *model, mjData *data)
{
	ros::Time sim_time_ros = ros::Time::now();

	ROS_WARN_STREAM_COND_NAMED(sim_time_ros < ros::Time(data->time), "mujoco_ros_sensordata_publisher",
	                           "ROS time not in sync with mjData! (" << sim_time_ros << " < " << ros::Time(data->time)
	                                                                 << ")");
	
	if (sim_time_ros < last_update_sim_time_ros_) {
		ROS_INFO_NAMED("mujoco_ros_sensordata_publisher", "Resetting mujoco_ros_sensordata_publisher due to time reset");
		ROS_DEBUG_STREAM_NAMED("mujoco_ros_sensordata_publisher",
		                       "sim time is " << sim_time_ros << " while last time was " << last_update_sim_time_ros_);
		last_update_sim_time_ros_ = sim_time_ros;
		ROS_WARN_STREAM_COND_NAMED(sim_time_ros < ros::Time::now(), "mujoco_ros_sensordata_publisher",
		                           "Current time moved forward within last stage update! " << sim_time_ros << " -> "
		                                                                                << ros::Time::now());
	}

	ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

	if (sim_period >= publish_period_ || publish_period_.isZero()) {
		last_update_sim_time_ros_ = sim_time_ros;
		auto msg = std_msgs::Float64MultiArray();
		msg.layout = layout_;
		for (int i = 0; i < model->nsensordata; ++i) {
			msg.data.push_back((double) data->sensordata[i]);
		}
		msg.data.push_back((double) data->time);
		publisher_.publish(msg);
	}
}

void SensordataPublisherPlugin::reset() {}

} // mujoco_ros_sensordata_publisher

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensordata_publisher::SensordataPublisherPlugin, mujoco_ros::MujocoPlugin)
