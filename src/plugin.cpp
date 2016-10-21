#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <urdf/model.h>

namespace gazebo {

class AGV2GazeboPlugin : public ModelPlugin {
public:
  AGV2GazeboPlugin() {}
  ~AGV2GazeboPlugin() {}
  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  virtual void Init();

private:
  void OnUpdate(const common::UpdateInfo& info);

  physics::Link_V links_;
  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  common::Time prevUpdateTime;
  std::string robot_namespace_;

  boost::shared_ptr<ros::NodeHandle> rosnode_;
  ros::Time last_update_time_;
  ros::NodeHandle nh_;
  ros::Time last_publish_;
};

void AGV2GazeboPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  ROS_INFO_STREAM("|Load|get model");
  model_ = parent;
  ROS_INFO_STREAM("|Load|set update");
  update_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&AGV2GazeboPlugin::OnUpdate, this, _1));
  robot_namespace_ = "";
  if (!sdf->HasElement("robotNamespace")) {
    ROS_INFO_STREAM("|Load| not set robot namespace");
  } else {
    robot_namespace_ =
      sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  rosnode_.reset(new ros::NodeHandle(robot_namespace_));
}

void AGV2GazeboPlugin::Init() {
  // Init time stuff
  prevUpdateTime = model_->GetWorld()->GetSimTime();
  last_publish_ = ros::Time(prevUpdateTime.Double());
  links_ = model_->GetLinks();
}

void AGV2GazeboPlugin::OnUpdate(const common::UpdateInfo& info) {
  math::Vector3 Acc;
  if (!ros::ok()) {
    ROS_INFO_STREAM("|OnUpdate| ros not ok");
  } else {
    //do things here
  }
}

GZ_REGISTER_MODEL_PLUGIN(AGV2GazeboPlugin)

} // namespace gazebo
