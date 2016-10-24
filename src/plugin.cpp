#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <urdf/model.h>
#include <compal_gazebo/SetModelPoseService.h>

namespace gazebo {

const std::string SetModelPoseServiceName = "SetModelPoseService";

// ========================================================================== //

class ServiceManager {
public:
  ServiceManager() : is_called_(false) {}
  virtual void ServiceManagerCallBack() = 0;
  void SetCalledFlag() { is_called_ = true; }
  void ClearCalledFlag() { is_called_ = false; }
  bool IsCalledFlag() { return is_called_; };
private:
  bool is_called_;
};

// ========================================================================== //

template <typename...> class ServicePackage;

template <typename SvrReq, typename SvrRes>
class ServicePackage<SvrReq, SvrRes> : public ServiceManager {
public:
  ServicePackage(std::string service_name) : ServiceName_(service_name) {}
  // virtual ~ServicePackage();
  bool Init(ros::NodeHandle* nh);
  bool ServerCallback(SvrReq &req, SvrRes &res);
  virtual void ServiceManagerCallBack();
  std::string GetServiceName() { return ServiceName_; };
protected:
  SvrReq req_;
  SvrRes res_;
private:
  std::string ServiceName_;
  boost::shared_ptr<ros::NodeHandle> rosnode_;
  ros::ServiceServer srv_server_;
};
template <typename SvrReq, typename SvrRes>
bool ServicePackage<SvrReq, SvrRes>::Init(ros::NodeHandle* nh) {
  rosnode_.reset(nh);
  ROS_INFO_STREAM("ready for adv new service called: " << ServiceName_ );
  srv_server_ = rosnode_->advertiseService(ServiceName_, &ServicePackage::ServerCallback, this);
  return true;
}
template <typename SvrReq, typename SvrRes>
bool ServicePackage<SvrReq, SvrRes>::ServerCallback(SvrReq &req, SvrRes &res) {
  req_ = req;
  res_ = res;
  SetCalledFlag();
  res.result = "data passed";
  return true;
}
template <typename SvrReq, typename SvrRes>
void ServicePackage<SvrReq, SvrRes>::ServiceManagerCallBack() {
  if ( IsCalledFlag() )
    ClearCalledFlag();
}

// ========================================================================== //

typedef compal_gazebo::SetModelPoseService::Request SetModelPoseServiceRequest;
typedef compal_gazebo::SetModelPoseService::Response SetModelPoseServiceResponse;

class SetModelPoseService
  : public ServicePackage<SetModelPoseServiceRequest,
                          SetModelPoseServiceResponse>{
public:
  SetModelPoseService(std::string service_name)
    : ServicePackage(service_name) {}
  void ServiceManagerCallBack() {
    if ( IsCalledFlag() ) {
      ROS_INFO_STREAM("x: " << req_.x << ", " << "y: " << req_.y << ", " << "x: " << req_.z);
      ClearCalledFlag();
    }
  };
};


// ========================================================================== //

class AGV2GazeboPlugin : public ModelPlugin {
  typedef std::vector<ServiceManager*> ServiceManagerVecPtr;
  typedef ServiceManagerVecPtr::iterator ServiceManagerVecIter;
public:
  AGV2GazeboPlugin() : set_model_pose_service_(SetModelPoseServiceName) {}
  ~AGV2GazeboPlugin() {}
  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  virtual void Init();

private:
  void OnUpdate(const common::UpdateInfo& info);

  SetModelPoseService set_model_pose_service_;

  physics::Link_V links_;
  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  common::Time prevUpdateTime;
  std::string robot_namespace_;

  boost::shared_ptr<ros::NodeHandle> rosnode_;
  ros::Time last_update_time_;
  ros::NodeHandle nh_;
  ros::Time last_publish_;
  ServiceManagerVecPtr service_list_;
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
  set_model_pose_service_.Init(rosnode_.get());
  service_list_.push_back( dynamic_cast<ServiceManager*>(&set_model_pose_service_) );
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
    for (ServiceManagerVecIter it = service_list_.begin(); it != service_list_.end(); ++it) {
      (*it)->ServiceManagerCallBack();
    }

  }
}

GZ_REGISTER_MODEL_PLUGIN(AGV2GazeboPlugin)

} // namespace gazebo
