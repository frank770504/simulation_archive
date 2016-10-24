#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <urdf/model.h>
#include <compal_gazebo/SetModelPoseService.h>
#include <compal_gazebo/PushRodService.h>

namespace gazebo {

const std::string SetModelPoseServiceName = "SetModelPoseService";
const std::string PushRodServiceName = "PushRodService";

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
      ClearCalledFlag();
      ROS_INFO_STREAM("x: " << req_.x << ", " << "y: " << req_.y << ", " << "x: " << req_.z);
      model_->SetLinkWorldPose( math::Pose(req_.x, req_.y, req_.z, 0., 0., 0.), "base_footprint" );
    }
  };
  void SetModelPtr(physics::ModelPtr model) {
    model_ = model;
  }
private:
  physics::ModelPtr model_;
};

// ========================================================================== //

typedef compal_gazebo::PushRodService::Request PushRodServiceRequest;
typedef compal_gazebo::PushRodService::Response PushRodServiceResponse;

class PushRodService
  : public ServicePackage<PushRodServiceRequest,
                          PushRodServiceResponse>{
public:
  PushRodService(std::string service_name)
    : ServicePackage(service_name) {}
  void ServiceManagerCallBack() {
    if ( IsCalledFlag() ) {
      physics::LinkPtr link_rod = model_->GetLink("rod_link");
      ROS_INFO_STREAM("the rod link name is: " << link_rod->GetName());
      ClearCalledFlag();
      if (req_.is_pushing) {
        link_rod->AddForce(math::Vector3(0.0, 0.0, req_.force));
        ROS_INFO_STREAM("flag: " << true << ", force: " << req_.force);
      } else {
        link_rod->AddForce(math::Vector3(0.0, 0.0, req_.force*-1));
        ROS_INFO_STREAM("flag: " << false << ", force: " << req_.force);
      }
    }
  }
  void SetModelPtr(physics::ModelPtr model) {
    model_ = model;
  }
private:
  physics::ModelPtr model_;
};

// ========================================================================== //

class AGV2GazeboPlugin : public ModelPlugin {
  typedef std::vector<ServiceManager*> ServiceManagerVecPtr;
  typedef ServiceManagerVecPtr::iterator ServiceManagerVecIter;
public:
  AGV2GazeboPlugin()
    : set_model_pose_service_(SetModelPoseServiceName),
      push_ros_test_service_(PushRodServiceName) {}
  ~AGV2GazeboPlugin() {}
  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  virtual void Init();

private:
  void OnUpdate(const common::UpdateInfo& info);

  SetModelPoseService set_model_pose_service_;
  PushRodService push_ros_test_service_;

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
  push_ros_test_service_.Init(rosnode_.get());
  set_model_pose_service_.SetModelPtr(model_);
  push_ros_test_service_.SetModelPtr(model_);
  service_list_.push_back( dynamic_cast<ServiceManager*>(&set_model_pose_service_) );
  service_list_.push_back( dynamic_cast<ServiceManager*>(&push_ros_test_service_) );
}

void AGV2GazeboPlugin::Init() {
  // Init time stuff
  prevUpdateTime = model_->GetWorld()->GetSimTime();
  last_publish_ = ros::Time(prevUpdateTime.Double());
  links_ = model_->GetLinks();
  for(physics::Link_V::iterator it = links_.begin(); it != links_.end(); ++it) {
    ROS_INFO_STREAM( "Link Name: " << (*it)->GetName());
  }
}

void AGV2GazeboPlugin::OnUpdate(const common::UpdateInfo& info) {
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
