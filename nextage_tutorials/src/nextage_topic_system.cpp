// Bounded, non-real-time ROS1/hrpsys topic transport for one NEXTAGE arm.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace nextage_tutorials {
class TopicSystem : public hardware_interface::SystemInterface {
  using Return = hardware_interface::return_type;
  using Callback = hardware_interface::CallbackReturn;
  using Clock = std::chrono::steady_clock;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  std::array<double, 6> position_{}, command_{}, ft_{}, anchor_{}, last_sent_{};
  std::array<double, 6> low_{}, high_{};
  std::map<std::string, double> all_;
  Clock::time_point joint_time_{}, ft_time_{}, sent_time_{};
  std::vector<std::string> names_, full_names_;
  std::string sensor_, frame_, topic_;
  bool active_ = false, claimed_ = false, fault_ = false;
  double chest_ = 0;
  static double elapsed(Clock::time_point t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
  bool fresh() const { return elapsed(joint_time_) < 0.25 && elapsed(ft_time_) < 0.25; }
  Return fail(const std::string & reason) {
    if (!fault_) RCLCPP_ERROR(node_->get_logger(), "Transport stopped: %s. Reactivation required.", reason.c_str());
    fault_ = true; claimed_ = false; return Return::ERROR;
  }
public:
  Callback on_init(const hardware_interface::HardwareInfo & info) override {
    if (SystemInterface::on_init(info) != Callback::SUCCESS || info.joints.size()!=6 || info.sensors.size()!=1)
      return Callback::ERROR;
    auto & p = info.hardware_parameters;
    frame_ = p.at("sensor_frame"); topic_ = p.at("command_topic"); sensor_ = info.sensors[0].name;
    full_names_ = {"CHEST_JOINT0", "HEAD_JOINT0", "HEAD_JOINT1"};
    for (const auto & arm : {"LARM", "RARM"}) for (int i=0; i<6; ++i)
      full_names_.push_back(std::string(arm)+"_JOINT"+std::to_string(i));
    for (size_t i=0;i<6;++i) {
      names_.push_back(info.joints[i].name);
      low_[i]=std::stod(info.joints[i].parameters.at("lower"));
      high_[i]=std::stod(info.joints[i].parameters.at("upper"));
      position_[i]=command_[i]=std::numeric_limits<double>::quiet_NaN();
    }
    node_=rclcpp::Node::make_shared(info.name+"_transport", rclcpp::NodeOptions().use_global_arguments(false));
    pub_=node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(topic_, rclcpp::QoS(1));
    joints_sub_=node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        double age=(node_->now()-rclcpp::Time(m->header.stamp)).seconds();
        if (age < -0.1 || age > 0.25 || m->name.size()!=m->position.size()) return;
        std::map<std::string,double> values;
        for(size_t i=0;i<m->name.size();++i) if(std::isfinite(m->position[i])) values[m->name[i]]=m->position[i];
        for(const auto & name:full_names_) if(!values.count(name)) return;
        all_=values; for(size_t i=0;i<6;++i) position_[i]=values.at(names_[i]);
        joint_time_=Clock::now();
      });
    ft_sub_=node_->create_subscription<geometry_msgs::msg::WrenchStamped>(p.at("wrench_topic"),rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr m) {
        double age=(node_->now()-rclcpp::Time(m->header.stamp)).seconds();
        if(age < -0.1 || age > 0.25 || m->header.frame_id!=frame_) return;
        const auto & w=m->wrench;
        std::array<double,6> values={w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z};
        for(double v:values) if(!std::isfinite(v)) return;
        ft_=values; ft_time_=Clock::now();
      });
    return Callback::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for(size_t i=0;i<6;++i) result.emplace_back(names_[i],"position",&position_[i]);
    const std::array<std::string,6> axes={"force.x","force.y","force.z","torque.x","torque.y","torque.z"};
    for(size_t i=0;i<6;++i) result.emplace_back(sensor_,axes[i],&ft_[i]);
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for(size_t i=0;i<6;++i) result.emplace_back(names_[i],"position",&command_[i]);
    return result;
  }
  Callback on_activate(const rclcpp_lifecycle::State &) override {
    auto start=Clock::now();
    while(rclcpp::ok() && elapsed(start)<5.0) {
      rclcpp::spin_some(node_);
      if(fresh() && pub_->get_subscription_count()>0) break;
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    if(!fresh() || pub_->get_subscription_count()==0) {
      RCLCPP_ERROR(node_->get_logger(), "Need fresh fullbody states, compensated FT and command bridge");
      return Callback::FAILURE;
    }
    command_=anchor_=last_sent_=position_; chest_=all_.at("CHEST_JOINT0");
    active_=true; claimed_=false; fault_=false; sent_time_=Clock::now();
    return Callback::SUCCESS;
  }
  Callback on_deactivate(const rclcpp_lifecycle::State &) override { active_=claimed_=false; return Callback::SUCCESS; }
  Return perform_command_mode_switch(const std::vector<std::string>& start, const std::vector<std::string>& stop) override {
    for(const auto & name:names_) for(const auto & s:stop) if(s==name+"/position") claimed_=false;
    bool starting=false;
    for(const auto & name:names_) for(const auto & s:start) if(s==name+"/position") starting=true;
    if(starting) {
      if(!fresh() || fault_) return Return::ERROR;
      command_=anchor_=last_sent_=position_; chest_=all_.at("CHEST_JOINT0");
      claimed_=true; sent_time_=Clock::now();
    }
    return Return::OK;
  }
  Return read(const rclcpp::Time &, const rclcpp::Duration &) override {
    rclcpp::spin_some(node_);
    if(active_ && claimed_ && !fresh()) return fail("stale joint/FT data");
    return Return::OK;
  }
  Return write(const rclcpp::Time &, const rclcpp::Duration &) override {
    if(!active_ || !claimed_) return Return::OK;
    if(fault_ || !fresh()) return fail("stale data or latched fault");
    if(pub_->get_subscription_count()==0) return fail("command bridge disconnected");
    if(node_->count_publishers(topic_)>1) return fail("another fullbody command publisher is running");
    if(std::abs(all_.at("CHEST_JOINT0")-chest_)>0.005) return fail("waist moved");
    for(size_t i=0;i<6;++i) {
      if(std::abs(ft_[i])>(i<3?15.0:1.5)) return fail("external wrench exceeds commissioning limit");
      if(!std::isfinite(command_[i]) || command_[i]<low_[i] || command_[i]>high_[i] ||
         std::abs(command_[i]-anchor_[i])>0.10 || std::abs(command_[i]-position_[i])>0.05)
        return fail("joint command exceeds commissioning envelope");
    }
    double dt=elapsed(sent_time_);
    if(dt<0.04) return Return::OK;
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names=full_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for(const auto & name:full_names_) point.positions.push_back(all_.at(name));
    for(size_t i=0;i<6;++i) {
      // Both command slew and measured-to-target lead are bounded.
      double step=0.10*std::min(dt,0.08);
      double target=std::clamp(command_[i],last_sent_[i]-step,last_sent_[i]+step);
      target=std::clamp(target,position_[i]-0.01,position_[i]+0.01);
      point.positions[std::find(full_names_.begin(),full_names_.end(),names_[i])-full_names_.begin()]=target;
      last_sent_[i]=target;
    }
    point.time_from_start.nanosec=100000000;
    msg.points.push_back(point); pub_->publish(msg); sent_time_=Clock::now();
    return Return::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(nextage_tutorials::TopicSystem, hardware_interface::SystemInterface)
