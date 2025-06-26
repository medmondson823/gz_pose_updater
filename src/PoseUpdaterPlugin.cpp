#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <memory>
#include <thread>

using namespace gz;
using namespace sim;

class PoseUpdaterPlugin : public System,
                          public ISystemConfigure,
                          public ISystemPreUpdate
{
public:
  PoseUpdaterPlugin() = default;

  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm,
                 EventManager &) override
  {
    model_ = Model(entity);
    if (!model_.Valid(ecm))
    {
      std::cerr << "[PoseUpdaterPlugin] Invalid model." << std::endl;
      return;
    }

    // Start ROS 2 node WITHOUT crashing
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    ros_node_ = std::make_shared<rclcpp::Node>("pose_updater_node");

    // Get topic from SDF
    std::string topic = "/amcl_pose";
    if (sdf->HasElement("ros_topic"))
      topic = sdf->Get<std::string>("ros_topic");

    subscribed_topic_ = topic;

    // Subscribe
    sub_ = ros_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      topic, 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
      {
        const auto &p = msg->pose.pose.position;
        const auto &o = msg->pose.pose.orientation;

        gz::math::Vector3d position(p.x, p.y, p.z);
        gz::math::Quaterniond orientation(o.w, o.x, o.y, o.z);

        if (this->subscribed_topic_ == "/asv/amcl_pose")
        {
          //Force ASV to float
          position.Z() = 2.1;

          // Force ASV to be upright
          double yaw = orientation.Euler().Z();
          orientation = gz::math::Quaterniond(1.57, 0.0, yaw);
        }

        this->target_pose_ = gz::math::Pose3d(position, orientation);
        this->pose_ready_ = true;
      });


    // Spin the ROS node
    ros_spin_thread_ = std::thread([this]()
                                   { rclcpp::spin(this->ros_node_); });

    std::cout << "[PoseUpdaterPlugin] Listening for pose on: " << topic << std::endl;
  }

  void PreUpdate(const UpdateInfo &, EntityComponentManager &ecm) override
  {
    if (pose_ready_)
    {
      model_.SetWorldPoseCmd(ecm, target_pose_);
      pose_ready_ = false;
    }
  }

  ~PoseUpdaterPlugin() override
  {
    rclcpp::shutdown();
    if (ros_spin_thread_.joinable())
      ros_spin_thread_.join();
  }

private:
  Model model_;
  gz::math::Pose3d target_pose_;
  bool pose_ready_ = false;

  std::string subscribed_topic_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
  std::thread ros_spin_thread_;
};

// Name to be included in world file
GZ_ADD_PLUGIN(PoseUpdaterPlugin,
              sim::System,
              PoseUpdaterPlugin::ISystemConfigure,
              PoseUpdaterPlugin::ISystemPreUpdate)
