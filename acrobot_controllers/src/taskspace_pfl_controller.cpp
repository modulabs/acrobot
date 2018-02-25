#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>

namespace acrobot_controllers{

class TaskspacePflController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        // get joint name from the parameter server
        if (!n.getParam("joints", joint_names_)) 
        {
            ROS_ERROR_STREAM("Could not find joints name");
            return false;
        }
        n_joints_ = joint_names_.size();

        ROS_INFO("n joints = %d", n_joints_);
        for (unsigned int i=0; i<joint_names_.size(); i++)
        {
            ROS_INFO("joint name = %s", joint_names_[i].c_str());
        }
        tau_.resize(n_joints_);

        if (n_joints_ == 0)
        {
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }

        // Get joint handle and URDF
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }

        for (unsigned int i=0; i<n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));

                ROS_INFO("joint name = %s", joints_[i].getName().c_str());
            }
            catch (const hardware_interface::HardwareInterfaceException& e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);

            return true;
        }

        return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
        tau_[0] = 10.0;
        tau_[1] = 10.0;
        for (unsigned int i=0; i<n_joints_; i++)
        {
            joints_[i].setCommand(tau_[i]);
        }
    }

    void starting(const ros::Time& time) {}
    void stopping(const ros::Time& time) {}

private:
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
    std::vector<double> tau_;
    unsigned int n_joints_;
};
PLUGINLIB_EXPORT_CLASS(acrobot_controllers::TaskspacePflController, controller_interface::ControllerBase);
}

