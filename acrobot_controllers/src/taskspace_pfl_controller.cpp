#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include "acrobot.h"

namespace acrobot_controllers{

class TaskspacePflController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    // ~TaskspacePflController() 
    // {
    //     ROS_INFO("Deconstruction");
    //     if (acrobot_)
    //         delete acrobot_;
    // }
        
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        // get joint name from the parameter server
        if (!n.getParam("joints", joint_names_)) 
        {
            ROS_ERROR_STREAM("Could not find joints name");
            return false;
        }
        n_joints_ = joint_names_.size();
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
        }

        // acrobot initialization, [to do] replace acrobot class with kdl
        //// target
        Vector2f q_d(0, 0);
        Vector2f qdot_d(0, 0);
        Vector2f qddot_d(0, 0);

        //// Robot
        Link link1(0.1710, 0.377790, 0.27948, 0.0027273, 0.01);
        Link link2(0.289, 0.388200, 0.32843, 0.003348, 0.01);

        acrobot_ = new Acrobot(link1, link2);
        acrobot_->setTargetValue(q_d, qdot_d, qddot_d);
        acrobot_->setParameter4SwingUp(2, 100, 50);

        return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
        // get joint state
        Vector2f q(joints_[0].getPosition(), joints_[1].getPosition());
        Vector2f qdot(joints_[0].getVelocity(), joints_[1].getVelocity());

        // torque
        tau_[0] = 0.0;
        tau_[1] = acrobot_->calcControlInput(q, qdot);

        // set command
        for (unsigned int i=0; i<n_joints_; i++)
        {
            joints_[i].setCommand(tau_[i]);
        }
    }

    void starting(const ros::Time& time) { }

    void stopping(const ros::Time& time) { }

private:
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
    std::vector<double> tau_;
    unsigned int n_joints_;

    Acrobot* acrobot_;
};
PLUGINLIB_EXPORT_CLASS(acrobot_controllers::TaskspacePflController, controller_interface::ControllerBase);
}

