/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ManipulabilityGradientCtrl.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base/logging.h>

using namespace ctrl_lib;

ManipulabilityGradientCtrl::ManipulabilityGradientCtrl(std::string const& name)
    : ManipulabilityGradientCtrlBase(name)
{
}

ManipulabilityGradientCtrl::ManipulabilityGradientCtrl(std::string const& name, RTT::ExecutionEngine* engine)
    : ManipulabilityGradientCtrlBase(name, engine)
{
}

ManipulabilityGradientCtrl::~ManipulabilityGradientCtrl()
{
}

bool ManipulabilityGradientCtrl::configureHook()
{
    if (! ManipulabilityGradientCtrlBase::configureHook())
        return false;

    delta_q_ = _delta_q.get();
    kp_ = _kp.get();

    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(_urdf.get(), tree))
    {
        LOG_ERROR("Unable to parse urdf mode from file %s", _urdf.get().c_str());
        return false;
    }

    KDL::Chain chain;
    if(!tree.getChain(_controlled_in_frame.get(), _controlled_frame_frame.get(), chain))
    {
        LOG_ERROR("Unable to extract chain between frame %s and frame %s from KDL tree", _controlled_in_frame.get().c_str(), _controlled_frame_frame.get().c_str());
        return false;
    }
    jac_solver_ = new KDL::ChainJntToJacSolver(chain);

    std::vector<std::string> joint_names;
    for(uint i = 0; i < chain.getNrOfSegments(); i++)
    {
        KDL::Segment seg = chain.getSegment(i);
        if(seg.getJoint().getType() != KDL::Joint::None)
            joint_names.push_back(seg.getJoint().getName());
    }

    if(kp_.size() != joint_names.size())
    {
        LOG_ERROR("Number of joints in kinematic chain is %i, but kp vector has size %i", joint_names.size(), kp_.size());
        return false;
    }

    ctrl_out_.resize(joint_names.size());
    ctrl_out_.names = joint_names;
    q_.resize(joint_names.size());
    jac_ = KDL::Jacobian(joint_names.size());

    return true;
}

bool ManipulabilityGradientCtrl::startHook()
{
    if (! ManipulabilityGradientCtrlBase::startHook())
        return false;
    return true;
}

void ManipulabilityGradientCtrl::updateHook()
{
    ManipulabilityGradientCtrlBase::updateHook();

    if(_joint_state.read(joint_state_) != RTT::NoData)
    {
        base::Time start = base::Time::now();

        for(uint i = 0; i < ctrl_out_.size(); i++)
        {
            try{
                q_(i) = joint_state_.getElementByName(ctrl_out_.names[i]).position;
            }
            catch(std::exception e){
                LOG_ERROR("Joint with name %s is in configured kinematic chain, but not in joint state vector", ctrl_out_.names[i].c_str());
                throw e;
            }
        }

        double manip_cur_pos = manipulability(q_);
        for(uint i = 0; i < ctrl_out_.size(); i++)
        {
            q_(i) += delta_q_;
            ctrl_out_[i].speed = ctrl_out_[i].effort = kp_(i) * (manipulability(q_) - manip_cur_pos) / delta_q_;
            q_(i) -= delta_q_;
        }

        _ctrl_out.write(ctrl_out_);
        _sample_time.write((base::Time::now() - start).toSeconds());
        _manipulability.write(manip_cur_pos);
    }
}

void ManipulabilityGradientCtrl::errorHook()
{
    ManipulabilityGradientCtrlBase::errorHook();
}

void ManipulabilityGradientCtrl::stopHook()
{
    ManipulabilityGradientCtrlBase::stopHook();
}

void ManipulabilityGradientCtrl::cleanupHook()
{
    ManipulabilityGradientCtrlBase::cleanupHook();

    ctrl_out_.clear();
    delete jac_solver_;
}

double ManipulabilityGradientCtrl::manipulability(const KDL::JntArray &q)
{
    jac_solver_->JntToJac(q, jac_);
    return sqrt((jac_.data * jac_.data.transpose()).determinant());
}
