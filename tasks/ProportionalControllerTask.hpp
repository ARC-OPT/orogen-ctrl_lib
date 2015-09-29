/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_PROPORTIONALCONTROLLERTASK_TASK_HPP
#define CTRL_LIB_PROPORTIONALCONTROLLERTASK_TASK_HPP

#include "ctrl_lib/ProportionalControllerTaskBase.hpp"
#include <ctrl_lib/ProportionalController.hpp>

namespace ctrl_lib{
    class ProportionalControllerTask : public ProportionalControllerTaskBase
    {
	friend class ProportionalControllerTaskBase;
    protected:

        /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
        virtual bool readSetpoint() = 0;
        /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
        virtual bool readFeedback() = 0;
        /** Write the output of the controller to a port */
        virtual void writeControlOutput(const Eigen::VectorXd &ctrl_output_raw) = 0;

        Eigen::VectorXd control_output_raw; /** Control output */
        std::vector<std::string> field_names;
        ProportionalController* controller;

    public:
        ProportionalControllerTask(std::string const& name = "ctrl_lib::ProportionalControllerTask");
        ProportionalControllerTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~ProportionalControllerTask();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif

