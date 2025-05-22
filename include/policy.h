#ifndef APEX_MR_POLICY_H
#define APEX_MR_POLICY_H

#include "task.h"
#include "tpg.h"
#include "ros/ros.h"


class Policy {
public:
    Policy() = default;
    virtual bool execute(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        Activity::Type type) = 0;
    virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id) = 0;
};


class GripperPolicy: public Policy {
public:
    GripperPolicy();
    virtual bool execute(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        Activity::Type type) override;
    virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id) override;

};

#endif // APEX_MR_POLICY_H