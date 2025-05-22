#include "policy.h"
#include "logger.h"

GripperPolicy::GripperPolicy() {

}
bool GripperPolicy::execute(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        Activity::Type type) 
{
    return true;
}

void GripperPolicy::update_joint_states(const std::vector<double> &joint_states, int robot_id)
{
    return;
}