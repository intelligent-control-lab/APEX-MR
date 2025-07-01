/**
 * @file adg.h
 * @brief multi-modalTPG construction and shortcutting algorithms for TPG optimization
 * 
 * This file implements the multi-modal TPG framework from the APEX-MR paper 
 *
 * 
 * @author Philip Huang
 * @date 2025
 */

#ifndef APEX_MR_ADG_H
#define APEX_MR_ADG_H

#include "tpg.h"
#include "task.h"
#include "policy.h"

namespace tpg {
    
    struct PolicyNode : public Node {
        PolicyNode() = default;
        PolicyNode(int robot_id, Activity::Type type) : Node(robot_id, type) {
            this->type = type;
        }
    
        Activity::Type type;
        bool execute();
        bool isFinished();
    };

    class ShortcutSamplerADG : public ShortcutSampler {
    public:
        ShortcutSamplerADG(const TPGConfig &config, std::shared_ptr<ActivityGraph> act_graph,       
            const std::vector<std::vector<NodePtr>> &intermediate_nodes);
        void init(const std::vector<NodePtr> &start_nodes,
            const std::vector<int> &numNodes, const std::vector<std::vector<int>> &earliest_t,
            const std::vector<std::vector<NodePtr>> &timed_nodes) override;
    
    private:
        bool sampleUniform(Shortcut &shortcut) override;
        bool sampleComposite(Shortcut &shortcut) override;

        std::vector<std::vector<NodePtr>> nodes_; // ptr to each tpg node
        std::vector<std::vector<int>> act_lengths_; // length of each act for each robot
        std::vector<std::vector<int>> act_ids_; // activity id of each tpg node
        std::shared_ptr<ActivityGraph> act_graph_;
        std::vector<std::vector<NodePtr>> intermediate_nodes_;

        bool skip_home_ = true;
    };

    class ShortcutIteratorADG : public ShortcutIterator {
    public:
        ShortcutIteratorADG(const TPGConfig &config, std::shared_ptr<ActivityGraph> act_graph,
            const std::vector<std::vector<NodePtr>> &intermediate_nodes);
        void init(const std::vector<NodePtr> &start_nodes,
            const std::vector<int> &numNodes,
            const std::vector<NodePtr> &end_nodes) override;

        virtual bool step_begin(Shortcut &shortcut) override;
        virtual void step_end(const Shortcut &shortcut) override;
    private:
        std::vector<std::vector<NodePtr>> seg_nodes_;
        std::queue<std::pair<int, int>> rob_seg_idx;
        std::shared_ptr<ActivityGraph> act_graph_;      
        std::vector<std::vector<int>> act_ids_; // activity id of each tpg node
    
        bool skip_home_ = true;
    };

    class ADG: public TPG {
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<TPG>(*this);
        ar & act_graph_;
        ar & intermediate_nodes_;
        ar & exec_start_act_;
    }
    public:
        ADG() = default;
        ADG (std::shared_ptr<ActivityGraph> activity_graph);

        bool init_from_asynctrajs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const MRTrajectory &trajectories);
        bool init_from_tpgs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs);
        virtual std::string get_r1_inhand_obj_name() override;
        virtual std::string get_r2_inhand_obj_name() override;
        virtual Eigen::MatrixXd get_r1_inhand_goal_q() override;
        virtual Eigen::MatrixXd get_r2_inhand_goal_q() override;
        virtual void reset() override;
        virtual void checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut, const std::vector<std::vector<NodePtr>> &timedNodes) const override;
        virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id) override;
        virtual bool moveit_execute(std::shared_ptr<MoveitInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) override;
        virtual bool moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients) override;
        virtual bool saveToDotFile(const std::string &filename) const override;
        virtual void setPolicy(std::shared_ptr<Policy> policy) {policy_ = policy;}
        virtual void setExecStartAct(int robot_id, int act_id);
        virtual int getExecutedAct(int robot_id) const override;
        virtual std::shared_ptr<ActivityGraph> getActGraph() const override {return act_graph_;}
        virtual bool shiftPolicyNodeType2Edges();

    private:
        bool addTaskDeps();
        bool findCollisionDeps(std::shared_ptr<PlanInstance> instance, const std::vector<std::vector<int>> &reached_t, bool skip_sametask);

        void findCollisionDepsTask(int i, int j, ActPtr act_i, ActPtr act_j, NodePtr act_i_start_node, NodePtr act_j_start_node, NodePtr act_i_end_node,
                                NodePtr act_j_end_node, std::shared_ptr<PlanInstance> instance, const std::vector<std::vector<int>> &reached_t,
                                const std::vector<std::vector<bool>> &visited);
         void findCollisionDepsTaskParallel(int i, int j, ActPtr act_i, ActPtr act_j, NodePtr act_i_start_node, NodePtr act_j_start_node, NodePtr act_i_end_node,
                                NodePtr act_j_end_node, std::shared_ptr<PlanInstance> instance, const std::vector<std::vector<int>> &reached_t,
                                const std::vector<std::vector<bool>> &visited);

        virtual void initSampler(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<NodePtr>> &timed_nodes) override;
        virtual void initIterator() override;
        virtual bool isPolicyNode(NodePtr node) const override;
        virtual bool executePolicy(const NodePtr &startNode, NodePtr &endNode) override;
        virtual NodePtr getExecStartNode(int robot_id) const override;

        void findEarliestReachTimeSyncAct(int num_act, std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end);
        void updateScene(std::shared_ptr<PlanInstance> instance, ActPtr act, int existing_robot=-1) const;

        std::shared_ptr<ActivityGraph> act_graph_;
        std::shared_ptr<MoveitInstance> instance_; // used for moveit execution
        std::vector<std::vector<NodePtr>> intermediate_nodes_;
        std::vector<int> exec_start_act_;
        std::vector<std::unique_ptr<std::atomic_int>> executed_acts_;
        std::shared_ptr<Policy> policy_;
        std::string r1_in_hand_obj_name_ = "None";
        std::string r2_in_hand_obj_name_ = "None";
        Eigen::MatrixXd r1_in_hand_goal_q_ = Eigen::MatrixXd::Zero(6, 1);
        Eigen::MatrixXd r2_in_hand_goal_q_ = Eigen::MatrixXd::Zero(6, 1);
    };
}

#endif //APEX_MR_ADG_H