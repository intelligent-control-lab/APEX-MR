#ifndef APEX_MR_EXECUTION_H
#define APEX_MR_EXECUTION_H

#include <planner.h>
#include <task.h>
#include <queue>
#include <stack>
#include <map>

typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> TrajectoryClient;

namespace boost {
namespace serialization {

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(
    Archive & ar, 
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t, 
    const unsigned int file_version
){
    int rows = t.rows(), cols = t.cols();
    ar & rows;
    ar & cols;
    if(rows * cols != t.size())
        t.resize(rows, cols);

    for(int i = 0; i < t.size(); i++)
        ar & t.data()[i];
}

} // namespace serialization
} // namespace boost

namespace tpg {

    struct TPGConfig {
        bool shortcut = true;
        bool random_shortcut = true;
        bool biased_sample = false;
        bool forward_doubleloop = false;
        bool backward_doubleloop = false;
        bool forward_singleloop = true;
        bool helpful_shortcut = false;
        bool tight_shortcut = true;
        bool tight_shortcut_makespan = true;
        bool subset_shortcut = false;
        bool comp_shortcut = false;
        bool print_contact = false;
        bool run_policy = false;
        bool one_robust = true;
        double subset_prob = 0.4;
        double subset_prob_anneal = 1.0;
        double shortcut_time = 1;
        double dt = 0.1;
        bool switch_shortcut = false;
        bool allow_col = false;
        int ignore_steps = 5;
        std::string progress_file;
        double log_interval = 1.0;
        bool debug_graph = false;
        int seed = 1;
        bool parallel = false;
        bool sync_task = false;

        // execute
        double joint_state_thresh = 0.1;
    };
    
    struct type2Edge;
    struct Node;

    typedef std::shared_ptr<Node> NodePtr;
    typedef std::shared_ptr<type2Edge> type2EdgePtr;

    struct Node
    {
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & pose;
            ar & Type1Next;
            ar & Type2Next;
            ar & Type1Prev;
            ar & Type2Prev;
            ar & timeStep;
            ar & robotId;
            ar & nodeId;
            ar & actId;
        }

        RobotPose pose; // < The pose of the robot at this Node
        NodePtr Type1Next;                    ///< Pointer to the next Node of type 1
        std::vector<type2EdgePtr> Type2Next; ///< Vector of pointers to the next Nodes of type 2
        NodePtr Type1Prev;                    ///< Pointer to the previous Node of type 1
        std::vector<type2EdgePtr> Type2Prev; ///< Vector of pointers to the previous Nodes of type 2
        int timeStep = -1;                       ///< The time step at which this Node exists
        int robotId = -1;                        ///< The ID of the robot at this Node
        int nodeId = -1;                         ///< The ID of the Node
        int actId = -1;                          ///< The ID of the Activity, if any
        
        Node() = default;
        Node(int robot_id, int t)
        {
            this->timeStep = t;
            this->robotId = robot_id;
        };

    };

    struct type2Edge
    {
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & edgeId;
            ar & switchable;
            ar & nodeFrom;
            ar & nodeTo;
            ar & tight;
        }

        int edgeId = -1; ///< The ID of the edge
        bool switchable = true;                 ///< Whether this Node is switchable
        bool tight = false;

        NodePtr nodeFrom; ///< Pointer to the Node from which this edge originates
        NodePtr nodeTo;   ///< Pointer to the Node to which this edge leads
    };

    enum class CollisionType {
        UNKNOWN = -1,
        NONE = 0,
        STATIC = 1,
        ROBOT = 2,
        NO_NEED = 3,
        UNTIGHT = 4,
    };
    
    struct Shortcut {
        std::weak_ptr<Node> ni;
        std::weak_ptr<Node> nj;
        std::vector<RobotPose> path;
        std::weak_ptr<Node> n_robot_col;
        std::shared_ptr<Activity> activity;
        std::vector<int> subset_indices;
        CollisionType col_type = CollisionType::UNKNOWN;

        std::map<int, Shortcut> comp_shortcuts; // for composite shortcuts
        int orig_span = 0;

        Shortcut() = default;
        Shortcut(NodePtr ni, NodePtr nj) {
            this->ni = ni;
            this->nj = nj;
        }

        bool expired() const {
            return ni.expired() || nj.expired();
        };

        bool composite() const {
            return comp_shortcuts.size() > 0;
        };

        bool partial() const {
            return subset_indices.size() > 0;
        };
        
        int robot_id() const {
            return (!expired()) ? ni.lock()->robotId : -1;
        };

    };

    class ShortcutSampler {
    public:
        ShortcutSampler(const TPGConfig &config);
        virtual void init(const std::vector<NodePtr> &start_nodes, const std::vector<int> &numNodes,
                        const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<NodePtr>> &timed_nodes);

        virtual bool sample(Shortcut &shortcut, double time_progress);
        virtual void updateFailedShortcut(const Shortcut &shortcut);
        void update_config(const TPGConfig &config);
    
    protected:
        virtual bool sampleUniform(Shortcut &shortcut);
        virtual bool sampleBiased(Shortcut &shortcut);
        virtual bool sampleComposite(Shortcut &shortcut);
        virtual bool samplePartial(Shortcut &shortcut, double time_progress);
        virtual void resetFailedShortcuts();

        bool biased_ = false;
        bool partial_ = false;
        bool comp_ = false;
        double subset_prob_ = 0.4;
        int num_robots_ = 0;
        std::vector<std::vector<NodePtr>> nodes_;
        std::vector<int> numNodes_;
        std::vector<Shortcut> failed_shortcuts_;
        std::vector<Shortcut> failed_shortcuts_static_;
        std::vector<Shortcut> already_shortcuts_;
        std::vector<Shortcut> untight_shortcuts_;
        std::vector<std::vector<double>> sample_prob_;

        std::vector<std::vector<NodePtr>> timed_nodes_; // timed_nodes_[i][t] = node of robot i at time t
        double scale_ = 15.0;

    };

    class ShortcutIterator {
    public:
        ShortcutIterator(const TPGConfig &config);
        virtual void init(const std::vector<NodePtr> &start_nodes, 
                            const std::vector<int> &numNodes,
                            const std::vector<NodePtr> &end_nodes);

        virtual bool step_begin(Shortcut &shortcut);
        virtual void step_end(const Shortcut &shortcut);
    
    protected:
        std::queue<NodePtr> q_i;
        std::queue<NodePtr> q_j;
        std::vector<NodePtr> start_nodes_;
        std::vector<NodePtr> end_nodes_;
        std::vector<int> numNodes_;

        int num_robots_ = 0;
        bool backward_doubleloop = false;
        bool forward_doubleloop = false;
        bool forward_singleloop = true;
    };

    class TPG {
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & dt_;
        ar & num_robots_;
        ar & type2Edges_;
        ar & start_nodes_;
        ar & end_nodes_;
        ar & numNodes_;
        ar & solution_;
        ar & pre_shortcut_flowtime_;
        ar & pre_shortcut_makespan_;
        ar & post_shortcut_flowtime_;
        ar & post_shortcut_makespan_;
        ar & t_shortcut_;
        ar & t_init_;
        ar & t_simplify_;
        ar & t_shortcut_check_;
        ar & num_shortcut_checks_;
        ar & num_valid_shortcuts_;
        ar & flowtime_improv_;
        ar & collisionCheckMatrix_;
    }
    public:
        TPG() = default;
        // copy constructor
        TPG(const TPG &tpg);
        virtual void reset();
        virtual bool init(std::shared_ptr<PlanInstance> instance, const MRTrajectory &solution, const TPGConfig &config);
        virtual bool optimize(std::shared_ptr<PlanInstance> instance, const TPGConfig &config);
        virtual bool saveToDotFile(const std::string &filename) const;
        virtual bool moveit_execute(std::shared_ptr<MoveitInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);
        virtual bool actionlib_execute(const std::vector<std::string> &joint_names, TrajectoryClient &client);
        virtual bool moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients);
        virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id);
        virtual std::string get_r1_inhand_obj_name();
        virtual std::string get_r2_inhand_obj_name();
        virtual Eigen::MatrixXd get_r1_inhand_goal_q();
        virtual Eigen::MatrixXd get_r2_inhand_goal_q();
        virtual void saveStats(const std::string &filename, const std::string &start_pose = "", const std::string &goal_pose = "") const;
        virtual void setSyncJointTrajectory(trajectory_msgs::JointTrajectory &joint_traj, double &flowtime, double &makespan) const;
        virtual MRTrajectory getSyncJointTrajectory(std::shared_ptr<PlanInstance> instance) const;
        void getSolution(MRTrajectory &solution) const {
            solution = solution_;
        }

        NodePtr getStartNode(int robot_id) const {
            return (robot_id < start_nodes_.size()) ? start_nodes_[robot_id] : nullptr;
        }

        NodePtr getEndNode(int robot_id) const {
            return (robot_id < end_nodes_.size()) ? end_nodes_[robot_id] : nullptr;
        }

        int getNumNodes(int robot_id) const {
            return (robot_id < numNodes_.size()) ? numNodes_[robot_id] : 0;
        }

        void getSolutionTraj(int robot_id, RobotTrajectory &traj) const {
            if (robot_id < solution_.size()) {
                traj = solution_[robot_id];
            }
        }

        double getShortcutTime() const {
            return t_shortcut_;
        }

        TPGConfig getConfig() const {
            return config_;
        }

        virtual int getExecutedAct(int robot_id) const {
            return 0;
        }

        virtual std::shared_ptr<ActivityGraph> getActGraph() const {return nullptr;}

    protected:
        int getTotalNodes() const;
        int getTotalType2Edges() const;
        void getCollisionCheckMatrix(int robot_i, int robot_j, Eigen::MatrixXi &col_matrix) const;
        void updateCollisionCheckMatrix(int robot_i, int robot_j, const Eigen::MatrixXi &col_matrix);
        void transitiveReduction();
        virtual void initSampler(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<NodePtr>> &timed_nodes);
        virtual void initIterator();
        bool findCollisionDeps(std::shared_ptr<PlanInstance> instance, const MRTrajectory &solution,
            const TPGConfig &config);
        bool findCollisionDepsParallel(std::shared_ptr<PlanInstance> instance, const MRTrajectory &solution,
            const TPGConfig &config);
        virtual void findShortcuts(std::shared_ptr<PlanInstance> instance, double runtime_limit);
        virtual void findShortcutsRandom(std::shared_ptr<PlanInstance> instance, double runtime_limit);
        void findEarliestReachTime(std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end) const;
        void updateEarliestReachTime(std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end, Shortcut &shortcut);
        void findLatestReachTime(std::vector<std::vector<int>> &reached_t, const std::vector<int> &reached_end);
        void findTimedNodes(const std::vector<std::vector<int>> &earliest_t, std::vector<std::vector<NodePtr>> & timed_nodes);
        void findTightType2Edges(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<int>> &latest_t);
        void findFlowtimeMakespan(double &flowtime, double &makespan);
        void logProgressStep(const std::vector<int>& reached_end);
        void computePathLength(std::shared_ptr<PlanInstance> instance);
        void preCheckShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
            const std::vector<int> &earliest_t, const std::vector<int> &latest_t) const;
        int computeShortcutSteps(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut) const;
        void retimeShortcut(std::shared_ptr<PlanInstance> instance, int shortcutSteps, Shortcut &shortcut) const;
        virtual void checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut, const std::vector<std::vector<NodePtr>> &timedNodes) const;
        void switchShortcuts();
        void updateTPG(std::shared_ptr<PlanInstance> instance, const Shortcut &shortcut, const std::vector<Eigen::MatrixXi> &col_matrix);
        bool dfs(NodePtr ni, NodePtr nj, std::vector<std::vector<bool>> &visited) const;
        bool bfs(NodePtr ni, std::vector<std::vector<bool>> &visited, bool forward, bool bwd_shiftone=true) const;
        bool hasCycle() const;
        bool repairTPG(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut, const std::vector<std::vector<int>> &earliest_t);
        virtual void moveit_async_execute_thread(const std::vector<std::string> &joint_names, ros::ServiceClient &clients, int robot_id);
        virtual bool isPolicyNode(NodePtr node) const;
        virtual NodePtr getExecStartNode(int robot_id) const;
        virtual bool executePolicy(const NodePtr &startNode, NodePtr &endNode) {throw std::runtime_error("Not implemented");};

        TPGConfig config_;
        double dt_ = 0.1;
        std::vector<std::vector<Eigen::MatrixXi>>  collisionCheckMatrix_; // 1 if no collision, 0 if collision
        int num_robots_;
        int idType2Edges_ = 0;
        std::vector<type2Edge> type2Edges_;
        std::vector<NodePtr> start_nodes_;
        std::vector<NodePtr> end_nodes_;
        std::vector<int> numNodes_;
        MRTrajectory solution_;

        std::vector<std::vector<double>> joint_states_;
        std::vector<std::unique_ptr<std::atomic_int>> executed_steps_;
        std::unique_ptr<ShortcutSampler> shortcut_sampler_;
        std::unique_ptr<ShortcutIterator> shortcut_iterator_;

        // stats
        double pre_shortcut_flowtime_ = 0.0;
        double pre_shortcut_makespan_ = 0.0;
        double post_shortcut_flowtime_ = 0.0;
        double post_shortcut_makespan_ = 0.0;
        double t_shortcut_ = 0.0;
        double t_init_ = 0.0;
        double t_simplify_ = 0.0;
        double t_shortcut_check_ = 0.0;
        int num_shortcut_checks_ = 0;
        int num_valid_shortcuts_ = 0;
        int numtype2edges_pre_ = 0;
        int num_colcheck_pre_ = 0;
        int numtype2edges_post_ = 0;
        int num_colcheck_post_ = 0;
        double flowtime_improv_ = 0.0;
        double path_length_ = 0.0;
        double wait_time_ = 0.0;
        SmoothnessMetrics smoothness_;
        std::string r1_in_hand_obj_name_ = "None";
        std::string r2_in_hand_obj_name_ = "None";
        Eigen::MatrixXd r1_in_hand_goal_q_ = Eigen::MatrixXd::Zero(6, 1);
        Eigen::MatrixXd r2_in_hand_goal_q_ = Eigen::MatrixXd::Zero(6, 1);
    };

}

#endif // APEX_MR_EXECUTION_H