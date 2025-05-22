#ifndef APEX_MR_SHORTCUTTER_H
#define APEX_MR_SHORTCUTTER_H

#include "task.h"
#include "logger.h"
#include "instance.h"
#include <random>

struct ShortcutOptions {
    double t_limit = 1;
    double log_interval = 0.1;
    double dt = 0.1;
    int seed = 1;
    std::string progress_file = "";
    bool comp_shortcut = false;
    bool prioritized_shortcut = false;
    bool path_shortcut = false;
    bool tight_shortcut = true;
    bool auto_selector = false;
    bool round_robin = false;
    bool thompson_selector = false;
    bool forward_doubleloop = false;
    bool backward_doubleloop = false;
    bool forward_singleloop = false;
    double gamma_minus = 0.1; // gamma of the selector weight update
    double gamma_plus = 100.0;
    double tau = 0.01; // tau of time spent
    double time_award_factor = 1.0;
    std::vector<std::string> algos{"comp", "prioritized", "path"};
    std::vector<double> weights{0.8, 0.1, 0.1};

    std::vector<double> alphas{10, 1, 1}; // alpha of the beta distribution
    std::vector<double> betas{1, 1, 1};
    int thompson_c = 1000;
    double thompson_alpha_factor = 100.0;
    double thompson_beta_factor = 0.1;
    bool print_contact = false;
};

bool shortcutSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    moveit_msgs::RobotTrajectory &smoothed_traj,
                    const ShortcutOptions &options);

void calculate_path_length_and_wait_time(std::shared_ptr<PlanInstance> instance, const MRTrajectory &synced_plan, 
        double &path_length, double &wait_time);


enum class CollisionType {
    UNKNOWN = -1,
    NONE = 0,
    STATIC = 1,
    ROBOT = 2,
    NO_NEED = 3,
    UNTIGHT = 4,
};

struct Shortcut {
    int robot_id;
    int a, b;
    std::vector<RobotPose> path;
    std::vector<std::vector<RobotPose>> comp_path; // robot pose at each timestep for each robot
    std::vector<int> subset_indices;
    std::shared_ptr<Activity> act;
    CollisionType col_type = CollisionType::UNKNOWN;

    std::vector<double> new_t_afterb;
    std::map<int, int> timed_index_afterb;

    // type
    bool comp_shortcut = false;
    bool prioritized_shortcut = false;
    bool path_shortcut = false;
};

class ShortcutSampler {
public:
    ShortcutSampler(const ShortcutOptions &options);
    
    virtual bool init(const MRTrajectory &plan);

    virtual void updatePlan(const MRTrajectory &plan);

    virtual bool sampleShortcut(Shortcut &shortcut, double time_progress);
    
    virtual void updateWeights(const Shortcut &shortcut, double diff, double t_used);

    virtual bool step_begin(Shortcut &shortcut);

    virtual void step_end(const Shortcut &shortcut);

    virtual double getAlgoCountPercent(int i) const {
        return n_algo[i] * 1.0 / (std::accumulate(n_algo.begin(), n_algo.end(), 0) + 1e-5);
    };

    virtual double getAlgoValidPercent(int i) const {
        return n_valid[i] * 1.0 / (std::accumulate(n_valid.begin(), n_valid.end(), 0) + 1e-5);
    };

    virtual void printStats() const;

protected:
    virtual bool sampleUniform(Shortcut &shortcut);

    virtual void sampleAlgo(Shortcut &shortcut);

    virtual void setAlgo(Shortcut &shortcut, const std::string &algo);

    double sampleBetaDist(double alpha, double beta);
    
    ShortcutOptions options_;
    std::vector<int> num_points_;

    // auto selector
    std::mt19937 rand_;
    std::discrete_distribution<> dist_;

    // round robin
    int robin_counter_ = 0;

    // iterator
    std::queue<Shortcut> q_;

    // count how many times each algorithm is selected
    std::vector<int> n_algo, n_valid;
};

class ShortcutSamplerMT : public ShortcutSampler {
public:
    ShortcutSamplerMT(const ShortcutOptions &options, std::shared_ptr<ActivityGraph> act_graph);
    
    virtual bool init(const MRTrajectory &sync_plan, const std::vector<int> &num_points, const std::vector<std::vector<int>> &num_task_points);

    bool sampleShortcut(Shortcut &shortcut, double time_progress) override;

protected:
    bool sampleUniform(Shortcut &shortcut) override;

    std::shared_ptr<ActivityGraph> act_graph_;
    int num_robots_;
    std::vector<std::vector<int>> num_task_points_;
    std::vector<std::vector<int>> act_ids_;
};

class Shortcutter {
public: 
    Shortcutter(std::shared_ptr<PlanInstance> instance,
                const ShortcutOptions &options);
    
    bool shortcutSolution(const moveit_msgs::RobotTrajectory &plan_traj,
                          MRTrajectory &smoothed_solution);

    bool shortcutSolution(const MRTrajectory &solution,
                          MRTrajectory &smoothed_solution);

    double calculate_makespan(const MRTrajectory &plan);

    double calculate_flowtime(const MRTrajectory &plan);

protected:
    void preCheckShortcut(Shortcut &shortcut);

    void checkShortcut(Shortcut &shortcut);

    void updatePlan(const Shortcut &shortcut);

    bool logProgress(double elapsed);

    void update_stats(const MRTrajectory &plan);

    int n_check_ = 0;
    int n_valid_ = 0;
    int n_colcheck_ = 0;
    double t_check_ = 0;

    double path_length_;
    double wait_time_;

    std::shared_ptr<PlanInstance> instance_;
    ShortcutOptions options_;
    std::shared_ptr<ShortcutSampler> sampler_;

    double makespan_;
    double flowtime_;
    double pre_makespan_;
    double pre_flowtime_;
    SmoothnessMetrics smoothness_;

    int num_robots_;
    MRTrajectory plan_;

    std::ofstream file_;
};

class ShortcutterMT {
public: 
    ShortcutterMT(std::shared_ptr<PlanInstance> instance,
                std::shared_ptr<ActivityGraph> act_graph,
                const ShortcutOptions &options);
    
    bool shortcutSolution(const MRTrajectory &solution,
                          MRTrajectory &smoothed_solution);
    
    void preCheckShortcut(Shortcut &shortcut);

    void checkShortcut(Shortcut &shortcut);

    void updatePlan(const Shortcut &shortcut);

    bool logProgress(const std::string &filename, double elapsed);

    void update_timed_index();

protected:

    double calculate_makespan();

    void calculate_numpoints();

    void updateScene(int robot_id, int act_id);

    void prempt_home_act();

    bool checkTaskDep();

    int n_check_ = 0;
    int n_valid_ = 0;
    int n_colcheck_ = 0;
    double t_check_ = 0;

    double path_length_;
    double wait_time_;

    std::vector<std::vector<int>> num_task_points_; // number of points for each robot, then each task
    std::vector<int> num_points_; // number of points for each robot

    std::shared_ptr<PlanInstance> instance_;
    std::shared_ptr<ActivityGraph> act_graph_;
    ShortcutOptions options_;
    std::shared_ptr<ShortcutSamplerMT> sampler_;
    
    MRTrajectory synced_plan_;
    std::vector<std::vector<int>> timed_index_; // index of each timestep in the plan
    double makespan_;
    double pre_makespan_;
};

#endif // APEX_MR_SHORTCUTTER_H