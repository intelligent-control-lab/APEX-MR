#include "shortcutter.h"

bool shortcutSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    moveit_msgs::RobotTrajectory &smoothed_traj,
                    const ShortcutOptions &options)
{
    if (!options.comp_shortcut && !options.prioritized_shortcut && !options.path_shortcut) {
        log("No shortcut options is enabled!", LogLevel::ERROR);
        return false;
    }
    /* assume the timestep is already uniformly discretized */
    double t_limit = options.t_limit;
    double log_interval = options.log_interval;
    double log_limit = options.log_interval;
    double elapsed = 0.0;
    double dt = options.dt;
    // reset seed
    srand(options.seed);

    smoothed_traj.joint_trajectory.joint_names = plan_traj.joint_trajectory.joint_names;
    // deepcopy the points
    smoothed_traj.joint_trajectory.points.clear();
    double old_makespan = plan_traj.joint_trajectory.points.back().time_from_start.toSec();
    int count = plan_traj.joint_trajectory.points.size();
    for (size_t i = 0; i < count; i++) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = plan_traj.joint_trajectory.points[i].positions;
        point.velocities.resize(point.positions.size());
        point.accelerations.resize(point.positions.size());
        point.time_from_start = plan_traj.joint_trajectory.points[i].time_from_start;
        smoothed_traj.joint_trajectory.points.push_back(point);
    }
    
    std::ofstream file;
    if (options.progress_file != "") {
        file.open(options.progress_file, std::ios::app);
        if (!file.is_open()) {
            log("Failed to open file " + options.progress_file + " for writing!", LogLevel::ERROR);
            return false;
        }
    }

    int n_check = 0;
    int n_valid = 0;
    int n_colcheck = 0;
    double t_check = 0;

    double pathlen, t_wait;
    if (t_limit > 0) {
        MRTrajectory synced_plan_begin;
        convertSolution(instance, smoothed_traj, synced_plan_begin, false);
        calculate_path_length_and_wait_time(instance, synced_plan_begin, pathlen, t_wait);
        file << ",,," << old_makespan << ",," << old_makespan << "," << "," << "," << elapsed << ","  << "," 
            << t_check << "," << n_check << "," << n_valid << ",,,," << n_colcheck << "," << pathlen << ","
            << t_wait << std::endl;
    }

    double makespan = old_makespan;
    while (elapsed < t_limit) {
        auto tic = std::chrono::high_resolution_clock::now();
        // pick two random time points
        int a = rand() % count, b = rand() % count;
        if (a > b) {
            std::swap(a, b);
        }
        if ((a + 1) <= b) {
            
            // find the time to move along the shortcut
            double t_delta = smoothed_traj.joint_trajectory.points[b].time_from_start.toSec() 
                        - smoothed_traj.joint_trajectory.points[a].time_from_start.toSec();
            
            if (options.comp_shortcut) {
                std::vector<double> time_ab;
                std::vector<RobotPose> poses_a, poses_b;
                int dof_s = 0;
                for (int i = 0; i < instance->getNumberOfRobots(); i++) {
                    RobotPose pose_a = instance->initRobotPose(i);
                    RobotPose pose_b = instance->initRobotPose(i);
                    // copy point a's position from dof_s to dof_s + instance->getRobotDOF(i) to pose_i
                    for (int d = 0; d < instance->getRobotDOF(i); d++) {
                        pose_a.joint_values[d] = smoothed_traj.joint_trajectory.points[a].positions[dof_s + d];
                        pose_b.joint_values[d] = smoothed_traj.joint_trajectory.points[b].positions[dof_s + d];
                    }
                    double t_ab = instance->computeDistance(pose_a, pose_b) / instance->getVMax(i);
                    time_ab.push_back(t_ab);
                    poses_a.push_back(pose_a);
                    poses_b.push_back(pose_b);
                    dof_s += instance->getRobotDOF(i);
                }

                double max_time_ab = *std::max_element(time_ab.begin(), time_ab.end());
                int num_steps = std::ceil(max_time_ab / dt);

                if (num_steps < (b - a)) {
                    // interpolate and check collision
                    bool hasCollision = false;
                    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_ab;
                    n_check++;
                    std::vector<RobotPose> poses_prev = poses_a;
                    for (int c = 1; c < num_steps; c++) {
                        double alpha = c * 1.0 / num_steps;
                        std::vector<RobotPose> poses;
                        for (int i = 0; i < instance->getNumberOfRobots(); i++) {
                            RobotPose pose_i = instance->interpolate(poses_a[i], poses_b[i], alpha);
                            poses.push_back(pose_i);
                        }
                        auto tic_inner = std::chrono::high_resolution_clock::now();
                        for (int i = 0; i < poses.size(); i++) {
                            if (instance->checkCollision({poses[i]}, false)) {
                                hasCollision = true;
                                break;
                            }
                            for (int j = i + 1; j < poses.size(); j++) {
                                n_colcheck += 2;
                                if (instance->checkCollision({poses[i], poses[j]}, true) || instance->checkCollision({poses[i], poses_prev[j]}, true)) {
                                    hasCollision = true;
                                    break;
                                }
                            }
                            if (hasCollision) {
                                break;
                            }
                        }
                        poses_prev = poses;

                        auto toc_inner = std::chrono::high_resolution_clock::now();
                        t_check += std::chrono::duration_cast<std::chrono::nanoseconds>(toc_inner - tic_inner).count() * 1e-9;
                        if (hasCollision) {
                            log("Shortcut has collision", LogLevel::DEBUG);
                            break;
                        }
                        trajectory_msgs::JointTrajectoryPoint point;
                        for (int i = 0; i < instance->getNumberOfRobots(); i++) {
                            point.positions.insert(point.positions.end(), poses[i].joint_values.begin(), poses[i].joint_values.end());
                        }
                        point.velocities.resize(point.positions.size());
                        point.accelerations.resize(point.positions.size());
                        traj_ab.push_back(point);
                    }
                    for (int i = 0; i < poses_prev.size(); i++) {
                        for (int j = i + 1; j < poses_prev.size(); j++) {
                            n_colcheck += 2;
                            if (instance->checkCollision({poses_b[i], poses_prev[j]}, true)) {
                                hasCollision = true;
                                break;
                            }
                        }
                        if (hasCollision) {
                            break;
                        }
                    }
                    // if the shortcut is actually collision free, then update the trajectory
                    if (!hasCollision) {
                        // delete the points between a and b
                        smoothed_traj.joint_trajectory.points.erase(smoothed_traj.joint_trajectory.points.begin() + a + 1, smoothed_traj.joint_trajectory.points.begin() + b);
                        // insert the new points
                        smoothed_traj.joint_trajectory.points.insert(smoothed_traj.joint_trajectory.points.begin() + a + 1, traj_ab.begin(), traj_ab.end());
                        // update the time from start from the point after a
                        double t = smoothed_traj.joint_trajectory.points[a].time_from_start.toSec();
                        for (int i = a + 1; i < smoothed_traj.joint_trajectory.points.size(); i++) {
                            t += dt;
                            smoothed_traj.joint_trajectory.points[i].time_from_start = ros::Duration(t);
                        }
                        makespan = smoothed_traj.joint_trajectory.points.back().time_from_start.toSec();
                        log("Shortcut found, t_new " + std::to_string(max_time_ab) + ", t_old " + std::to_string(t_delta) + ", steps" + std::to_string(num_steps)
                            + ", current makespan, " + std::to_string(makespan)
                            + ", elapsed " + std::to_string(elapsed), LogLevel::DEBUG);
                        count = smoothed_traj.joint_trajectory.points.size();
                        n_valid++;
                    }
                }
            }
            else if (options.prioritized_shortcut || options.path_shortcut) {
                int r_id = rand() % instance->getNumberOfRobots();
                RobotPose pose_a = instance->initRobotPose(r_id);
                RobotPose pose_b = instance->initRobotPose(r_id);
                // copy point a's position from dof_s to dof_s + instance->getRobotDOF(i) to pose_i
                int dof_s = r_id * instance->getRobotDOF(r_id);
                for (int d = 0; d < instance->getRobotDOF(r_id); d++) {
                    pose_a.joint_values[d] = smoothed_traj.joint_trajectory.points[a].positions[dof_s + d];
                    pose_b.joint_values[d] = smoothed_traj.joint_trajectory.points[b].positions[dof_s + d];
                }

                // compute time and distance between a and b
                double dist_ab_new = instance->computeDistance(pose_a, pose_b);
                double dist_ab_old = 0;
                RobotPose pose_i = pose_a;
                RobotPose pose_j = pose_b;
                for (int i = a + 1; i <= b; i++) {
                    for (int d = 0; d < instance->getRobotDOF(r_id); d++) {
                        pose_j.joint_values[d] = smoothed_traj.joint_trajectory.points[i].positions[dof_s + d];
                    }
                    dist_ab_old += instance->computeDistance(pose_i, pose_j);
                    pose_i = pose_j;
                }

                double t_ab = dist_ab_new / instance->getVMax(r_id);
                int num_steps = std::ceil(t_ab / dt);

                bool no_need = true;
                if (options.prioritized_shortcut && (num_steps < (b-a))) {
                    no_need = false;
                }
                if (options.path_shortcut && (dist_ab_new < dist_ab_old)) {
                    no_need = false;
                    num_steps = b - a;
                }

                if (!no_need) {
                    // interpolate and check collision
                    bool hasCollision = false;
                    int diff_steps = b - a - num_steps;
                    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_ab;
                    n_check++;
                    // check collision all the way to the end, or to b if path_shortcut
                    int c_end = smoothed_traj.joint_trajectory.points.size();
                    if (options.path_shortcut) {
                        c_end = b;
                    }
                    for (int c = a + 1; c < c_end; c++) {
                        dof_s = r_id * instance->getRobotDOF(r_id);
                        RobotPose pose_r = instance->initRobotPose(r_id);
                        if (c < a + num_steps) {
                            double alpha = (c - a) * 1.0 / num_steps;
                            pose_r = instance->interpolate(pose_a, pose_b, alpha);
                        }
                        else if (c < smoothed_traj.joint_trajectory.points.size() - diff_steps) {
                            for (int d = 0; d < instance->getRobotDOF(r_id); d++) {
                                pose_r.joint_values[d] = smoothed_traj.joint_trajectory.points[c + diff_steps].positions[dof_s + d];
                            }
                        }
                        else {
                            for (int d = 0; d < instance->getRobotDOF(r_id); d++) {
                                pose_r.joint_values[d] = smoothed_traj.joint_trajectory.points.back().positions[dof_s + d];
                            }
                        }

                        std::vector<RobotPose> poses_c, pose_cminus1;
                        // collect the robot poses 
                        dof_s = 0;
                        for (int i = 0; i < instance->getNumberOfRobots(); i++) {
                            if (i == r_id) {
                                poses_c.push_back(pose_r);
                                pose_cminus1.push_back(pose_r);
                            } else {
                                RobotPose pose_i = instance->initRobotPose(i);
                                RobotPose posei_minus1 = instance->initRobotPose(i);
                                for (int d = 0; d < instance->getRobotDOF(i); d++) {
                                    pose_i.joint_values[d] = smoothed_traj.joint_trajectory.points[c].positions[dof_s + d];
                                    posei_minus1.joint_values[d] = smoothed_traj.joint_trajectory.points[c-1].positions[dof_s + d];
                                }
                                pose_cminus1.push_back(posei_minus1);
                                poses_c.push_back(pose_i);
                            }
                            dof_s += instance->getRobotDOF(i);
                        }
                         
                        auto tic_inner = std::chrono::high_resolution_clock::now();
                        n_colcheck += 1;
                        hasCollision = instance->checkCollision({pose_r}, false);
                        if (!hasCollision) {
                            for (int i = 0; i < poses_c.size(); i++) {
                                if (i == r_id) {
                                    continue;
                                }
                                n_colcheck += 2;
                                if (instance->checkCollision({pose_r, poses_c[i]}, true) || instance->checkCollision({pose_r, pose_cminus1[i]}, true)) {
                                    hasCollision = true;
                                    break;
                                }
                            }
                        }
                        auto toc_inner = std::chrono::high_resolution_clock::now();
                        t_check += std::chrono::duration_cast<std::chrono::nanoseconds>(toc_inner - tic_inner).count() * 1e-9;
                        if (hasCollision) {
                            log("Shortcut has collision", LogLevel::DEBUG);
                            break;
                        }
                        trajectory_msgs::JointTrajectoryPoint point;
                        for (int i = 0; i < instance->getNumberOfRobots(); i++) {
                            point.positions.insert(point.positions.end(), poses_c[i].joint_values.begin(), poses_c[i].joint_values.end());
                        }
                        point.velocities.resize(point.positions.size());
                        point.accelerations.resize(point.positions.size());
                        traj_ab.push_back(point);
                    }
                    // if the shortcut is actually collision free, then update the trajectory
                    if (!hasCollision) {
                        // edit the shortcut points
                        for (int c = a + 1; c < c_end ; c++) {
                            smoothed_traj.joint_trajectory.points[c].positions = traj_ab[c - a - 1].positions;
                        }
                        if (options.prioritized_shortcut) {
                            // remove the points at the end that are staying still
                            for (int c = smoothed_traj.joint_trajectory.points.size() - 1; c > 0; c--) {
                                bool is_still = true;
                                for (int d = 0; d < smoothed_traj.joint_trajectory.points[c].positions.size(); d++) {
                                    if (std::abs(smoothed_traj.joint_trajectory.points[c].positions[d] - smoothed_traj.joint_trajectory.points[c-1].positions[d]) > 1e-5) {
                                        is_still = false;
                                        break;
                                    }
                                }
                                if (is_still) {
                                    smoothed_traj.joint_trajectory.points.pop_back();
                                }
                                else {
                                    break;
                                }
                            }
                            makespan = smoothed_traj.joint_trajectory.points.back().time_from_start.toSec();
                        }
                        else if (options.path_shortcut) {
                            // reset the speed
                            MRTrajectory speedup_traj;
                            convertSolution(instance, smoothed_traj, speedup_traj, true);
                            makespan = 0;
                            for (int i = 0; i < speedup_traj.size(); i++) {
                                makespan = std::max(makespan, speedup_traj[i].times.back());
                            }
                        }
                        count = smoothed_traj.joint_trajectory.points.size();
                        n_valid++;
                        log("Shortcut found, robot " + std::to_string(r_id) + " t_new " + std::to_string(t_ab) + ", t_old " + std::to_string(t_delta) 
                            + ", steps" + std::to_string(num_steps) + " a=" + std::to_string(a) +  " b=" + std::to_string(b)
                            + ", current makespan, " + std::to_string(makespan)
                            + ", count " + std::to_string(count), LogLevel::DEBUG);
                    }
                }

            }
        
        } 

        auto toc = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
        if (elapsed > log_limit) {
            log_limit += log_interval;
            if (options.progress_file != "") {
                MRTrajectory synced_plan;
                convertSolution(instance, smoothed_traj, synced_plan, false);
                calculate_path_length_and_wait_time(instance, synced_plan, pathlen, t_wait);
                file << ",,," << old_makespan << ",," << makespan << "," << ",," << elapsed << ","  << "," 
                    << t_check << "," << n_check << "," << n_valid << ",,,," << n_colcheck << "," << pathlen << ","
                    << t_wait << std::endl;
            }
        }
    }
    file.close();
    log("Shortcutting done, new makespan " + std::to_string(makespan)
        + ", old makespan " + std::to_string(plan_traj.joint_trajectory.points.back().time_from_start.toSec())
        + ", elapsed " + std::to_string(elapsed), LogLevel::INFO);
    return true;
}

void calculate_path_length_and_wait_time(std::shared_ptr<PlanInstance> instance, const MRTrajectory &synced_plan, 
        double &path_length, double &wait_time) {
    int num_robots = instance->getNumberOfRobots();

    path_length = 0;
    wait_time = 0;

    for (int i = 0; i < num_robots; i++) {
        for (int j = 0; j < synced_plan[i].trajectory.size() - 1; j++) {
            double d = instance->computeDistance(synced_plan[i].trajectory[j], synced_plan[i].trajectory[j+1]);
            path_length += d;
            if (d < 1e-5) {
                wait_time += (synced_plan[i].times[j+1] - synced_plan[i].times[j]);
            }
        }
    }

}


ShortcutSampler::ShortcutSampler(const ShortcutOptions &options)
    : options_(options), rand_(options.seed), dist_(options.weights.begin(), options.weights.end())
{
    n_algo.resize(options.algos.size(), 0);
    n_valid.resize(options.algos.size(), 0);
}

void ShortcutSampler::updatePlan(const MRTrajectory &plan)
{
    num_points_.resize(plan.size());
    for (int i = 0; i < plan.size(); i++) {
        num_points_[i] = plan[i].trajectory.size();
    }
}

bool ShortcutSampler::init(const MRTrajectory &plan)
{
    updatePlan(plan);
    if (options_.forward_doubleloop || options_.forward_singleloop || options_.backward_doubleloop) {
        for (int i = 0; i < num_points_.size(); i++) {
            Shortcut shortcut;
            shortcut.robot_id = i;
            shortcut.a = 0;
            if (options_.backward_doubleloop) {
                shortcut.b = num_points_[i] - 1;
            }
            else {
                shortcut.b = 2;
            }
            q_.push(shortcut);
        }
    }

    return true;
}

bool ShortcutSampler::sampleShortcut(Shortcut &shortcut, double time_progress)
{
    if (options_.forward_doubleloop || options_.forward_singleloop || options_.backward_doubleloop) {
        return step_begin(shortcut);
    }
    return sampleUniform(shortcut);
}

bool ShortcutSampler::step_begin(Shortcut &shortcut)
{
    Shortcut new_s = q_.front();
    int a = new_s.a;
    int b = new_s.b;
    int r_id = new_s.robot_id;
    q_.pop();

    if (options_.backward_doubleloop) {
        if ((a + 2) >= num_points_[r_id]) {
            // first pointer reached the end, reset from start
            new_s.a = 0;
            new_s.b = num_points_[r_id] - 1;
            q_.push(new_s);
            return false;
        }
        else if ((a + 1) == b) {
            // last pointer reached first pointer, reset inner loop
            new_s.a = a + 1;
            new_s.b = num_points_[r_id] - 1;
            q_.push(new_s);
            return false;
        }
    }
    else {
        if (b >= num_points_[r_id]) {
            if ((a + 2) >= num_points_[r_id]) {
                // first pointer reached the end, reset from start
                new_s.a = 0;
                new_s.b = 2;
                q_.push(new_s);
            }
            else {
                // just last pointer reached the end, reset inner loop
                new_s.a = a + 1;
                new_s.b = a + 3;
                q_.push(new_s);
            }
            
            return false;
        }
        else if ((a + 1) == b) {
            // shouldn't need this, but just in case skip one-step shortcut
            new_s.a = a;
            new_s.b = b + 1;
            q_.push(new_s);
            
            return false;
        }
    }
    
    shortcut.robot_id = r_id;
    shortcut.a = a;
    shortcut.b = b;

    sampleAlgo(shortcut);

    return true;
}

void ShortcutSampler::step_end(const Shortcut &shortcut)
{
    int a = shortcut.a;
    int b = shortcut.b;
    int r_id = shortcut.robot_id;

    Shortcut new_s = shortcut;

    if (options_.backward_doubleloop) {
        if (shortcut.col_type == CollisionType::NONE) {
            new_s.a = a + 1;
            new_s.b = num_points_[r_id] - 1;
        }
        else {
            new_s.a = a;
            new_s.b = b - 1;
        }
        new_s.a = a;
        new_s.b = b - 1;
    }
    else if (options_.forward_doubleloop) {
        new_s.a = a;
        new_s.b = b + 1;
    }
    else if (options_.forward_singleloop) {
        if (shortcut.col_type == CollisionType::NONE) {
            new_s.a = b;
            new_s.b = b + 2;
        }
        else {
            new_s.a = a;
            new_s.b = b + 1;
        }
    }
    else {
        log("Invalid shortcut options for iterator", LogLevel::ERROR);
    }

    q_.push(new_s);
}

double ShortcutSampler::sampleBetaDist(double alpha, double beta)
{
    std::gamma_distribution<double> gamma_alpha(alpha, 1.0);
    std::gamma_distribution<double> gamma_beta(beta, 1.0);
    double x = gamma_alpha(rand_);
    double y = gamma_beta(rand_);
    return x / (x + y);
}

void ShortcutSampler::updateWeights(const Shortcut &shortcut, double diff, double t_used)
{
    int ind = 0;
    if (shortcut.comp_shortcut) {
        ind = 0;
    }
    else if (shortcut.prioritized_shortcut) {
        ind = 1;
    }
    else if (shortcut.path_shortcut) {
        ind = 2;
    }
    n_algo[ind] ++;

    if (options_.auto_selector) {
        // update formulat w = w * (1-gamma-) * (1 + gamma+ * diff)
        double t_adjust = std::min(t_used/options_.tau, 1.0);
        double adjusted_diff = diff + options_.time_award_factor * (1 - t_adjust);
        options_.weights[ind] *= (1 - options_.gamma_minus);
        if (shortcut.col_type == CollisionType::NONE) {
            options_.weights[ind] *= (1 + options_.gamma_plus * adjusted_diff);
            n_valid[ind] ++;
        }
        // renomralize the weights
        double sum = std::accumulate(options_.weights.begin(), options_.weights.end(), 0.0);
        for (int i = 0; i < options_.weights.size(); i++) {
            options_.weights[i] /= sum;
        }

        dist_ = std::discrete_distribution<>(options_.weights.begin(), options_.weights.end());
    }
    else if (options_.thompson_selector) {
        double t_adjust = std::min(t_used/options_.tau, 1.0);
        double adjusted_diff = diff + options_.time_award_factor * (1 - t_adjust);
        
        if (shortcut.col_type == CollisionType::NONE) {
            options_.alphas[ind] += adjusted_diff * options_.thompson_alpha_factor;
            n_valid[ind] ++;
        }
        else {
            // failure
            options_.betas[ind] += options_.thompson_beta_factor;
        }
        if (options_.alphas[ind] + options_.betas[ind] > options_.thompson_c) {
            // renormalize alpha = c/(c+1) * alpha, beta = c/(c+1) * beta
            double c = options_.thompson_c;
            options_.alphas[ind] = c / (options_.alphas[ind] + options_.betas[ind]) * options_.alphas[ind];
            options_.betas[ind] = c / (options_.alphas[ind] + options_.betas[ind]) * options_.betas[ind]; 
        }
    }
}

bool ShortcutSampler::sampleUniform(Shortcut &shortcut)
{
    int r_id = rand() % num_points_.size();
    int a = rand() % num_points_[r_id];
    int b = rand() % num_points_[r_id];
    if (a > b) {
        std::swap(a, b);
    }
    if ((a+1)>=b) {
        return false;
    }
    
    shortcut.robot_id = r_id;
    shortcut.a = a;
    shortcut.b = b;

    sampleAlgo(shortcut);

    return true;
}

void ShortcutSampler::sampleAlgo(Shortcut &shortcut)
{
    if (options_.auto_selector) {
        // randomly sample a float between 0 and 1
        std::string algo = options_.algos[dist_(rand_)];
        setAlgo(shortcut, algo);
    }
    else if (options_.round_robin) {
        robin_counter_ = (robin_counter_ + 1) % options_.algos.size();
        std::string algo = options_.algos[robin_counter_];
        setAlgo(shortcut, algo);
    }
    else if (options_.thompson_selector) {
        std::vector<double> samples;
        for (int i = 0; i < options_.algos.size(); i++) {
            samples.push_back(sampleBetaDist(options_.alphas[i], options_.betas[i]));
        }
        int max_idx = std::distance(samples.begin(), std::max_element(samples.begin(), samples.end()));
        std::string algo = options_.algos[max_idx];
        setAlgo(shortcut, algo);
    }
    else {
        shortcut.comp_shortcut = options_.comp_shortcut;
        shortcut.prioritized_shortcut = options_.prioritized_shortcut;
        shortcut.path_shortcut = options_.path_shortcut;
    }
}

void ShortcutSampler::setAlgo(Shortcut &shortcut, const std::string &algo)
{
    if (algo == "comp") {
        shortcut.comp_shortcut = true;
        shortcut.prioritized_shortcut = false;
        shortcut.path_shortcut = false;
    }
    else if (algo == "prioritized") {
        shortcut.comp_shortcut = false;
        shortcut.prioritized_shortcut = true;
        shortcut.path_shortcut = false;
    }
    else if (algo == "path") {
        shortcut.comp_shortcut = false;
        shortcut.prioritized_shortcut = false;
        shortcut.path_shortcut = true;
    }
}

void ShortcutSampler::printStats() const
{
    if (options_.auto_selector) {
        // print all the weights
        std::string weights = "Weights: ";
        for (int i = 0; i < options_.weights.size(); i++) {
            weights += options_.algos[i] + ": " + std::to_string(options_.weights[i]) + ", ";
        }
        log(weights, LogLevel::INFO);
    }
    else if (options_.thompson_selector) {
        // print all the alphas and betas
        std::string alphas = "Alphas: ";
        std::string betas = "Betas: ";
        for (int i = 0; i < options_.alphas.size(); i++) {
            alphas += options_.algos[i] + ": " + std::to_string(options_.alphas[i]) + ", ";
            betas += options_.algos[i] + ": " + std::to_string(options_.betas[i]) + ", ";
        }
        log(alphas, LogLevel::INFO);
        log(betas, LogLevel::INFO);
    }
}

Shortcutter::Shortcutter(std::shared_ptr<PlanInstance> instance,
                const ShortcutOptions &options)
{
    instance_ = instance;
    options_ = options;
}

bool Shortcutter::shortcutSolution(const moveit_msgs::RobotTrajectory &plan_traj,
                          MRTrajectory &smoothed_solution)
{
    MRTrajectory plan;
    convertSolution(instance_, plan_traj, plan, false);
    
    return shortcutSolution(plan, smoothed_solution);
}

bool Shortcutter::shortcutSolution(const MRTrajectory &solution,
                                MRTrajectory &smoothed_solution)
{
    if (!options_.auto_selector && !options_.comp_shortcut && !options_.prioritized_shortcut && !options_.path_shortcut
        && !options_.thompson_selector && !options_.round_robin) {
        log("No shortcut options is enabled!", LogLevel::ERROR);
        return false;
    }

    /* assume the timestep is already uniformly discretized */
    double t_limit = options_.t_limit;
    double log_interval = options_.log_interval;
    double log_limit = options_.log_interval;
    double elapsed = 0.0;
    double dt = options_.dt;
    num_robots_ = instance_->getNumberOfRobots();
    // reset seed
    srand(options_.seed);

    // deepcopy the points
    plan_.clear();
    plan_ = solution;

    int max_count = 0;
    for (int i = 0; i < solution.size(); i++) {
        max_count = std::max(max_count, (int)plan_[i].trajectory.size());
    }
    
    // append the last point to make the trajectory uniform length across robots
    for (int i = 0; i < plan_.size(); i++) {
        int count = plan_[i].trajectory.size();
        if (count < max_count) {
            RobotPose last_pose = plan_[i].trajectory.back();
            for (int j = count; j < max_count; j++) {
                plan_[i].trajectory.push_back(last_pose);
                plan_[i].times.push_back(plan_[i].times.back() + options_.dt);
            }
        }
    }
    
    update_stats(plan_);
    
    pre_makespan_ = makespan_;
    pre_flowtime_ = flowtime_;
    int count = std::ceil(makespan_ / dt) + 1;

    if (options_.progress_file != "") {
        file_.open(options_.progress_file, std::ios::app);
        if (!file_.is_open()) {
            log("Failed to open file " + options_.progress_file + " for writing!", LogLevel::ERROR);
            return false;
        }
    }

    sampler_ = std::make_shared<ShortcutSampler>(options_);
    sampler_->init(solution);

    if (t_limit > 0) {
        calculate_path_length_and_wait_time(instance_, plan_, path_length_, wait_time_);
        logProgress(elapsed);
    }

    while (elapsed < t_limit) {
        auto tic = std::chrono::high_resolution_clock::now();

        Shortcut shortcut;

        if (!sampler_->sampleShortcut(shortcut, elapsed / t_limit)) {
            auto toc = std::chrono::high_resolution_clock::now();
            elapsed += std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
            continue;
        }
        
        double makespan_prev = makespan_;
        double pathlen_prev = path_length_;

        preCheckShortcut(shortcut);
        if (shortcut.col_type == CollisionType::NONE) {
            auto tic_inner = std::chrono::high_resolution_clock::now();
            checkShortcut(shortcut);
            auto inner = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - tic_inner).count();
            t_check_ += (inner * 1e-9);
            n_check_++;
            n_colcheck_ += instance_->numCollisionChecks();
            if (shortcut.col_type == CollisionType::NONE) {
                updatePlan(shortcut);
                sampler_->updatePlan(plan_);

                n_valid_ ++;
                calculate_path_length_and_wait_time(instance_, plan_, path_length_, wait_time_);
                
                std::string shortuct_type = (shortcut.comp_shortcut) ? "comp" : (shortcut.prioritized_shortcut) ? "prioritized" : "path";
                log("Shortcut " + shortuct_type + " found, robot " + ((shortcut.comp_shortcut) ? "" : std::to_string(shortcut.robot_id))
                    + ", steps" + ((options_.comp_shortcut) ? std::to_string(shortcut.comp_path.size() + 1) : std::to_string(shortcut.path.size() + 1)) 
                    + " a=" + std::to_string(shortcut.a) +  " b=" + std::to_string(shortcut.b)
                    + ", current makespan, " + std::to_string(makespan_), LogLevel::DEBUG);
            }            
        }

        // update auto selector
        if (options_.auto_selector || options_.thompson_selector) {
            auto toc = std::chrono::high_resolution_clock::now();
            double t_used = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
            sampler_->updateWeights(shortcut, (pathlen_prev - path_length_) / pathlen_prev, t_used);
        }

        if (options_.forward_doubleloop || options_.backward_doubleloop || options_.forward_singleloop) {
            sampler_->step_end(shortcut);
        }

        auto toc = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
        while (elapsed > log_limit) {
            log_limit += log_interval;
            if (options_.progress_file != "") {
                update_stats(plan_);
                logProgress(elapsed);
            }
            bool success = validateSolution(instance_, plan_);
            if (!success) {
                log("Invalid solution after shortcutting!", LogLevel::ERROR);
                return false;
            }
        }
    }
    file_.close();
    log("Shortcutting done, new makespan " + std::to_string(makespan_)
        + ", old makespan " + std::to_string(pre_makespan_)
        + ", elapsed " + std::to_string(elapsed), LogLevel::INFO);

    sampler_->printStats();
    
    if (options_.path_shortcut || options_.auto_selector || options_.thompson_selector || options_.round_robin) {
        retimeSolution(instance_, plan_, smoothed_solution, options_.dt);    
    }
    else {
        smoothed_solution = plan_;
    }
    
    return true;
}

void Shortcutter::preCheckShortcut(Shortcut &shortcut) {
    int a = shortcut.a;
    int b = shortcut.b;
    int r_id = shortcut.robot_id;
    if (shortcut.comp_shortcut) {
        std::vector<double> time_ab;
        std::vector<RobotPose> poses_a, poses_b;
        int dof_s = 0;
        for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
            RobotPose pose_a = plan_[i].trajectory[a];
            RobotPose pose_b = plan_[i].trajectory[b];
            
            double t_ab = instance_->computeDistance(plan_[i].trajectory[a], plan_[i].trajectory[b]) / instance_->getVMax(i);
            time_ab.push_back(t_ab);
            poses_a.push_back(pose_a);
            poses_b.push_back(pose_b);
        }

        double max_time_ab = *std::max_element(time_ab.begin(), time_ab.end());
        int num_steps = std::ceil(max_time_ab / options_.dt);

        if (num_steps >= (b - a)) {
            shortcut.col_type = CollisionType::NO_NEED;
            return;
        }

        shortcut.col_type = CollisionType::NONE;
        for (int c = 1; c < num_steps; c++) {
            double alpha = c * 1.0 / num_steps;
            std::vector<RobotPose> poses;

            for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
                RobotPose pose_i = instance_->interpolate(poses_a[i], poses_b[i], alpha);
                poses.push_back(pose_i);
            }
            shortcut.comp_path.push_back(poses);
        }
    }
    else if (shortcut.path_shortcut || shortcut.prioritized_shortcut)
    {
        // compute time and distance between a and b
        RobotPose pose_a = plan_[r_id].trajectory[a];
        RobotPose pose_b = plan_[r_id].trajectory[b];
        double dist_ab_new = instance_->computeDistance(pose_a, pose_b);
        double dist_ab_old = 0;
        
        for (int i = a + 1; i <= b; i++) {
            dist_ab_old += instance_->computeDistance(plan_[r_id].trajectory[i-1], plan_[r_id].trajectory[i]);
        }

        double t_ab = dist_ab_new / instance_->getVMax(r_id);
        int num_steps = std::ceil(t_ab / options_.dt);

        bool no_need = true;
        if (shortcut.prioritized_shortcut) {
            if (num_steps >= (b-a)) {
                shortcut.col_type = CollisionType::NO_NEED;
                return;
            }
            int diff = (b - a) - num_steps;
            double d_b_diff = instance_->computeDistance(plan_[r_id].trajectory[b-diff], plan_[r_id].trajectory[b]);
            if (d_b_diff < 1e-5) {
                shortcut.col_type = CollisionType::NO_NEED;
                return;
            }
            shortcut.col_type = CollisionType::NONE;
            for (int c = 1; c < num_steps; c++) {
                double alpha = c * 1.0 / num_steps;
                RobotPose pose_i = instance_->interpolate(pose_a, pose_b, alpha);
                shortcut.path.push_back(pose_i);
            }
        }
        if (shortcut.path_shortcut) {
            num_steps = b - a;
            if (dist_ab_new > (dist_ab_old - 1e-5)) {
                shortcut.col_type = CollisionType::NO_NEED;
                return;
            }
            shortcut.col_type = CollisionType::NONE;
            for (int c = 1; c < num_steps; c++) {
                double alpha = c * 1.0 / num_steps;
                RobotPose pose_i = instance_->interpolate(pose_a, pose_b, alpha);
                shortcut.path.push_back(pose_i);
            }
        }
    }

}

void Shortcutter::checkShortcut(Shortcut &shortcut) {
    int a = shortcut.a;
    int b = shortcut.b;

    if (shortcut.comp_shortcut) {
        // interpolate and check collision
        std::vector<trajectory_msgs::JointTrajectoryPoint> traj_ab;
        std::vector<RobotPose> poses_prev;
        for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
            poses_prev.push_back(plan_[i].trajectory[shortcut.a]);
        }
        for (int c = 0; c < shortcut.comp_path.size(); c++) {
            std::vector<RobotPose> poses = shortcut.comp_path[c];
             
            auto tic_inner = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < poses.size(); i++) {
                if (instance_->checkCollision({poses[i]}, false)) {
                    shortcut.col_type = CollisionType::STATIC;
                    return;
                }
                for (int j = i + 1; j < poses.size(); j++) {
                    if (instance_->checkCollision({poses[i], poses[j]}, true) || instance_->checkCollision({poses[i], poses_prev[j]}, true)) {
                        shortcut.col_type = CollisionType::ROBOT;
                        return;
                    }
                }
            }
            poses_prev = poses;

        }
        for (int i = 0; i < poses_prev.size(); i++) {
            for (int j = i + 1; j < poses_prev.size(); j++) {
                if (instance_->checkCollision({plan_[i].trajectory[shortcut.b], poses_prev[j]}, true)) {
                    shortcut.col_type = CollisionType::ROBOT;
                    return;
                }
            }
        }
    }
    else if (shortcut.prioritized_shortcut || shortcut.path_shortcut) {
        // interpolate and check collision
        int r_id = shortcut.robot_id;
        int num_steps = shortcut.path.size() + 1;
        int diff_steps = b - a - num_steps;
        
        // check collision all the way to the end, or to b if path_shortcut
        int c_end = 0;
        if (shortcut.path_shortcut) {
            c_end = b;
        }
        else {
            for (int i = 0; i < plan_.size(); i++) {
                c_end = std::max(c_end, (int)plan_[i].trajectory.size());
            }
        }
        for (int c = a + 1; c < c_end; c++) {
            RobotPose pose_r;
            if (c < a + num_steps) {
                pose_r = shortcut.path[c - a - 1];
            }
            else if (c < c_end - diff_steps) {
                pose_r = plan_[r_id].trajectory[c + diff_steps];
            }
            else {
                pose_r = plan_[r_id].trajectory.back();
            }

            std::vector<RobotPose> poses_c, pose_cminus1;
            // collect other robot's poses
            for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
                if (i == r_id) {
                    poses_c.push_back(pose_r);
                    pose_cminus1.push_back(pose_r); // won't be used to check collision, just placeholder
                } else {
                    poses_c.push_back(plan_[i].trajectory[c]);
                    pose_cminus1.push_back(plan_[i].trajectory[c-1]);
                }
            }
            
            // check collision
            if (instance_->checkCollision({pose_r}, false)) {
                shortcut.col_type = CollisionType::STATIC;
                return;
            }
            for (int i = 0; i < poses_c.size(); i++) {
                if (i == r_id) {
                    continue;
                }
                if (instance_->checkCollision({pose_r, poses_c[i]}, true) || instance_->checkCollision({pose_r, pose_cminus1[i]}, true)) {
                    shortcut.col_type = CollisionType::ROBOT;
                    return;
                }
            }
        }

        if (shortcut.path_shortcut) {
            // check collision between the last point and b of the original plan
            for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
                if (shortcut.path.size() > 0) {
                    if (instance_->checkCollision({plan_[i].trajectory[shortcut.b], shortcut.path.back()}, true)) {
                        shortcut.col_type = CollisionType::ROBOT;
                        return;
                    }
                }
            }
        }
    }
}

void Shortcutter::updatePlan(const Shortcut &shortcut) {
    if (shortcut.comp_shortcut) {
        for (int i = 0; i < num_robots_; i++) {
            // delete the points between a and b
            plan_[i].trajectory.erase(plan_[i].trajectory.begin() + shortcut.a + 1, plan_[i].trajectory.begin() + shortcut.b);
            plan_[i].times.erase(plan_[i].times.begin() + shortcut.a + 1, plan_[i].times.begin() + shortcut.b);
            // insert the new points
            for (int c = 0; c < shortcut.comp_path.size(); c++) {
                plan_[i].trajectory.insert(plan_[i].trajectory.begin() + shortcut.a + 1 + c, shortcut.comp_path[c][i]);
                plan_[i].times.insert(plan_[i].times.begin() + shortcut.a + 1 + c, plan_[i].times[shortcut.a] + (1 + c) * options_.dt);
            }
            // update the time from start from the point after a
            for (int j = shortcut.a + 1 + shortcut.comp_path.size(); j < plan_[i].trajectory.size(); j++) {
                plan_[i].times[j] = plan_[i].times[j-1] + options_.dt;
            }

        }
        makespan_ = calculate_makespan(plan_);
    }
    else if (shortcut.prioritized_shortcut) {
        // edit the shortcut points
        int r_id = shortcut.robot_id;
        int count = plan_[r_id].trajectory.size();
        plan_[r_id].trajectory.erase(plan_[r_id].trajectory.begin() + shortcut.a + 1, plan_[r_id].trajectory.begin() + shortcut.b);
        plan_[r_id].times.erase(plan_[r_id].times.begin() + shortcut.a + 1, plan_[r_id].times.begin() + shortcut.b);
        // insert the new points
        for (int c = 0; c < shortcut.path.size(); c++) {
            plan_[r_id].trajectory.insert(plan_[r_id].trajectory.begin() + shortcut.a + 1 + c, shortcut.path[c]);
            plan_[r_id].times.insert(plan_[r_id].times.begin() + shortcut.a + 1 + c, plan_[r_id].times[shortcut.a] + (1 + c) * options_.dt);
        }
        
        // update the time from start from the point after a
        for (int j = shortcut.a + 1 + shortcut.path.size(); j < plan_[r_id].trajectory.size(); j++) {
            plan_[r_id].times[j] = plan_[r_id].times[j-1] + options_.dt;
        }

        // add the last point (waiting at the end)
        int new_count = plan_[r_id].trajectory.size();
        for (int c = new_count; c < count; c++) {
            plan_[r_id].trajectory.push_back(plan_[r_id].trajectory.back());
            plan_[r_id].times.push_back(plan_[r_id].times.back() + options_.dt);
        }

        // remove the points at the end that are all staying still       
        int j = plan_[r_id].trajectory.size() - 1;
        while (j > 1)  {
            bool is_still = true;
            for (int i = 0; i < num_robots_; i++) {
                if (instance_->computeDistance(plan_[i].trajectory[j], plan_[i].trajectory[j-1]) > 1e-5) {
                    is_still = false;
                    break;
                }
            }
            if (is_still) {
                for (int i = 0; i < num_robots_; i++) {
                    plan_[i].trajectory.pop_back();
                    plan_[i].times.pop_back();
                }
                j--;
            }
            else {
                break;
            }
        }
        makespan_ = calculate_makespan(plan_);
    }
    else if (shortcut.path_shortcut) {
        int r_id = shortcut.robot_id;
        for (int c = 0; c < shortcut.path.size(); c++) {
            plan_[r_id].trajectory[shortcut.a + 1 + c] = shortcut.path[c];
        }

        MRTrajectory speedup_traj;
        retimeSolution(instance_, plan_, speedup_traj, options_.dt);
        makespan_ = calculate_makespan(speedup_traj);
    }
}

double Shortcutter::calculate_makespan(const MRTrajectory &plan) {
    double makespan = 0;
    for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
        makespan = std::max(makespan, plan[i].times.back());
    }
    return makespan;
}

double Shortcutter::calculate_flowtime(const MRTrajectory &plan) {
    double flowtime = 0;
    for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
        for (int j = plan[i].trajectory.size() - 1; j >= 1; j--) {
            if (instance_->computeDistance(plan[i].trajectory[j], plan[i].trajectory[j-1]) > 1e-5) {
                flowtime += plan[i].times[j];
                break;
            }
        }
    }
    return flowtime;
}

void Shortcutter::update_stats(const MRTrajectory &plan) {
    MRTrajectory retime_plan = plan;
    if (options_.path_shortcut || options_.auto_selector || options_.thompson_selector || options_.round_robin) {
        retimeSolution(instance_, plan, retime_plan, options_.dt);
    }
    makespan_ = calculate_makespan(retime_plan);
    flowtime_ = calculate_flowtime(retime_plan);
    smoothness_ = calculate_smoothness(retime_plan, instance_);
}

bool Shortcutter::logProgress(double elapsed)
{
    file_ << "," << "," 
        << pre_flowtime_ << "," << pre_makespan_ << "," << flowtime_ << "," << makespan_ 
        << "," << ",," << elapsed << "," << "," << t_check_ << "," << n_check_
        << "," << n_valid_ << "," << "," << ","
        << ", " << n_colcheck_ << "," << path_length_ << "," << wait_time_ 
        << "," << smoothness_.normalized_jerk_score << "," << smoothness_.directional_consistency;
    
    if (sampler_ != nullptr && options_.algos.size() > 0) {
        file_ << "," << sampler_->getAlgoCountPercent(0) << "," << sampler_->getAlgoCountPercent(1)
            << "," << sampler_->getAlgoCountPercent(2)
            << "," << sampler_->getAlgoValidPercent(0) << "," << sampler_->getAlgoValidPercent(1)
            << "," << sampler_->getAlgoValidPercent(2) << std::endl;
    }
    else {
        file_ << "," << "," << "," << "," << "," << "," << std::endl;
    }
    return true;
}
