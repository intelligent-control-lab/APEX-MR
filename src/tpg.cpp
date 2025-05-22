#include "tpg.h"
#include "logger.h"

namespace tpg {

ShortcutSampler::ShortcutSampler(const TPGConfig &config) {
    update_config(config);
    std::srand(config.seed);
}

void ShortcutSampler::update_config(const TPGConfig &config) {
    biased_ = config.biased_sample; 
    partial_ = config.subset_shortcut;
    subset_prob_ = config.subset_prob;
    comp_ = config.comp_shortcut;
}

void ShortcutSampler::init(const std::vector<NodePtr> &start_nodes, const std::vector<int> &numNodes,
                            const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<NodePtr>> &timed_nodes) {
    num_robots_ = start_nodes.size();
    nodes_.resize(num_robots_);
    timed_nodes_ = timed_nodes;

    for (int i = 0; i < num_robots_; i++) {
        NodePtr node = start_nodes[i];
        std::vector<NodePtr> nodes_i;
        while (node != nullptr) {
            nodes_i.push_back(node);
            node = node->Type1Next;
        }

        nodes_[i] = nodes_i;
    }
    numNodes_ = numNodes;

    resetFailedShortcuts();
}

bool ShortcutSampler::sample(Shortcut &shortcut, double time_progress) {
    shortcut.subset_indices.clear();
    if (biased_) {
        return sampleBiased(shortcut);
    } else if (comp_) {
        return sampleComposite(shortcut);
    }
    else if (partial_) {
        return samplePartial(shortcut, time_progress);
    } else {
        return sampleUniform(shortcut);
    }
}

bool ShortcutSampler::sampleUniform(Shortcut &shortcut) {
    int i = std::rand() % num_robots_;
    int startNode = std::rand() % numNodes_[i];
    // if (startNode >= numNodes_[i] - 2) {
    //     return false;
    // }
    // int length = std::rand() % (numNodes_[i] - startNode - 2) + 2;
    // int endNode = startNode + length;
    int endNode = std::rand() % numNodes_[i];
    if (endNode < startNode) {
        std::swap(startNode, endNode);
    }
    if (startNode + 1 >= endNode) {
        return false;
    }
    // if (endNode <= 1 || startNode >= endNode - 1) {
    //     return false;
    // }

    shortcut.ni = nodes_[i][startNode];
    shortcut.nj = nodes_[i][endNode];
    return true;
}

bool ShortcutSampler::sampleComposite(Shortcut &shortcut) {

    // bool success = sampleUniform(shortcut);
    // if (!success) {
    //     return false;
    // }
    // int robot_id = shortcut.robot_id();
    // int start_i = shortcut.ni.lock()->timeStep;
    // int end_i = shortcut.nj.lock()->timeStep;
    // int start_t = start_i;
    // while (timed_nodes_[robot_id][start_t]->timeStep < start_i) {
    //     start_t++;
    // }
    // int end_t = std::max(start_t, end_i);
    // while (timed_nodes_[robot_id][end_t]->timeStep < end_i) {
    //     end_t++;
    // }

    int robot_id = 0;
    int start_t = std::rand() % timed_nodes_[robot_id].size();
    int end_t = std::rand() % timed_nodes_[robot_id].size();
    if (end_t < start_t) {
        std::swap(start_t, end_t);
    }
    if ((start_t + 1) >= end_t) {
        return false;
    }
    shortcut.ni = timed_nodes_[robot_id][start_t];
    shortcut.nj = timed_nodes_[robot_id][end_t];
    shortcut.orig_span = end_t - start_t;

    for (int i = 0; i < num_robots_; i++) {
        if (i == robot_id) {
            continue;
        }
        Shortcut shortcut_i;
        shortcut_i.ni = timed_nodes_[i][start_t];
        shortcut_i.nj = timed_nodes_[i][end_t];
        shortcut.comp_shortcuts[i] = shortcut_i;
    }
    return true;
}

bool ShortcutSampler::sampleBiased(Shortcut &shortcut) {
    int i = std::rand() % num_robots_;
    int startNode = std::rand() % numNodes_[i];
    if (startNode >= numNodes_[i] - 2) {
        return false;
    }
    int length = std::rand() % (numNodes_[i] - startNode - 2) + 2;
    int endNode = startNode + length;
    // if (endNode <= 1 || startNode >= endNode - 1) {
    //     return false;
    // }


    for (int k = 0; k < already_shortcuts_.size(); k++) {
        NodePtr k1 = already_shortcuts_[k].ni.lock();
        NodePtr k2 = already_shortcuts_[k].nj.lock();
        if (k1->robotId == i) {
            int s = k1->timeStep;
            int e = k2->timeStep;
            if (startNode >= s && endNode <= e) {
                return false;
            }
        }
    }
    for (int k = 0; k < untight_shortcuts_.size(); k++) {
        NodePtr k1 = untight_shortcuts_[k].ni.lock();
        NodePtr k2 = untight_shortcuts_[k].nj.lock();
        if (k1->robotId == i) {
            int s = k1->timeStep;
            int e = k2->timeStep;
            if (startNode >= s && endNode <= e) {
                return false;
            }
        }
    }
    

    double prob = 1.0;

    for (int k = 0; k < failed_shortcuts_.size(); k++) {
        NodePtr k1 = failed_shortcuts_[k].ni.lock();
        NodePtr k2 = failed_shortcuts_[k].nj.lock();
        if (k1->robotId == i) {
            int s = k1->timeStep;
            int e = k2->timeStep;
            //double prob_k = std::min(1.0, (std::pow(s - startNode, 2) + std::pow(e - endNode, 2))/ (scale_ * scale_));
            double prob_k = std::min(1.0, (std::max(0, startNode -s) + std::max(0, e - endNode)) / scale_);
            prob = prob * prob_k;
        }
    }
    for (int k = 0; k < failed_shortcuts_static_.size(); k++) {
        NodePtr k1 = failed_shortcuts_static_[k].ni.lock();
        NodePtr k2 = failed_shortcuts_static_[k].nj.lock();
        if (k1->robotId == i) {
            int s = k1->timeStep;
            int e = k2->timeStep;
            //double prob_k = std::min(1.0, (std::pow(s - startNode, 2) + std::pow(e - endNode, 2))/ (scale_ * scale_));
            double prob_k = std::min(1.0, (std::max(0, startNode -s) + std::max(0, e - endNode)) / scale_);
            prob = prob * prob_k;
        }
    }
    
    // drop the sample based on the probability
    int s = std::rand() % 1000;
    prob = prob * 1000;
    log("Sample prob: " + std::to_string(prob) + " random: " + std::to_string(s), LogLevel::DEBUG);

    if (s >= prob) {
        return false;
    }
    shortcut.ni = nodes_[i][startNode];
    shortcut.nj = nodes_[i][endNode];
    return true;
}

bool ShortcutSampler::samplePartial(Shortcut &shortcut, double time_progress) {
    bool success = sampleUniform(shortcut);
    if (!success)
        return false;

    // sample the subset
    NodePtr node_i = shortcut.ni.lock();
    int robot = node_i->robotId;
    int dof = node_i->pose.joint_values.size();

    // iterate each dimension, append it to subset_indices if the sample prob [0, 1] is less than subset_prob_
    double prob_t = 1 - (1-subset_prob_) * std::pow(time_progress, 7);
    if (prob_t >= 0.999) {
        return true;
    }

    for (int i = 0; i < dof; i++) {
        double s = std::rand() % 1000;
        if (s < prob_t * 1000) {
            shortcut.subset_indices.push_back(i);
        }
    }
    if (shortcut.subset_indices.size() == dof) {
        shortcut.subset_indices.clear(); // clear the subset indices if all dimensions are selected
        return true;
    }
    if (shortcut.subset_indices.size() == 0) {
        return false;
    }

    return true;
}

void ShortcutSampler::resetFailedShortcuts() {
    untight_shortcuts_.clear();

    // loop over already_shortcuts_ and failed_shortcuts_static_ to remove any entries with weak pointer
    for (int i = already_shortcuts_.size() - 1; i >=0; i--) {
        if (already_shortcuts_[i].expired()) {
            already_shortcuts_.erase(already_shortcuts_.begin() + i);
        }
    }
    for (int i = failed_shortcuts_static_.size() - 1; i >=0; i--) {
        if (failed_shortcuts_static_[i].expired()) {
            failed_shortcuts_static_.erase(failed_shortcuts_static_.begin() + i);
        }
    }
    for (int i = failed_shortcuts_.size() - 1; i >=0; i--) {
        if (failed_shortcuts_[i].expired() || failed_shortcuts_[i].n_robot_col.expired()) {
            failed_shortcuts_.erase(failed_shortcuts_.begin() + i);
        }
    }
}

void ShortcutSampler::updateFailedShortcut(const Shortcut &shortcut) {
    if (!biased_) {
        if (shortcut.col_type == CollisionType::STATIC) {
            failed_shortcuts_static_.push_back(shortcut);
        }
        else if (shortcut.col_type == CollisionType::NO_NEED) {
            already_shortcuts_.push_back(shortcut);
        }
        else if (shortcut.col_type == CollisionType::ROBOT) {
            failed_shortcuts_.push_back(shortcut);
        }
        else if (shortcut.col_type == CollisionType::UNTIGHT) {
            untight_shortcuts_.push_back(shortcut);
        }
    }
}

ShortcutIterator::ShortcutIterator(const TPGConfig &config) {
    backward_doubleloop = config.backward_doubleloop;
    forward_doubleloop = config.forward_doubleloop;
    forward_singleloop = config.forward_singleloop;
}

void ShortcutIterator::init(const std::vector<NodePtr> &start_nodes, 
                            const std::vector<int> &numNodes,
                            const std::vector<NodePtr> &end_nodes) {
    q_i = std::queue<NodePtr>();
    q_j = std::queue<NodePtr>();
    num_robots_ = start_nodes.size();
    start_nodes_ = start_nodes;
    end_nodes_ = end_nodes;
    numNodes_ = numNodes;

    for (int i = 0; i < num_robots_; i++) {
        q_i.push(start_nodes_[i]);
        if (backward_doubleloop) {
            q_j.push(end_nodes_[i]);
        }
        else {
            q_j.push(start_nodes_[i]->Type1Next);
        }
    }
}

bool ShortcutIterator::step_begin(Shortcut &shortcut) {
    NodePtr node_i = q_i.front();
    NodePtr node_j = q_j.front();
    q_i.pop();
    q_j.pop();

    if (backward_doubleloop) {
        if (node_i == node_j) {
            q_i.push(start_nodes_[node_i->robotId]);
            q_j.push(end_nodes_[node_i->robotId]);

            return false;
        }
        else if (node_i->Type1Next == node_j) {
            q_i.push(node_i->Type1Next);
            q_j.push(end_nodes_[node_i->robotId]);

            return false;
        }
    }
    else {
        if (node_j == nullptr) {
            if (node_i->Type1Next == nullptr) {
                q_i.push(start_nodes_[node_i->robotId]);
                q_j.push(start_nodes_[node_i->robotId]->Type1Next);
            }
            else {
                q_i.push(node_i->Type1Next);
                q_j.push(node_i->Type1Next->Type1Next);
            }
            
            return false;
        }
        else if (node_i->Type1Next == node_j) {
            q_i.push(node_i);
            q_j.push(node_j->Type1Next);
            
            return false;
        }
    }

    shortcut.ni = node_i;
    shortcut.nj = node_j;
    return true;
}

void ShortcutIterator::step_end(const Shortcut &shortcut) {
    NodePtr node_i = shortcut.ni.lock();
    NodePtr node_j = shortcut.nj.lock();
    if (backward_doubleloop) {
        q_i.push(node_i);
        q_j.push(node_j->Type1Prev);
    }
    else if (forward_doubleloop) {
        q_i.push(node_i);
        q_j.push(node_j->Type1Next);
    }
    else if (forward_singleloop) {
        if (shortcut.col_type == CollisionType::NONE) {
            q_i.push(node_j);
            q_j.push(node_j->Type1Next);
        }
        else {
            q_i.push(node_i);
            q_j.push(node_j->Type1Next);
        }
    }
}

void TPG::reset() {
    start_nodes_.clear();
    end_nodes_.clear();
    numNodes_.clear();
    collisionCheckMatrix_.clear();
    type2Edges_.clear();
    solution_.clear();
    idType2Edges_ = 0;
    num_robots_ = 0;
    joint_states_.clear();
    executed_steps_.clear();
    pre_shortcut_flowtime_ = 0.0;
    pre_shortcut_makespan_ = 0.0;
    post_shortcut_flowtime_ = 0.0;
    post_shortcut_makespan_ = 0.0;
    t_shortcut_ = 0.0;
    t_init_ = 0.0;
    t_simplify_ = 0.0;
    t_shortcut_check_ = 0.0;
    num_shortcut_checks_ = 0;
    num_valid_shortcuts_ = 0;
    flowtime_improv_ = 0.0;
    numtype2edges_pre_ = 0;
    num_colcheck_pre_ = 0;
    numtype2edges_post_ = 0;
    num_colcheck_post_ = 0;
}

TPG::TPG(const TPG &tpg) {
    config_ = tpg.config_;
    dt_ = tpg.dt_;
    num_robots_ = tpg.num_robots_;
    type2Edges_ = tpg.type2Edges_;
    start_nodes_ = tpg.start_nodes_;
    end_nodes_ = tpg.end_nodes_;
    numNodes_ = tpg.numNodes_;
    solution_ = tpg.solution_;
    pre_shortcut_flowtime_ = tpg.pre_shortcut_flowtime_;
    pre_shortcut_makespan_ = tpg.pre_shortcut_makespan_;
    post_shortcut_flowtime_ = tpg.post_shortcut_flowtime_;
    post_shortcut_makespan_ = tpg.post_shortcut_makespan_;
    t_shortcut_ = tpg.t_shortcut_;
    t_init_ = tpg.t_init_;
    t_simplify_ = tpg.t_simplify_;
    t_shortcut_check_ = tpg.t_shortcut_check_;
    num_shortcut_checks_ = tpg.num_shortcut_checks_;
    collisionCheckMatrix_ = tpg.collisionCheckMatrix_;
}

bool TPG::init(std::shared_ptr<PlanInstance> instance, const MRTrajectory &solution,
    const TPGConfig &config) {
    dt_ = config.dt;
    config_ = config;
    num_robots_ = instance->getNumberOfRobots();
    solution_ = solution;

    instance->numCollisionChecks(); // reset the number of collision checks in the instance 
    auto t_start = std::chrono::high_resolution_clock::now();

    // 1. populate type 1 nodes
    for (int i = 0; i < num_robots_; i++) {
        double cost = solution_[i].cost;
        int numNodes = std::ceil(cost / dt_) + 1;
        int ind = 0;
        
        std::vector<NodePtr> nodes_i;
        for (int j = 0; j < numNodes; j++) {
            double time = j * dt_;
            while (ind + 1 < solution_[i].times.size() && solution_[i].times[ind+1] <= time) {
                ind++;
            }
            RobotPose pose_j = instance->initRobotPose(i);
            if (ind + 1 == solution_[i].times.size()) {
                // assuming obstacle stays at the end of the trajectory
                pose_j = solution_[i].trajectory[ind];
            } else {
                double alpha = (time - solution_[i].times[ind]) / (solution_[i].times[ind + 1] - solution_[i].times[ind]);
                pose_j = instance->interpolate(solution_[i].trajectory[ind], solution_[i].trajectory[ind + 1], alpha);
            }

            NodePtr node = std::make_shared<Node>(i, j);
            node->pose = pose_j;
            nodes_i.push_back(node);
        }

        for (int j = 0; j < numNodes; j++) {
            if (j < numNodes - 1) {
                nodes_i[j]->Type1Next = nodes_i[j + 1];
            }
            if (j > 0) {
                nodes_i[j]->Type1Prev = nodes_i[j - 1];
            }
        }

        start_nodes_.push_back(nodes_i[0]);
        end_nodes_.push_back(nodes_i.back());
        numNodes_.push_back(numNodes);
    }

    // 2. populate type 2 edges
    if (!findCollisionDepsParallel(instance, solution, config)) {
        return false;
    }

    t_init_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-9;

    // 4. Simplify the edges with MCP algorithm
    t_start = std::chrono::high_resolution_clock::now();
    
    transitiveReduction();
    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }
    
    numtype2edges_pre_ = getTotalType2Edges();
    t_simplify_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-9;  
    
    num_colcheck_pre_ = instance->numCollisionChecks();

    findFlowtimeMakespan(pre_shortcut_flowtime_, pre_shortcut_makespan_);

    computePathLength(instance);
    smoothness_ = calculate_smoothness(getSyncJointTrajectory(instance), instance);

    double nowait_time = 0;
    for (int i = 0; i < num_robots_; i++) {
        nowait_time += end_nodes_[i]->timeStep * config_.dt;
    }
    wait_time_ = pre_shortcut_flowtime_ - nowait_time;

    log("TPG initialized with "  + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges_pre_)
        + " type 2 edges, makespan " + std::to_string(pre_shortcut_makespan_) + " s", LogLevel::HLINFO);
    log("in " + std::to_string(t_init_ + t_simplify_) + " s, " + std::to_string(num_colcheck_pre_) + " col checks", LogLevel::HLINFO);


    return true;
}

bool TPG::findCollisionDeps(std::shared_ptr<PlanInstance> instance, const MRTrajectory &solution,
    const TPGConfig &config) {
    for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < num_robots_; j++) {
            if (i == j) {
                continue;
            }
            NodePtr node_i = start_nodes_[i];
            NodePtr node_j_start = start_nodes_[j];
            while (node_i != nullptr) {
                NodePtr node_j = node_j_start;
                bool inCollision = false;
                while (node_j != nullptr && node_j->timeStep < node_i->timeStep) {
                    if (instance->checkCollision({node_i->pose, node_j->pose}, true)) {
                        inCollision = true;
                    } else if (inCollision) {
                        inCollision = false;
                        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                        edge->edgeId = idType2Edges_;
                        if (config_.one_robust) {
                            edge->nodeFrom = node_j;
                        }
                        else {
                            edge->nodeFrom = node_j->Type1Prev;
                        }
                        edge->nodeTo = node_i;
                        if (config_.one_robust) {
                            node_j->Type2Next.push_back(edge);
                        }
                        else {
                            node_j->Type1Prev->Type2Next.push_back(edge);
                        }
                        node_i->Type2Prev.push_back(edge);
                        idType2Edges_++;
                        node_j_start = node_j;
                    }
                    node_j = node_j->Type1Next;
                }
                if (inCollision) {
                    if (node_j == nullptr) {
                        log("TPG has target collision: robot " + std::to_string(j) + " (at the end " + std::to_string(numNodes_[j]) 
                            +  ") blocking robot " + std::to_string(i)
                            + " at time " + std::to_string(node_i->timeStep), LogLevel::ERROR);
                        return false;
                    }
                    std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                    edge->edgeId = idType2Edges_;
                    if (config_.one_robust) {
                        edge->nodeFrom = node_j;
                    }
                    else {
                        edge->nodeFrom = node_j->Type1Prev;
                    }
                    edge->nodeTo = node_i;
                    if (config_.one_robust) {
                        node_j->Type2Next.push_back(edge);
                    }
                    else {
                        node_j->Type1Prev->Type2Next.push_back(edge);
                    }
                    node_i->Type2Prev.push_back(edge);
                    idType2Edges_++;
                    node_j_start = node_j;
                }
                node_i = node_i->Type1Next;
            }
        }
    }

    return true;
}

bool TPG::findCollisionDepsParallel(std::shared_ptr<PlanInstance> instance, const MRTrajectory &solution,
    const TPGConfig &config) {
    struct CollisionResult {
        int robot_i;
        int robot_j;
        NodePtr node_i;
        NodePtr node_j;
        bool is_exit_collision;  // True if this is an exit point from collision
        bool is_end_collision;   // True if this is an end-of-sequence collision
    };

    std::vector<CollisionResult> all_collision_results;
    std::mutex results_mutex;
    
    // Parallelize the outer robot pair loops
    #pragma omp parallel
    {
        std::vector<CollisionResult> thread_results;
        
        #pragma omp for collapse(2)
        for (int i = 0; i < num_robots_; i++) {
            for (int j = 0; j < num_robots_; j++) {
                if (i == j) {
                    continue;
                }
                
                // For each robot pair, collect all nodes for processing
                std::vector<NodePtr> nodes_i;
                NodePtr temp_i = start_nodes_[i];
                while (temp_i != nullptr) {
                    nodes_i.push_back(temp_i);
                    temp_i = temp_i->Type1Next;
                }
                
                // Process each node_i
                for (NodePtr node_i : nodes_i) {
                    NodePtr node_j_start = start_nodes_[j];
                    
                    // Collect all relevant node_j candidates for collision checking
                    std::vector<NodePtr> check_nodes;
                    NodePtr node_j = node_j_start;
                    while (node_j != nullptr && node_j->timeStep < node_i->timeStep) {
                        check_nodes.push_back(node_j);
                        node_j = node_j->Type1Next;
                    }
                    
                    if (check_nodes.empty()) {
                        continue;
                    }
                    
                    // Inner parallel collision detection
                    std::vector<bool> collision_results(check_nodes.size(), false);
                    
                    #pragma omp parallel for if(check_nodes.size() > 10)
                    for (int idx = 0; idx < check_nodes.size(); idx++) {
                        collision_results[idx] = instance->checkCollision(
                            {node_i->pose, check_nodes[idx]->pose}, true);
                    }
                    
                    // Process results to identify collision transitions
                    bool inCollision = false;
                    
                    for (int idx = 0; idx < check_nodes.size(); idx++) {
                        if (collision_results[idx]) {
                            inCollision = true;
                        } else if (inCollision) {
                            // Found collision exit point
                            inCollision = false;
                            thread_results.push_back({
                                i, j, node_i, check_nodes[idx], true, false
                            });
                        }
                    }
                    
                    // Handle end-of-sequence collision
                    if (inCollision && node_j != nullptr) {
                        thread_results.push_back({
                            i, j, node_i, node_j, false, true
                        });
                    } else if (inCollision) {
                        // Collision continues to the end with no node_j
                        // This would be an error condition - store for later reporting
                        thread_results.push_back({
                            i, j, node_i, nullptr, false, true
                        });
                    }
                }
            }
        }
        
        // Combine thread results
        #pragma omp critical
        {
            all_collision_results.insert(all_collision_results.end(), 
                                        thread_results.begin(), 
                                        thread_results.end());
        }
    }
    
    // Now process all collision results and create type2 edges
    for (const auto& result : all_collision_results) {
        if (result.node_j == nullptr) {
            log("TPG has target collision: robot " + std::to_string(result.robot_j) + 
                " (at the end " + std::to_string(numNodes_[result.robot_j]) + 
                ") blocking robot " + std::to_string(result.robot_i) + 
                " at time " + std::to_string(result.node_i->timeStep), LogLevel::ERROR);
            return false;
        }
        
        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
        edge->edgeId = idType2Edges_;
        
        if (config_.one_robust) {
            edge->nodeFrom = result.node_j;
        } else {
            edge->nodeFrom = result.node_j->Type1Prev;
        }
        
        edge->nodeTo = result.node_i;
        
        if (config_.one_robust) {
            result.node_j->Type2Next.push_back(edge);
        } else {
            result.node_j->Type1Prev->Type2Next.push_back(edge);
        }
        
        result.node_i->Type2Prev.push_back(edge);
        idType2Edges_++;
    }
    return true;
}

bool TPG::optimize(std::shared_ptr<PlanInstance> instance, const TPGConfig &config) {
    config_ = config;
    instance->numCollisionChecks(); // reset the number of collision checks in the instance 
    num_colcheck_post_ = 0;
    auto t_start = std::chrono::high_resolution_clock::now();

    if (config.shortcut && config.shortcut_time > 0) {
        if (config.random_shortcut) {
            findShortcutsRandom(instance, config.shortcut_time);
        } else {
            findShortcuts(instance, config.shortcut_time);
        }

        t_shortcut_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-9;
        
        if (hasCycle()) {
            log("TPG has cycle after shortcut", LogLevel::ERROR);
            return false;
        }
        transitiveReduction();
        numtype2edges_post_ = getTotalType2Edges();

        if (config.switch_shortcut) {
            switchShortcuts();
        }

        // trajectory_msgs::JointTrajectory joint_traj;
        // size_t num_joints = 0;
        // for (int i = 0; i < num_robots_; i++ ) {
        //     num_joints += instance->getRobotDOF(i);
        // }
        // joint_traj.joint_names.resize(num_joints);
        // setSyncJointTrajectory(joint_traj, post_shortcut_flowtime_, post_shortcut_makespan_);
        findFlowtimeMakespan(post_shortcut_flowtime_, post_shortcut_makespan_);
        computePathLength(instance);
        smoothness_ = calculate_smoothness(getSyncJointTrajectory(instance), instance);

        log ("TPG after finding shortcuts: " + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges_post_) 
            + " type 2 edges, makespan " + std::to_string(post_shortcut_makespan_) + " s", LogLevel::HLINFO);
        log ("in " + std::to_string(t_shortcut_) + " s, " + std::to_string(num_colcheck_post_) + " col checks.", LogLevel::HLINFO);
    }

    // 5. Print the TPG for debugging purposes
    // Eigen::MatrixXi col_matrix_ij;
    // getCollisionCheckMatrix(0, 1, col_matrix_ij);
    // std::cout << "Collision matrix between 0 and 1" << std::endl;
    // std::cout << col_matrix_ij << std::endl;

    // for (int i = 0; i < num_robots_; i++) {
    //     NodePtr node_i = start_nodes_[i];
    //     while (node_i != nullptr) {
    //         log("Robot " + std::to_string(i) + " at time " + std::to_string(node_i->timeStep), LogLevel::DEBUG);
    //         log("Type 1 Next: " + ((node_i->Type1Next != nullptr) ? std::to_string(node_i->Type1Next->timeStep) : ""), LogLevel::DEBUG);
    //         log("Type 1 Prev: " + ((node_i->Type1Prev != nullptr) ? std::to_string(node_i->Type1Prev->timeStep) : ""), LogLevel::DEBUG);
    //         for (auto edge : node_i->Type2Next) {
    //             log("Type 2 Next: " + std::to_string(edge->nodeTo->robotId) + " " + std::to_string(edge->nodeTo->timeStep), LogLevel::DEBUG);
    //         }
    //         for (auto edge : node_i->Type2Prev) {
    //             log("Type 2 Prev: " + std::to_string(edge->nodeFrom->robotId) + " " + std::to_string(edge->nodeFrom->timeStep), LogLevel::DEBUG);
    //         }
    //         node_i = node_i->Type1Next;
    //     }

    // }

    return true;
}

int TPG::getTotalNodes() const {
    int total_nodes = 0;
    for (int i = 0; i < num_robots_; i++) {
        total_nodes += numNodes_[i];
    }
    return total_nodes;
}

int TPG::getTotalType2Edges() const {
    int total_edges = 0;
    for (int i = 0; i < num_robots_; i++) {
        NodePtr node_i = start_nodes_[i];
        while (node_i != nullptr) {
            total_edges += node_i->Type2Next.size();
            node_i = node_i->Type1Next;
        }
    }
    return total_edges;
}

void TPG::saveStats(const std::string &filename, const std::string &start_pose, const std::string &goal_pose) const {
    std::ofstream file(filename, std::ios::app);
    file << start_pose << "," << goal_pose << "," 
        << pre_shortcut_flowtime_ << "," << pre_shortcut_makespan_ << "," << post_shortcut_flowtime_ << "," << post_shortcut_makespan_ 
        << ",," << t_init_ << "," << t_shortcut_ << "," << t_simplify_ << "," << t_shortcut_check_ << "," << num_shortcut_checks_
        << "," << num_valid_shortcuts_ << "," << numtype2edges_pre_ << "," << num_colcheck_pre_ << "," << numtype2edges_post_
        << ", " << num_colcheck_post_ << "," << path_length_ << "," << wait_time_ 
        << "," << smoothness_.normalized_jerk_score << "," << smoothness_.directional_consistency << std::endl;
    file.close();
}

void TPG::findShortcuts(std::shared_ptr<PlanInstance> instance, double runtime_limit)
{
    // for every robot
    // for every pair of nodes
    // check if this is free of robot-space/robot-object collisions
    // find the dependent parent nodes and child nodes
    // for every other node that is independent with the current pair
    // compute the collision matrix
    // if there is no collision, add the shortcut and remove old nodes
    // add the new nodes to the list of nodes

    double elapsed = 0;
    double log_limit = config_.log_interval;
    double log_interval = config_.log_interval;

    initIterator();

    std::vector<std::vector<int>> earliest_t, latest_t;
    std::vector<int> reached_end, updated_reached_end;
    std::vector<std::vector<NodePtr>> timed_nodes;
    findEarliestReachTime(earliest_t, reached_end);
    findTimedNodes(earliest_t, timed_nodes);

    if (config_.tight_shortcut) {
        findLatestReachTime(latest_t, reached_end);
        findTightType2Edges(earliest_t, latest_t);
    }

    if (runtime_limit > 0) {
        computePathLength(instance);
        smoothness_ = calculate_smoothness(getSyncJointTrajectory(instance), instance);
        logProgressStep(reached_end);
    }

    while (elapsed < runtime_limit) {
        auto tic = std::chrono::high_resolution_clock::now();

        Shortcut shortcut;
        bool valid = shortcut_iterator_->step_begin(shortcut);
        if (!valid) {
            auto toc = std::chrono::high_resolution_clock::now();
            elapsed += std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
            continue;
        }

        int robot_id = shortcut.ni.lock()->robotId;
        
        preCheckShortcuts(instance, shortcut, earliest_t[robot_id], latest_t[robot_id]);
        if (shortcut.col_type == CollisionType::NONE) {
            std::vector<Eigen::MatrixXi> col_matrix;

            auto tic_inner = std::chrono::high_resolution_clock::now();
            checkShortcuts(instance, shortcut, timed_nodes);
            auto inner = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - tic_inner).count();
            t_shortcut_check_ += (inner * 1e-9);
            num_shortcut_checks_++;
            num_colcheck_post_ += instance->numCollisionChecks();

            if (shortcut.col_type == CollisionType::NONE) {
                auto node_i = shortcut.ni.lock();
                auto node_j = shortcut.nj.lock();
                log("found shortcut for robot " + std::to_string(robot_id) + " of length " + std::to_string(shortcut.path.size()), LogLevel::DEBUG);
                log("from " + std::to_string(node_i->timeStep) + " to " + std::to_string(node_j->timeStep), LogLevel::DEBUG);
                // add the shortcut
                updateTPG(instance, shortcut, col_matrix);
                if (shortcut.composite()) {
                    for (auto it = shortcut.comp_shortcuts.begin(); it != shortcut.comp_shortcuts.end(); it++) {
                        updateTPG(instance, it->second, col_matrix);
                    }
                }
                if (shortcut.composite() || config_.allow_col) {
                    updateEarliestReachTime(earliest_t, updated_reached_end, shortcut);
                    if (!repairTPG(instance, shortcut, earliest_t)) {
                        log("Failed to repair TPG", LogLevel::ERROR);
                        return;
                    }
                }

                // calculate statistics
                findEarliestReachTime(earliest_t, updated_reached_end);
                findTimedNodes(earliest_t, timed_nodes);
                int flowtime_diff = 0;
                for (int ri = 0; ri < num_robots_; ri++) {
                    flowtime_diff += (reached_end[ri] - updated_reached_end[ri]); 
                }
                flowtime_improv_ += (flowtime_diff * dt_);

                reached_end = updated_reached_end;
                if (config_.tight_shortcut) {
                    findLatestReachTime(latest_t, reached_end);
                    findTightType2Edges(earliest_t, latest_t);
                }
                num_valid_shortcuts_++;
            }
        }

        shortcut_iterator_->step_end(shortcut);

        auto toc = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
        while (elapsed > log_limit) {
            t_shortcut_ = elapsed;
            computePathLength(instance);
            smoothness_ = calculate_smoothness(getSyncJointTrajectory(instance), instance);
            logProgressStep(reached_end);
            log_limit += log_interval;
        }
    }
    // findEarliestReachTime(earliest_t, reached_end);
    // logProgressStep(reached_end);

}

void TPG::initSampler(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<NodePtr>> &timed_nodes) {
    shortcut_sampler_ = std::make_unique<ShortcutSampler>(config_);
    shortcut_sampler_->init(start_nodes_, numNodes_, earliest_t, timed_nodes);
}

void TPG::initIterator() {
    shortcut_iterator_ = std::make_unique<ShortcutIterator>(config_);
    shortcut_iterator_->init(start_nodes_, numNodes_, end_nodes_);
}

void TPG::logProgressStep(const std::vector<int>& reached_end) {
    if (config_.progress_file != "") {
        int flowspan_i = 0;
        int makespan_i = 0;
        wait_time_ = 0;
        for (int ri = 0; ri < num_robots_; ri++) {
            flowspan_i += reached_end[ri];
            makespan_i = std::max(makespan_i, reached_end[ri]);
            wait_time_ += (reached_end[ri] - end_nodes_[ri]->timeStep) * config_.dt;
        }

        post_shortcut_flowtime_ = flowspan_i * dt_;
        post_shortcut_makespan_ = makespan_i * dt_;
        numtype2edges_post_ = getTotalType2Edges();

        saveStats(config_.progress_file);
    }
}

void TPG::computePathLength(std::shared_ptr<PlanInstance> instance) {
    path_length_ = 0;

    for (int ri = 0; ri < num_robots_; ri++) {
        NodePtr node_i = start_nodes_[ri]->Type1Next;
        while (node_i != nullptr) {
            path_length_ += instance->computeDistance(node_i->pose, node_i->Type1Prev->pose);
            node_i = node_i->Type1Next;
        }
    }
}

void TPG::findShortcutsRandom(std::shared_ptr<PlanInstance> instance, double runtime_limit) {

    // randomly sample shortcuts and check if they are valid for time
    double elapsed = 0;
    double log_limit = config_.log_interval;
    double log_interval = config_.log_interval;
    double elapsed_sc = 0;
    
    std::vector<std::vector<int>> earliest_t, latest_t;
    std::vector<int> reached_end, updated_reached_end;
    std::vector<std::vector<NodePtr>> timed_nodes;
    findEarliestReachTime(earliest_t, reached_end);
    findTimedNodes(earliest_t, timed_nodes);
    if (config_.tight_shortcut) {
        findLatestReachTime(latest_t, reached_end);
        findTightType2Edges(earliest_t, latest_t);
    }

    if (runtime_limit > 0) {
        computePathLength(instance);
        smoothness_ = calculate_smoothness(getSyncJointTrajectory(instance), instance);
        logProgressStep(reached_end);
    }

    initSampler(earliest_t, timed_nodes);

    while (elapsed < runtime_limit) {
        auto tic = std::chrono::high_resolution_clock::now();

        Shortcut shortcut;
        bool valid = shortcut_sampler_->sample(shortcut, elapsed / runtime_limit);
        if (!valid) {
            auto toc = std::chrono::high_resolution_clock::now();
            elapsed += (std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9);
            continue;
        }
        assert (!shortcut.expired());

        int robot_id = shortcut.robot_id();
        preCheckShortcuts(instance, shortcut, earliest_t[robot_id], latest_t[robot_id]);
        if (shortcut.col_type == CollisionType::NONE) {
            std::vector<Eigen::MatrixXi> col_matrix;

            auto tic_inner = std::chrono::high_resolution_clock::now();
            checkShortcuts(instance, shortcut, timed_nodes);
            auto inner = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - tic_inner).count();
            t_shortcut_check_ += (inner * 1e-9);
            num_shortcut_checks_++;
            num_colcheck_post_ += instance->numCollisionChecks();

            log("Time taken for checking shortcuts: " + std::to_string(inner) + " ns", LogLevel::DEBUG);

            if (shortcut.col_type == CollisionType::NONE) {
                // add the shortcut
                NodePtr node_i = shortcut.ni.lock();
                NodePtr node_j = shortcut.nj.lock();
                log("found shortcut for robot " + std::to_string(robot_id) + " of length " + std::to_string(shortcut.path.size()), LogLevel::DEBUG);
                log("from " + std::to_string(node_i->timeStep) + " to " + std::to_string(node_j->timeStep), LogLevel::DEBUG);
                
                updateTPG(instance, shortcut, col_matrix);
                if (shortcut.composite()) {
                    for (auto it = shortcut.comp_shortcuts.begin(); it != shortcut.comp_shortcuts.end(); it++) {
                        updateTPG(instance, it->second, col_matrix);
                    }
                }
                if (shortcut.composite() || config_.allow_col) {
                    updateEarliestReachTime(earliest_t, updated_reached_end, shortcut);
                    if (!repairTPG(instance, shortcut, earliest_t)) {
                        log("Failed to repair TPG", LogLevel::ERROR);
                        return;
                    }
                }
                if (config_.debug_graph) {
                    saveToDotFile("tpg_shortcut" + std::to_string(num_valid_shortcuts_) + ".dot");
                }

                findEarliestReachTime(earliest_t, updated_reached_end);
                findTimedNodes(earliest_t, timed_nodes);
                shortcut_sampler_->init(start_nodes_, numNodes_, earliest_t, timed_nodes);

                // calculate statistics
                int flowtime_diff = 0;
                for (int ri = 0; ri < num_robots_; ri++) {
                    flowtime_diff += (reached_end[ri] - updated_reached_end[ri]); 
                }
                flowtime_improv_ += (flowtime_diff * dt_);

                reached_end = updated_reached_end;
                if (config_.tight_shortcut) {
                    findLatestReachTime(latest_t, reached_end);
                    findTightType2Edges(earliest_t, latest_t);
                }
                num_valid_shortcuts_++;
                elapsed_sc = elapsed;
            }
        }
        if (shortcut.col_type != CollisionType::NONE) {
            shortcut_sampler_->updateFailedShortcut(shortcut);
        }

        if (config_.comp_shortcut && (elapsed - elapsed_sc > 2.0)) {
            config_.comp_shortcut = false;
            shortcut_sampler_->update_config(config_);
            log("Switching to non-composite shortcuts", LogLevel::INFO);
        }
        auto toc = std::chrono::high_resolution_clock::now();
        elapsed += (std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9);
        while (elapsed > log_limit) {
            t_shortcut_ = elapsed;
            computePathLength(instance);
            smoothness_ = calculate_smoothness(getSyncJointTrajectory(instance), instance);
            logProgressStep(reached_end);
            log_limit += log_interval;
        }
    }

    // findEarliestReachTime(earliest_t, reached_end);
    // logProgressStep(reached_end);

    assert(hasCycle() == false);
}

int TPG::computeShortcutSteps(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut) const {
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();
    int shortcutSteps;

    if (shortcut.partial()) {
        // partial shortcut 
        double distance = 0;
        std::vector<RobotPose> paths;
        
        paths.push_back(ni->pose);
        NodePtr node_i = ni->Type1Next;
        while (node_i != nullptr && node_i != nj) {
            RobotPose pose_i = instance->initRobotPose(ni->robotId);
            pose_i.joint_values = node_i->pose.joint_values;
            pose_i.hand_values = node_i->pose.hand_values;
            double alpha = (node_i->timeStep - ni->timeStep) * 1.0 / (nj->timeStep - ni->timeStep);
            for (int i = 0; i < shortcut.subset_indices.size(); i++) {
                int dim = shortcut.subset_indices[i];
                pose_i.joint_values[dim] = instance->interpolate(ni->pose, nj->pose, alpha, dim);
            }
            node_i = node_i->Type1Next;
            paths.push_back(pose_i);
        }
        paths.push_back(nj->pose);

        // retime the shortcut
        for (int i = 0; i < paths.size() - 1; i++) {
            distance += instance->computeDistance(paths[i], paths[i+1]);
        }

        // remove the first and last element in paths
        paths.pop_back();
        paths.erase(paths.begin());

        shortcutSteps = std::ceil(distance / instance->getVMax(ni->robotId) / dt_) + 1;
        shortcut.path = paths;
    }
    else {
        double timeNeeded = instance->computeDistance(ni->pose, nj->pose) / instance->getVMax(ni->robotId);
        timeNeeded = std::ceil(timeNeeded / dt_) * dt_;
        shortcutSteps = timeNeeded / dt_ + 1;
        
        shortcut.path.clear();
        for (int i = 1; i < shortcutSteps - 1; i++) {
            double alpha = i * 1.0 / (shortcutSteps - 1);
            RobotPose pose_i = instance->interpolate(ni->pose, nj->pose, alpha);
            shortcut.path.push_back(pose_i);
        }
    }
    return shortcutSteps;
}

void TPG::retimeShortcut(std::shared_ptr<PlanInstance> instance, int shortcutSteps, Shortcut &shortcut) const {
    NodePtr ni = shortcut.ni.lock();
    NodePtr nj = shortcut.nj.lock();

    if (shortcut.composite()) {
        std::vector<RobotPose> paths = shortcut.path;
        shortcut.path.clear();
        paths.insert(paths.begin(), ni->pose);
        paths.push_back(nj->pose);
        for (int i = 1; i < shortcutSteps - 1; i++) {
            double alpha = (paths.size() - 1) * i * 1.0 / (shortcutSteps - 1);
            int ind = static_cast<int>(alpha);
            RobotPose pose_i = instance->interpolate(paths[ind], paths[ind+1], alpha - ind);
            shortcut.path.push_back(pose_i);
        }
        for (auto it = shortcut.comp_shortcuts.begin(); it != shortcut.comp_shortcuts.end(); it++) {
            paths = it->second.path;
            it->second.path.clear();
            paths.insert(paths.begin(), it->second.ni.lock()->pose);
            paths.push_back(it->second.nj.lock()->pose);
            for (int i = 1; i < shortcutSteps - 1; i++) {
                double alpha = (paths.size() - 1) * i * 1.0 / (shortcutSteps - 1);
                int ind = static_cast<int>(alpha);
                RobotPose pose_i = instance->interpolate(paths[ind], paths[ind+1], alpha - ind);
                it->second.path.push_back(pose_i);
            }
        }
    }
    else if (shortcut.partial()) {
        std::vector<RobotPose> paths = shortcut.path;
        shortcut.path.clear();
        paths.insert(paths.begin(), ni->pose);
        paths.push_back(nj->pose);
        for (int i = 1; i < shortcutSteps - 1; i++) {
            double alpha = (paths.size() - 1) * i * 1.0 / (shortcutSteps - 1);
            int ind = static_cast<int>(alpha);
            RobotPose pose_i = instance->interpolate(paths[ind], paths[ind+1], alpha - ind);
            shortcut.path.push_back(pose_i);
        }
    }
}

void TPG::preCheckShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
        const std::vector<int> &earliest_t, const std::vector<int> &latest_t) const {
    
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();

    int shortcutSteps;
    if (shortcut.composite()) {
        if (shortcut.orig_span < 2) {
            shortcut.col_type = CollisionType::NO_NEED; // no need for shortcut
            log("Shortcut is not needed", LogLevel::DEBUG);
            return;
        }
        // iterator over the comp_shortcuts map
        shortcutSteps = computeShortcutSteps(instance, shortcut);
        for (auto it = shortcut.comp_shortcuts.begin(); it != shortcut.comp_shortcuts.end(); it++) {
            int r_id = it->first;
            auto n_ri = it->second.ni.lock();
            auto n_rj = it->second.nj.lock();
            int steps = computeShortcutSteps(instance, it->second);
            shortcutSteps = std::max(shortcutSteps, steps);
        }
        if (shortcutSteps > shortcut.orig_span) {
            shortcut.col_type = CollisionType::NO_NEED; // no need for shortcut
            log("Shortcut is not needed", LogLevel::DEBUG);
            return;
        }
    }
    else {
        // check if there is a shortcut between ni and nj
        assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);
        if (ni->timeStep >= nj->timeStep - 1) {
            shortcut.col_type = CollisionType::NO_NEED; // no need for shortcut
            log("Shortcut is not needed", LogLevel::DEBUG);
            return;
        }
        shortcutSteps = computeShortcutSteps(instance, shortcut);
        if (shortcutSteps > (nj->timeStep - ni->timeStep)) {
            shortcut.col_type = CollisionType::NO_NEED; // no need for shortcut
            log("Shortcut is not needed", LogLevel::DEBUG);
            return;
        }
    }
    
    retimeShortcut(instance, shortcutSteps, shortcut);
    

    // if (config_.helpful_shortcut) {
    //     bool helpful = false;
    //     std::shared_ptr <Node> current = ni->Type1Next;
    //     while (!helpful && current != nj) {
    //         if (current->Type2Next.size() > 0) {
    //             helpful = true;
    //         }
    //         current = current->Type1Next;
    //     }
    //     current = nj;
    //     while (!helpful && current != nullptr) {
    //         if (current->Type2Prev.size() > 0) {
    //             for (auto edge : current->Type2Prev) {
    //                 if (edge->nodeFrom->timeStep >= current->timeStep) {
    //                     break;
    //                 }
    //             }
    //         }
    //         if (current->Type2Next.size() > 0) {
    //             helpful = true;
    //         }
    //         current = current->Type1Next;
    //     }
    //     if (!helpful && (current != nullptr)) {
    //         return false;
    //     }
    // }

    if (!shortcut.composite() && config_.tight_shortcut) {
        NodePtr current = ni;
        bool has_tight_type2_edge = false;
        bool all_tight_type1_edge = (earliest_t[ni->timeStep] == latest_t[ni->timeStep]);
        while (current != nj) {

            for (auto edge : current->Type2Prev) {
                if (edge->tight) {
                    has_tight_type2_edge = true;
                    break;
                }
            }
            for (auto edge : current->Type2Next) {
                if (edge->tight) {
                    has_tight_type2_edge = true;
                    break;
                }
            }
            current = current->Type1Next;
            if (earliest_t[current->timeStep] < latest_t[current->timeStep]) {
                all_tight_type1_edge = false;
            }
        }
        
        if (!has_tight_type2_edge && !all_tight_type1_edge) {
            shortcut.col_type = CollisionType::UNTIGHT;
            log("Shortcut is untight", LogLevel::DEBUG);
            return;
        }
    }

    shortcut.col_type = CollisionType::NONE;
    return;
}

void TPG::checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
     const std::vector<std::vector<NodePtr>> &timedNodes) const {
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();


    int shortcutSteps = shortcut.path.size() + 2; 
    if (shortcut.composite()) {
        for (int i = 1; i < shortcutSteps - 1; i++) {
            std::vector<RobotPose> comp_poses;
            comp_poses.push_back(shortcut.path[i - 1]);
            for (auto it = shortcut.comp_shortcuts.begin(); it != shortcut.comp_shortcuts.end(); it++) {
                comp_poses.push_back(it->second.path[i - 1]);
            }
            if (instance->checkCollision(comp_poses, false) == true) {
                shortcut.col_type = CollisionType::STATIC; // collide with the env and self
                return;
            }
        }
        shortcut.col_type = CollisionType::NONE;
        return;
    }
    else if (config_.allow_col) {
        // check if there is a shortcut between ni and nj
        assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);
        int t = ni->timeStep;
        while (timedNodes[ni->robotId][t]->timeStep < ni->timeStep) {
            t++;
        }
        for (int i = 1; i < shortcutSteps - 1; i++) {
            // check environment collision
            std::vector<RobotPose> comp_poses, comp_poses_prev;
            comp_poses.push_back(shortcut.path[i - 1]);
            comp_poses_prev.push_back(shortcut.path[i - 1]);
            t++;
            for (int j = 0; j < num_robots_; j++) {
                if (j != ni->robotId) {
                    comp_poses.push_back(timedNodes[j][t]->pose);
                    comp_poses_prev.push_back(timedNodes[j][t - 1]->pose);
                }
            }
            
            // bool col1 = false, col2 = false;
            // if (instance->checkCollision(comp_poses, false) == true || instance->checkCollision(comp_poses_prev, true) == true) {
            //     shortcut.col_type = CollisionType::STATIC; // collide with the env
            //     col1 = true;
            // }
            if (instance->checkCollision({comp_poses[0]}, false) == true) {
                shortcut.col_type = CollisionType::STATIC; // collide with the env
                return;
            }
            for (int j = 1; j < comp_poses.size(); j++) {
                if(instance->checkCollision({comp_poses[0], comp_poses[j]}, true) == true 
                        || instance->checkCollision({comp_poses_prev[0], comp_poses_prev[j]}, true) == true) {
                    shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                    return;
                }
            }
        }
        // need to check against all committed nodes of nj (parent of nj basically)
        std::vector<std::vector<bool>> visited;
        for (int i = 0; i < num_robots_; i++) {
            std::vector<bool> v(numNodes_[i], false);
            visited.push_back(v);
        }
        bfs(nj, visited, false, false);
        while (timedNodes[ni->robotId][t]->timeStep < nj->timeStep) {
            t++; 
            std::vector<RobotPose> comp_poses, comp_poses_end;
            std::vector<bool> checked_end(num_robots_, false);
            for (int j = 0; j < num_robots_; j++) {
                if (j != ni->robotId) {
                    if (visited[j][timedNodes[j][t]->timeStep] == true) {
                        comp_poses.push_back(timedNodes[j][t]->pose);
                        log("Checking against robot " + std::to_string(j) + " at time " + std::to_string(t)
                            + " step " + std::to_string(timedNodes[nj->robotId][t]->timeStep), LogLevel::DEBUG);
                    }
                    else if (checked_end[j] == false) {
                        comp_poses_end.push_back(timedNodes[j][t]->pose);
                        checked_end[j] = true;
                        log("Checking against robot " + std::to_string(j) + " at time " + std::to_string(t)
                            + " step " + std::to_string(timedNodes[nj->robotId][t]->timeStep), LogLevel::DEBUG);
                    }
                }
            }
            RobotPose pose_i = (shortcut.path.size() > 0) ? shortcut.path.back() : ni->pose;
            for (int j = 0; j < comp_poses.size(); j++) {
                if (instance->checkCollision({pose_i, comp_poses[j]}, true) == true) {
                    shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                    return;
                }
            }
            for (int j = 0; j < comp_poses_end.size(); j++) {
                if (instance->checkCollision({pose_i, comp_poses_end[j]}, true) == true) {
                    shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                    return;
                }
            }
        }
        
        shortcut.col_type = CollisionType::NONE;
        return;
    }
    else {
        // check if there is a shortcut between ni and nj
        assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);
        for (int i = 1; i < shortcutSteps - 1; i++) {
            // check environment collision
            RobotPose pose_i = shortcut.path[i - 1];
            if (instance->checkCollision({pose_i}, false) == true) {
                shortcut.col_type = CollisionType::STATIC; // collide with the env
                return;
            }
        }
    }

    auto tic = std::chrono::high_resolution_clock::now();

    // find dependent parent and child nodes
    std::vector<std::vector<bool>> visited;
    for (int i = 0; i < num_robots_; i++) {
        std::vector<bool> v(numNodes_[i], false);
        visited.push_back(v);
    }
    visited[nj->robotId][nj->timeStep] = true;
    bfs(nj, visited, true);

    visited[ni->robotId][ni->timeStep] = true;
    bfs(ni, visited, false);

    if (shortcutSteps <= 2) {
        // special case, check if the static node collides with any other independent nodes
        for (int j = 0; j < num_robots_; j++) {
            if (j == ni->robotId) {
                continue;
            }
            NodePtr node_j = start_nodes_[j];
            while (node_j != nullptr) {
                if (visited[j][node_j->timeStep] == false) {
                    if (instance->checkCollision({ni->pose, node_j->pose}, true) ||
                        instance->checkCollision({nj->pose, node_j->pose}, true)) {
                        shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                        shortcut.n_robot_col = node_j;
                        return;
                    }
                }
                node_j = node_j->Type1Next;
            }
        }
        shortcut.col_type = CollisionType::NONE;
        return;
    }

    // for (int j = 0; j < num_robots_; j++) {
    //     Eigen::MatrixXi col_matrix_j(shortcutSteps, numNodes_[j]);
    //     col_matrix_j.setZero();
    //     col_matrix.push_back(col_matrix_j);
    // }

    // check robot-robot collision
    for (int i = 1; i < shortcutSteps - 1; i++) {
        RobotPose pose_i = shortcut.path[i - 1];
        for (int j = 0; j < num_robots_; j++) {
            if (j == ni->robotId) {
                continue;
            }
            NodePtr node_j = start_nodes_[j];
            while (node_j != nullptr) {
                if (visited[j][node_j->timeStep] == false) {
                    bool collide = instance->checkCollision({pose_i, node_j->pose}, true);
                    if (collide) {
                        shortcut.n_robot_col = node_j;
                        shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                        return;
                    }
                }
                node_j = node_j->Type1Next;
            }
        }
    }

    shortcut.col_type = CollisionType::NONE;
    return;
}

std::string TPG::get_r1_inhand_obj_name()
{
    return r1_in_hand_obj_name_;
}

std::string TPG::get_r2_inhand_obj_name()
{
    return r2_in_hand_obj_name_;
}

Eigen::MatrixXd TPG::get_r1_inhand_goal_q()
{
    return r1_in_hand_goal_q_;
}

Eigen::MatrixXd TPG::get_r2_inhand_goal_q()
{
    return r2_in_hand_goal_q_;
}

void TPG::updateTPG(std::shared_ptr<PlanInstance> instance, const Shortcut &shortcut, const std::vector<Eigen::MatrixXi> &col_matrix) {
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();
    if (ni == nj || ni->Type1Next == nj) {
        return;
    }

    NodePtr n_prev = ni->Type1Next;
    // remove dangling type-2 edge skipped by the shortcut
    // Note that outgoing edges of the first node on the shortcut cannot be removed, but outgoing edges from the last node can be removed
    std::vector<std::shared_ptr<type2Edge>> edges_to_keep, edges_to_check;
    while (n_prev != nullptr && n_prev->timeStep < nj->timeStep) {
        for (auto edge : n_prev->Type2Next) {
            // if (n_prev->timeStep == ni->timeStep + 1) {
            //     edges_to_keep.push_back(edge);
            // }
            // else {
            //     edge->nodeTo->Type2Prev.erase(std::remove_if(edge->nodeTo->Type2Prev.begin(), edge->nodeTo->Type2Prev.end(),
            //         [edge](std::shared_ptr<type2Edge> e) { return e->edgeId == edge->edgeId; }), edge->nodeTo->Type2Prev.end());
            // }
            edges_to_keep.push_back(edge);
        }
        for (auto edge : n_prev->Type2Prev) {
            // edge->nodeFrom->Type2Next.erase(std::remove_if(edge->nodeFrom->Type2Next.begin(), edge->nodeFrom->Type2Next.end(),
            //     [edge](std::shared_ptr<type2Edge> e) { return e->edgeId == edge->edgeId; }), edge->nodeFrom->Type2Next.end());
            
            edges_to_check.push_back(edge);
            //edge->nodeTo = nj;
            //nj->Type2Prev.push_back(edge);
        }
        n_prev = n_prev->Type1Next;
    }
    if (config_.allow_col) {
        for (auto edge : nj->Type2Next) {
            edges_to_keep.push_back(edge);
        }
        nj->Type2Next.clear();
    }
    
    // sort edge based on the timeStep of the nodeFrom in descending order
    std::sort(edges_to_check.begin(), edges_to_check.end(), [](std::shared_ptr<type2Edge> a, std::shared_ptr<type2Edge> b) {
        return a->nodeFrom->timeStep > b->nodeFrom->timeStep;
    });
    // replace all incoming edge with the latest edge (one with largest incoming timestep) to nj
    bool first_edge = false;
    for (auto edge : edges_to_check) {
        if (!first_edge) {
            edge->nodeTo = nj;
            nj->Type2Prev.push_back(edge);
            first_edge = true;
        }
        else {
            edge->nodeFrom->Type2Next.erase(std::remove_if(edge->nodeFrom->Type2Next.begin(), edge->nodeFrom->Type2Next.end(),
                 [edge](std::shared_ptr<type2Edge> e) { return e->edgeId == edge->edgeId; }), edge->nodeFrom->Type2Next.end());
        }
    }

    // attach the shortcut to the current TPG
    n_prev = ni;
    for (int i = 0; i < shortcut.path.size(); i++) {
        NodePtr node = std::make_shared<Node>(ni->robotId, n_prev->timeStep + 1);
        node->pose = shortcut.path[i];
        n_prev->Type1Next = node;
        node->Type1Prev = n_prev;
        node->actId = nj->actId;
        
        n_prev = node;
    }
    n_prev->Type1Next = nj;
    nj->Type1Prev = n_prev;
    int nj_prevt = nj->timeStep;

    // sort the edges based on outgoing timestep
    std::sort(edges_to_keep.begin(), edges_to_keep.end(), [](std::shared_ptr<type2Edge> a, std::shared_ptr<type2Edge> b) {
        return a->nodeTo->timeStep < b->nodeTo->timeStep;
    });

    // replace all outgoing edge with the earliest edge (one with earliest incoming timestep) from ni
    first_edge = false;
    for (auto edge : edges_to_keep) {
        if (!first_edge) {
            edge->nodeFrom = ni->Type1Next;
            ni->Type1Next->Type2Next.push_back(edge);
            first_edge = true;
            log("Readding edge from " + std::to_string(edge->nodeFrom->timeStep) + " to " + std::to_string(edge->nodeTo->timeStep), LogLevel::DEBUG);
        }
        else {
            edge->nodeTo->Type2Prev.erase(std::remove_if(edge->nodeTo->Type2Prev.begin(), edge->nodeTo->Type2Prev.end(),
                 [edge](std::shared_ptr<type2Edge> e) { return e->edgeId == edge->edgeId; }), edge->nodeTo->Type2Prev.end());
            log("removing edge to " + std::to_string(edge->nodeTo->timeStep), LogLevel::DEBUG);
        }
    }

    // update timestamp for the rest of the nodes
    while (n_prev->Type1Next != nullptr) {
        n_prev->Type1Next->timeStep = n_prev->timeStep + 1;
        n_prev = n_prev->Type1Next;
    }
    end_nodes_[ni->robotId] = n_prev;
    int numNodes_prev = numNodes_[ni->robotId];
    numNodes_[ni->robotId] = n_prev->timeStep + 1;
    int reducedSteps = numNodes_prev - numNodes_[ni->robotId];

    // update the collision matrix
    // for (int j = 0; j < num_robots_; j++) {
    //     if (ni->robotId == j) {
    //         continue;
    //     }
    //     Eigen::MatrixXi col_matrix_ij;
    //     getCollisionCheckMatrix(ni->robotId, j, col_matrix_ij);

    //     Eigen::MatrixXi new_col_matrix_ij(numNodes_[ni->robotId], numNodes_[j]);
    //     new_col_matrix_ij.block(0, 0, ni->timeStep + 1, numNodes_[j]) = col_matrix_ij.block(0, 0, ni->timeStep + 1, numNodes_[j]);
    //     if (col_matrix.size() > j && col_matrix[j].rows() > 0) {
    //         new_col_matrix_ij.block(ni->timeStep + 1, 0, nj->timeStep - 1 - ni->timeStep, numNodes_[j]) = col_matrix[j].block(1, 0, nj->timeStep - 1 - ni->timeStep, numNodes_[j]);
    //     } 
    //     new_col_matrix_ij.block(nj->timeStep, 0, numNodes_[ni->robotId] - nj->timeStep, numNodes_[j]) = col_matrix_ij.block(nj_prevt, 0, numNodes_prev - nj_prevt, numNodes_[j]);

    //     updateCollisionCheckMatrix(ni->robotId, j, new_col_matrix_ij);

    //         if ((nj->timeStep +  config_.ignore_steps) < numNodes_[j]) {
    //             while (node_ignored != nullptr && steps <= (nj->timeStep + config_.ignore_steps)) {
    //                 node_ignored = node_ignored->Type1Next;
    //                 steps++;
    //             }
    //             if (node_ignored != nullptr) {
    //                 type2Edge edge_f;
    //                 edge_f.edgeId = idType2Edges_;
    //                 edge_f.nodeFrom = nj;
    //                 edge_f.nodeTo = node_ignored;
    //                 nj->Type2Next.push_back(edge_f);
    //                 node_ignored->Type2Prev.push_back(edge_f);
    //                 idType2Edges_++;
    //             }
    //         }
    //     }
    // }
    if (hasCycle()) {
        if (config_.debug_graph) {
            saveToDotFile("cycle_after_shortcut.dot");
        }
        log("Cycle detected after adding shortcut", LogLevel::ERROR);
    }
    
}

bool TPG::repairTPG(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
            const std::vector<std::vector<int>> &earliest_t) {
    // 1. remove nodes that are the same in the shortcut (don't need to check collision)
    for (int i = 0; i < num_robots_; i++) {
        NodePtr ni, nj;
        if (i == shortcut.robot_id()) {
            ni = shortcut.ni.lock();
            nj = shortcut.nj.lock();
        }
        else if (shortcut.composite()) {
            ni = shortcut.comp_shortcuts[i].ni.lock();
            nj = shortcut.comp_shortcuts[i].nj.lock();
        }
        else {
            continue;
        }
        NodePtr node = ni;
        while (node != nullptr && node->timeStep < nj->timeStep) {
            if (instance->computeDistance(node->pose, node->Type1Next->pose) <= 0) {
                if (node->Type1Next->Type1Next != nullptr) {
                
                    node->Type1Next = node->Type1Next->Type1Next;
                    node->Type1Next->Type1Prev = node;
                }
            }
            node = node->Type1Next;
        }
    }

    // 2. check collision of every shortcut segment with independent 
    for (int i = 0; i < num_robots_; i++) {
        // check collision from robot j, to robot i
        // two parts, nodes before robot j's shortcut to nodes on robot i's shortcut
        // nodes on robot j's shortcut to nodes after robot i's shortcut
        NodePtr n_ii, n_ij;
        if (i == shortcut.robot_id()) {
            n_ii = shortcut.ni.lock();
            n_ij = shortcut.nj.lock();
        }
        else if (shortcut.composite()) {
            n_ii = shortcut.comp_shortcuts[i].ni.lock();
            n_ij = shortcut.comp_shortcuts[i].nj.lock();
        }
        else {
            // skip this i
            continue;
        }
        
        // run bfs from node_i backward
        std::vector<std::vector<bool>> visited;
        for (int ri = 0; ri < num_robots_; ri++) {
            std::vector<bool> v(numNodes_[ri], false);
            visited.push_back(v);
        }
        bfs(n_ii, visited, false);
        bfs(n_ij, visited, true);
    
        for (int j = 0; j < num_robots_; j++) {
            if (i == j) {
                continue;
            }
            // check collision from robot j to the shortcut
            log("Checking collision from robot " + std::to_string(j) + " to robot " + std::to_string(i) +
                " from t=" + std::to_string(n_ii->timeStep) + " to " + std::to_string(n_ij->timeStep), LogLevel::DEBUG);
            
            NodePtr node_i = n_ii->Type1Next;
            NodePtr iter_node_j_start = start_nodes_[j];
            NodePtr node_j = iter_node_j_start;

            while (node_i != nullptr && earliest_t[i][node_i->timeStep] < earliest_t[i][n_ij->timeStep]) {
                bool in_collision = false;
                node_j = iter_node_j_start;
                while (node_j != nullptr && earliest_t[j][node_j->timeStep] < earliest_t[i][node_i->timeStep]) {
                    if (visited[j][node_j->timeStep]) {
                        node_j = node_j->Type1Next;
                        continue;
                    }
                    bool has_collision = instance->checkCollision({node_i->pose, node_j->pose}, true);
                    if (has_collision) {
                        in_collision = true; 
                    }
                    else if (in_collision) {
                        in_collision = false;
                        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                        edge->edgeId = idType2Edges_++;
                        edge->nodeFrom = node_j;
                        edge->nodeTo = node_i;
                        node_j->Type2Next.push_back(edge);
                        node_i->Type2Prev.push_back(edge);
                        iter_node_j_start = node_j;
                        log("Added type 2 dependency from Robot " + std::to_string(node_j->robotId) + " at time "
                            + std::to_string(node_j->timeStep) + " " + std::to_string(earliest_t[j][node_j->timeStep]) 
                            + " -> Robot " + std::to_string(node_i->robotId) + " at time " 
                            + std::to_string(node_i->timeStep) + " " + std::to_string(earliest_t[i][node_i->timeStep]), LogLevel::DEBUG);
                    }
                    node_j = node_j->Type1Next;
                }
                if (in_collision) {
                    if (node_j == nullptr) {
                        log("Target conflict: robot " + std::to_string(j) + " at time " + std::to_string(numNodes_[j])
                            + " -> robot " + std::to_string(i) + " at time " + std::to_string(node_i->timeStep), LogLevel::ERROR);
                    }
                    std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                    edge->edgeId = idType2Edges_++;
                    edge->nodeFrom = node_j;
                    edge->nodeTo = node_i;
                    node_j->Type2Next.push_back(edge);
                    node_i->Type2Prev.push_back(edge);
                    iter_node_j_start = node_j;
                    log("Added type 2 dependency (end) from Robot " + std::to_string(node_j->robotId) + " at time "
                        + std::to_string(node_j->timeStep) + " " + std::to_string(earliest_t[j][node_j->timeStep])
                        + " -> Robot " + std::to_string(node_i->robotId) + " at time " 
                        + std::to_string(node_i->timeStep) + " " + std::to_string(earliest_t[i][node_i->timeStep]), LogLevel::DEBUG);
                }
                node_i = node_i->Type1Next;
            }

            // then check collision from the shortcut to robot j
            log("Checking collision from robot " + std::to_string(i) + " from t=" + std::to_string(n_ii->timeStep) + " to " + std::to_string(n_ij->timeStep)
                + " to robot " + std::to_string(j), LogLevel::DEBUG);
            node_j = start_nodes_[j];
            NodePtr iter_node_i_start = n_ii->Type1Next;
            while (node_j != nullptr && earliest_t[j][node_j->timeStep] < earliest_t[i][n_ii->timeStep]) {
                node_j = node_j->Type1Next;
            }
            while (node_j != nullptr) {
                if (visited[j][node_j->timeStep]) {
                    break;
                }
                bool in_collision = false;
                node_i = iter_node_i_start;
                while (node_i != nullptr && earliest_t[i][node_i->timeStep] < earliest_t[i][n_ij->timeStep] &&
                        earliest_t[i][node_i->timeStep] < earliest_t[j][node_j->timeStep]) {
                    
                    bool has_collision = instance->checkCollision({node_i->pose, node_j->pose}, true);
                    if (has_collision) {
                        in_collision = true; 
                    }
                    else if (in_collision) {
                        in_collision = false;
                        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                        edge->edgeId = idType2Edges_++;
                        edge->nodeFrom = node_i;
                        edge->nodeTo = node_j;
                        node_i->Type2Next.push_back(edge);
                        node_j->Type2Prev.push_back(edge);
                        iter_node_i_start = node_i;
                        log("Added type 2 dependency from Robot " + std::to_string(node_i->robotId) + " at time "
                            + std::to_string(node_i->timeStep) + " " + std::to_string(earliest_t[i][node_i->timeStep]) + 
                            " -> Robot " + std::to_string(node_j->robotId) + " at time " 
                            + std::to_string(node_j->timeStep) + " " + std::to_string(earliest_t[j][node_j->timeStep]) , LogLevel::DEBUG);
                    }
                    node_i = node_i->Type1Next;
                }
                if (in_collision) {
                    std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                    edge->edgeId = idType2Edges_++;
                    edge->nodeFrom = node_i;
                    edge->nodeTo = node_j;
                    node_i->Type2Next.push_back(edge);
                    node_j->Type2Prev.push_back(edge);
                    iter_node_i_start = node_i;
                    log("Added type 2 dependency (end) from Robot " + std::to_string(node_i->robotId) + " at time "
                        + std::to_string(node_i->timeStep) + " " + std::to_string(earliest_t[i][node_i->timeStep])
                        + " -> Robot " + std::to_string(node_j->robotId) + " at time " 
                        + std::to_string(node_j->timeStep) + " " + std::to_string(earliest_t[j][node_j->timeStep]), LogLevel::DEBUG);
                }
                node_j = node_j->Type1Next;
            }
        }
    }
    if (hasCycle()) {
        if (config_.debug_graph) {
            saveToDotFile("cycle_after_repair.dot");
        }
        log("Cycle detected after repairing the TPG", LogLevel::ERROR);
        return false;
    }
    return true;
}

void TPG::switchShortcuts() {
    for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < num_robots_; j++) {
            if (i == j) {
                continue;
            }

            Eigen::MatrixXi col_matrix_ij;
            getCollisionCheckMatrix(i, j, col_matrix_ij);
            // print the collision matrix

            // update the type 2 edges
            NodePtr node_i = start_nodes_[i];
            while (node_i->Type1Next != nullptr) {
                // check all other robots, remove incoming type-2 edges that appear after current timestamp, and add new outgoing type-2 edges 
                for (int k = node_i->Type2Prev.size() - 1; k >= 0; k--) {
                    auto edge = node_i->Type2Prev[k];
                    if (edge->switchable == false) {
                        continue;
                    }
                    auto nodeFrom = edge->nodeFrom;
                    auto edgeId = edge->edgeId;
                    int minWaitTime = nodeFrom->timeStep - node_i->timeStep;
                    if (minWaitTime > 0) {
                        // will switch the type2 dependency from node_i->next to node_j_free
                        NodePtr node_j_free = nodeFrom; 
                        while (node_j_free->Type1Prev != nullptr) {
                            node_j_free = node_j_free->Type1Prev;
                            if (col_matrix_ij(node_i->timeStep, node_j_free->timeStep) == 0) {
                                int minSwitchedWaitTime = node_i->timeStep + 1 - node_j_free->timeStep;
                                if (minSwitchedWaitTime < minWaitTime) {
                                    std::shared_ptr<type2Edge> newEdge = std::make_shared<type2Edge>();
                                    newEdge->edgeId = idType2Edges_;
                                    newEdge->nodeFrom = node_i->Type1Next;
                                    newEdge->nodeTo = node_j_free;
                                    node_i->Type1Next->Type2Next.push_back(newEdge);
                                    node_j_free->Type2Prev.push_back(newEdge);
                                    idType2Edges_++;
                                
                                    node_i->Type2Prev.erase(node_i->Type2Prev.begin() + k);
                                    nodeFrom->Type2Next.erase(std::remove_if(nodeFrom->Type2Next.begin(), nodeFrom->Type2Next.end(), 
                                        [edgeId](std::shared_ptr<type2Edge> e) { return e->edgeId == edgeId; }), nodeFrom->Type2Next.end());
                                    
                                    // TODO: fix when type2 edges are inconsistent

                                    log("Removed type 2 dependency from Robot " + std::to_string(nodeFrom->robotId) + " at time "
                                        + std::to_string(nodeFrom->timeStep) + " -> Robot " + std::to_string(node_i->robotId) + " at time " 
                                        + std::to_string(node_i->timeStep), LogLevel::DEBUG);
                                    log("Added type 2 dependency from Robot " + std::to_string(node_i->robotId) + " at time "
                                        + std::to_string(node_i->Type1Next->timeStep) + " -> Robot " + std::to_string(node_j_free->robotId) + " at time " 
                                        + std::to_string(node_j_free->timeStep), LogLevel::DEBUG);
                                    break;
                                }
                            }
                        }
                    }
                }

                node_i = node_i->Type1Next;
            }
        }

        // assert (hasCycle() == false);
    }

}


bool TPG::hasCycle() const 
{
    // Initialized visited matrix
    std::vector<std::vector<bool>> visited;
    std::vector<std::vector<bool>> inStack;

    std::stack<NodePtr> stack;

    for (int i = 0; i < num_robots_; i++) {
        std::vector<bool> v(numNodes_[i], false);
        std::vector<bool> s(numNodes_[i], false);
        visited.push_back(v);
        inStack.push_back(s);
    }

    for (int i = 0; i < num_robots_; i++) {
        NodePtr node_i = start_nodes_[i];
        while (node_i != nullptr) {
            if (!visited[i][node_i->timeStep]) {
                stack.push(node_i);
                // a depth first search to check if there is a cycle
                // by checking if an element is in the stack
                while (!stack.empty()) {
                    NodePtr node_v = stack.top();
                    if (!visited[node_v->robotId][node_v->timeStep]) {
                        visited[node_v->robotId][node_v->timeStep] = true;
                        inStack[node_v->robotId][node_v->timeStep] = true;
                    }
                    else {
                        inStack[node_v->robotId][node_v->timeStep] = false;
                        stack.pop();
                        continue;
                    }

                    if (node_v->Type1Next != nullptr) {
                        NodePtr node_next = node_v->Type1Next;
                        if (!visited[node_next->robotId][node_next->timeStep]) {
                            stack.push(node_next);
                        } else if (inStack[node_next->robotId][node_next->timeStep]) {
                            log("Cycle detected at robot " + std::to_string(node_next->robotId) + " at time " + std::to_string(node_next->timeStep), LogLevel::INFO);
                            return true;
                        }
                    }
                    for (auto edge : node_v->Type2Next) {
                        if (!visited[edge->nodeTo->robotId][edge->nodeTo->timeStep]) {
                            stack.push(edge->nodeTo);
                        } else if (inStack[edge->nodeTo->robotId][edge->nodeTo->timeStep]) {
                            log("Cycle detected at robot " + std::to_string(edge->nodeTo->robotId) + " at time " + std::to_string(edge->nodeTo->timeStep), LogLevel::INFO);
                            return true;
                        }
                    }
                }
            }
            node_i = node_i->Type1Next;
        }
    }
    return false;
}

bool TPG::dfs(NodePtr ni, NodePtr nj, std::vector<std::vector<bool>> &visited) const
{
    // DFS function to check if there is a path from ni to nj
    visited[ni->robotId][ni->timeStep] = true;
    if (ni->Type1Next != nullptr) {
        NodePtr ni_next = ni->Type1Next;
        
        // search the next type1 node
        if (ni_next == nj) {
            return true;
        }
        if (!visited[ni_next->robotId][ni_next->timeStep]) {
            if (dfs(ni_next, nj, visited)) {
                return true;
            }
        }
    }

    // search the type2 node neighbors
    for (auto edge : ni->Type2Next) {
        if (edge->nodeTo == nj) {
            return true;
        }
        if (!visited[edge->nodeTo->robotId][edge->nodeTo->timeStep]) {
            if (dfs(edge->nodeTo, nj, visited)) {
                return true;
            }
        }
    }
    return false;
}

void TPG::findFlowtimeMakespan(double &flowtime, double &makespan)
{
    // trajectory_msgs::JointTrajectory joint_traj;
    // size_t num_joints = 0;
    // for (int i = 0; i < num_robots_; i++ ) {
    //     num_joints += instance->getRobotDOF(i);
    // }
    // joint_traj.joint_names.resize(num_joints);
    // setSyncJointTrajectory(joint_traj, flowtime, makespan);

    std::vector<std::vector<int>> earliest_t, latest_t;
    std::vector<int> reached_end;
    findEarliestReachTime(earliest_t, reached_end);
    int flowspan_i = 0, makespan_i = 0;
    for (int i = 0; i < num_robots_; i++) {
        flowspan_i += reached_end[i];
        makespan_i = std::max(makespan_i, reached_end[i]);
    }

    flowtime = flowspan_i * dt_;
    makespan = makespan_i * dt_;
}

void TPG::updateEarliestReachTime(std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end,
        Shortcut &shortcut) {
    reached_end.resize(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        reached_end[i] = reached_t[i].back();
    }
    if (shortcut.composite()) {
        findEarliestReachTime(reached_t, reached_end);
    }
    else if (config_.allow_col) {
        NodePtr node_i = shortcut.ni.lock()->Type1Next;
        int robot_id = node_i->robotId;
        while (node_i != nullptr) {
            reached_t[robot_id][node_i->timeStep] = reached_t[robot_id][node_i->timeStep - 1] + 1;
            for (auto edge: node_i->Type2Prev) {
                reached_t[robot_id][node_i->timeStep] = std::max(
                    reached_t[robot_id][node_i->timeStep], reached_t[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] + 1
                );
            }
            if (node_i->Type1Next == nullptr) {
                reached_end[robot_id] = reached_t[robot_id][node_i->timeStep];
                reached_t[robot_id].resize(node_i->timeStep + 1);
            }
            node_i = node_i->Type1Next;
        }
    }
}

void TPG::findEarliestReachTime(std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end) const
{
    reached_t.clear();
    reached_end.clear();
    std::vector<NodePtr> nodes;
    for (int i = 0; i < num_robots_; i++) {
        NodePtr node_i = start_nodes_[i];
        nodes.push_back(node_i);

        std::vector<int> v(numNodes_[i], -1);
        reached_t.push_back(v);
        reached_end.push_back(-1);
    }

    int flowtime_i = 0;
    bool allReached = false;
    int j = 0;
    while(!allReached) {

        for (int i = 0; i < num_robots_; i++) {
            if (reached_t[i][nodes[i]->timeStep] == -1) {
                reached_t[i][nodes[i]->timeStep] = j;
            }
        }
        for (int i = 0; i < num_robots_; i++) {
            if (nodes[i]->Type1Next != nullptr) {
                bool safe = true;
                for (auto edge : nodes[i]->Type1Next->Type2Prev) {
                    if (reached_t[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] == -1) {
                        safe = false;
                        break;
                    }
                }
                if (safe) {
                    nodes[i] = nodes[i]->Type1Next;
                }
            }
            else if (reached_end[i] == -1) {
                reached_end[i] = j;
                flowtime_i += j;
            }
        }

        allReached = true;
        for (int i = 0; i < num_robots_; i++) {
            allReached &= (reached_end[i] != -1);
        }
        j++;
    }

    // print all the reached times
    // std::cout << "Earliest reach time:\n";
    // for (int i = 0; i < num_robots_; i++) {
    //     for (int j = 0; j < numNodes_[i]; j++) {
    //         std::cout << reached_t[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

}

void TPG::findTimedNodes(const std::vector<std::vector<int>> &earliest_t, std::vector<std::vector<NodePtr>> & timed_nodes)
{
    timed_nodes.resize(num_robots_);

    int makespan = 0;
    for (int i = 0; i < num_robots_; i++) {
        makespan = std::max(makespan, earliest_t[i].back());
    }

    for (int i = 0; i < num_robots_; i++) {
        std::vector<NodePtr> timed_nodes_i;
        timed_nodes_i.push_back(start_nodes_[i]);
        int last_t = 1;
        NodePtr node = start_nodes_[i]->Type1Next;
        while (node != nullptr) {
            int t = earliest_t[i][node->timeStep];
            while (last_t < t) {
                timed_nodes_i.push_back(timed_nodes_i.back());
                last_t++;
            }
            timed_nodes_i.push_back(node);
            last_t++;
            node = node->Type1Next;
        }

        while (last_t <= makespan) {
            // the robot stays at its last node until the end of makespan
            timed_nodes_i.push_back(end_nodes_[i]);
            last_t++;
        }
        timed_nodes[i] = timed_nodes_i;
    }
}

void TPG::findLatestReachTime(std::vector<std::vector<int>> &reached_t, const std::vector<int> &reached_end)
{
    reached_t.clear();
    std::vector<NodePtr> nodes;
    int j = 0;
    for (int i = 0; i < num_robots_; i++) { 
        NodePtr node_i = end_nodes_[i];
        nodes.push_back(node_i);

        std::vector<int> v(numNodes_[i], -1);
        v.back() = reached_end[i];
        reached_t.push_back(v);
        j = std::max(j, reached_end[i]);
    }
    if (config_.tight_shortcut_makespan) {
        for (int i = 0; i < num_robots_; i++) {
            reached_t[i].back() = j;
        }
    }

    bool allReached = false;

    while (!allReached) {
        for (int i = 0; i < num_robots_; i++) {

            if (reached_t[i][nodes[i]->timeStep] == -1) {
                reached_t[i][nodes[i]->timeStep] = j;
            }
        }

        for (int i = 0; i < num_robots_; i++) {
            if ((reached_t[i][nodes[i]->timeStep] >= j) && (nodes[i]->Type1Prev != nullptr)) {
                bool safe = true;
                for (auto edge : nodes[i]->Type1Prev->Type2Next) {
                    if (reached_t[edge->nodeTo->robotId][edge->nodeTo->timeStep] == -1) {
                        safe = false;
                        break;
                    }
                }
                if (safe) {
                    nodes[i] = nodes[i]->Type1Prev;
                }
            }
        }
        allReached = true;
        for (int i = 0; i < num_robots_; i++) {
            allReached &= (reached_t[i][0] != -1);
        }
        j--;
    }
    
    // print all the reached times
    // std::cout << "Latest reach time:\n";
    // for (int i = 0; i < num_robots_; i++) {
    //     for (int j = 0; j < numNodes_[i]; j++) {
    //         std::cout << reached_t[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
}

void TPG::findTightType2Edges(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<int>> &latest_t)
{
    for (int i = 0; i < num_robots_; i++) {
        NodePtr node = start_nodes_[i];

        while (node->Type1Next != nullptr) {
            int dt = earliest_t[i][node->Type1Next->timeStep] - earliest_t[i][node->timeStep];
    
            for (auto &edge : node->Type1Next->Type2Prev) {
                edge->tight = false;
            }
    
            if (dt > 1) {
                for (auto &edge : node->Type1Next->Type2Prev) {
                    int t_edge_start = earliest_t[edge->nodeFrom->robotId][edge->nodeFrom->timeStep];
                    int dt_edge = earliest_t[i][node->Type1Next->timeStep] - t_edge_start;
                    if (dt_edge == 1) {
                        edge->tight = true;
                    }
                    else {
                        edge->tight = false;
                    }
                }
            }
            node = node->Type1Next;
        }
    }
}

bool TPG::bfs(NodePtr ni, std::vector<std::vector<bool>> &visited, bool forward, bool bwd_shiftone) const
{
    // BFS function to find all the dependent nodes of ni
    std::queue<NodePtr> q;
    q.push(ni);
    visited[ni->robotId][ni->timeStep] = true;
    
    while (!q.empty()) {
        NodePtr node = q.front();
        q.pop();
        if (forward) {
            if (node->Type1Next != nullptr && !visited[node->Type1Next->robotId][node->Type1Next->timeStep]) {
                q.push(node->Type1Next);
                visited[node->Type1Next->robotId][node->Type1Next->timeStep] = true;
            }
            for (auto edge : node->Type2Next) {
                if (!visited[edge->nodeTo->robotId][edge->nodeTo->timeStep]) {
                    edge->switchable = false;
                    q.push(edge->nodeTo);
                    visited[edge->nodeTo->robotId][edge->nodeTo->timeStep] = true;
                }
            }
        } else {
            if (node->Type1Prev != nullptr && !visited[node->Type1Prev->robotId][node->Type1Prev->timeStep]) {
                q.push(node->Type1Prev);
                visited[node->Type1Prev->robotId][node->Type1Prev->timeStep] = true;
            }
            for (auto edge : node->Type2Prev) {
                int t_prev = (bwd_shiftone) ? edge->nodeFrom->timeStep - 1 : edge->nodeFrom->timeStep;
                if (t_prev >= 0 && !visited[edge->nodeFrom->robotId][t_prev]) {
                    edge->switchable = false;
                    if (bwd_shiftone) {
                        q.push(edge->nodeFrom->Type1Prev);
                    }
                    else {
                        q.push(edge->nodeFrom);
                    }
                    visited[edge->nodeFrom->robotId][t_prev] = true;
                }
            }
        }
    }
    return true;
}


void TPG::transitiveReduction() {
    for (int i = 0; i < num_robots_; i++) {
        NodePtr node_i = start_nodes_[i];
        while (node_i != nullptr) {
            for (int k = node_i->Type2Next.size() - 1; k >= 0; k--) {
                
                std::shared_ptr<type2Edge> edge = node_i->Type2Next[k];
                NodePtr n_from = node_i;
                NodePtr n_to = edge->nodeTo;
                
                // Remove this edge temporarily
                int idToRemove = edge->edgeId;
                n_from->Type2Next.erase(std::remove_if(n_from->Type2Next.begin(), n_from->Type2Next.end(), 
                    [idToRemove](std::shared_ptr<type2Edge> element) {return element->edgeId == idToRemove;}), n_from->Type2Next.end());
                n_to->Type2Prev.erase(std::remove_if(n_to->Type2Prev.begin(), n_to->Type2Prev.end(), 
                    [idToRemove](std::shared_ptr<type2Edge> element) {return element->edgeId == idToRemove;}), n_to->Type2Prev.end());

                // Initialized visited matrix
                std::vector<std::vector<bool>> visited;
                for (int i = 0; i < num_robots_; i++) {
                    std::vector<bool> v(numNodes_[i], false);
                    visited.push_back(v);
                }

                // If v is still reachable from u, then the edge (u, v) is transitive
                if (dfs(n_from, n_to, visited)) {
                    // Edge is transitive, remove it permanently
                } else {
                    // Edge is not transitive, add it back
                    n_to->Type2Prev.push_back(edge);
                    n_from->Type2Next.insert(n_from->Type2Next.begin() + k, edge);
                }
            }

            node_i = node_i->Type1Next;
        }
        
    }

}

void TPG::getCollisionCheckMatrix(int robot_i, int robot_j, Eigen::MatrixXi &col_matrix) const {
    if (robot_i > robot_j) {
        col_matrix = collisionCheckMatrix_[robot_j][robot_i - robot_j - 1].transpose();
    }
    else {
        col_matrix = collisionCheckMatrix_[robot_i][robot_j - robot_i - 1];
    }
}

void TPG::updateCollisionCheckMatrix(int robot_i, int robot_j, const Eigen::MatrixXi &col_matrix) {
    if (robot_i > robot_j) {
        collisionCheckMatrix_[robot_j][robot_i - robot_j - 1] = col_matrix.transpose();
    }
    else {
        collisionCheckMatrix_[robot_i][robot_j - robot_i - 1] = col_matrix;
    }
}

bool TPG::saveToDotFile(const std::string& filename) const {
    std::ofstream out(filename);
    out << "digraph G {" << std::endl;

    // define node attributes here
    out << "node [shape=circle];" << std::endl;
    out << "rankdir=LR;" << std::endl;

    // define all the nodes
    for (int i = 0; i < num_robots_; i++) {
        out << "subgraph cluster_" << i << " {" << std::endl;
        out << "label = \"Robot " << i << "\";" << std::endl;
        out << "rank=same;" << std::endl;
        NodePtr node_i = start_nodes_[i];
        while (node_i != nullptr) {
            out << "n" << i << "_" << node_i->timeStep << " [label=\"" << i << "_" << node_i->timeStep << "\"];" << std::endl;
            node_i = node_i->Type1Next;
        }
        node_i = start_nodes_[i];
        out << "n" << i << "_" << node_i->timeStep;
        while (node_i != nullptr) {
            if (node_i->Type1Next != nullptr) {
                out << " -> " << "n" << i << "_" << node_i->Type1Next->timeStep;
            }
            node_i = node_i->Type1Next;
        }
        out << ";" << std::endl;
        out << "}" << std::endl;
    }

    // define all the edges
    for (int i = 0; i < num_robots_; i++) {
        NodePtr node_i = start_nodes_[i];
        while (node_i != nullptr) {
            for (auto edge : node_i->Type2Prev) {
                out << "n" << edge->nodeFrom->robotId << "_" << edge->nodeFrom->timeStep << " -> " << "n" << i << "_" << node_i->timeStep << ";" << std::endl;
            }
            node_i = node_i->Type1Next;
        }
    }

    out << "}" << std::endl;
    out.close();

    std::string command = "dot -Tpng " + filename + " -o " + filename + ".png";
    int result = system(command.c_str());

    return result == 0;
}

bool TPG::moveit_execute(std::shared_ptr<MoveitInstance> instance, 
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) {
    // convert solution to moveit plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    trajectory_msgs::JointTrajectory &joint_traj = my_plan.trajectory_.joint_trajectory;
    joint_traj.joint_names = move_group->getActiveJoints();
    double flowtime, makespan;
    setSyncJointTrajectory(joint_traj, flowtime, makespan);

    executed_steps_.clear();
    for (int i = 0; i < num_robots_; i++) {
        NodePtr start_node = getExecStartNode(i);
        int start_t = start_node->timeStep;
        executed_steps_.push_back(std::make_unique<std::atomic<int>>(start_t));
    }

    // execute the plan
    move_group->execute(my_plan);

    return true;
}

bool TPG::actionlib_execute(const std::vector<std::string> &joint_names, TrajectoryClient &client) {
    moveit_msgs::ExecuteTrajectoryGoal goal;
    
    goal.trajectory.joint_trajectory.joint_names = joint_names;
    double flowtime, makespan;
    setSyncJointTrajectory(goal.trajectory.joint_trajectory, flowtime, makespan);

    auto doneCb = [](const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::ExecuteTrajectoryResultConstPtr& result) {
        log("Trajectory execution action finished: " + state.toString(), LogLevel::INFO);
    };

    client.sendGoal(goal, doneCb);

    bool finished = client.waitForResult(ros::Duration(30.0));
    if (finished) {
        actionlib::SimpleClientGoalState state = client.getState();
        return true;
    } else {
        log("Action did not finish before the time out.", LogLevel::ERROR);
        return false;
    }
}

MRTrajectory TPG::getSyncJointTrajectory(std::shared_ptr<PlanInstance> instance) const {
    MRTrajectory traj;
    double flowtime, makespan;
    moveit_msgs::RobotTrajectory joint_traj;
    int total_dof = 0;
    for (int i = 0; i < num_robots_; i++) {
        total_dof += start_nodes_[i]->pose.joint_values.size();
    }
    joint_traj.joint_trajectory.joint_names.resize(total_dof);
    setSyncJointTrajectory(joint_traj.joint_trajectory, flowtime, makespan);

    convertSolution(instance, joint_traj, traj, false);
    return traj;
}

void TPG::setSyncJointTrajectory(trajectory_msgs::JointTrajectory &joint_traj, double &flowtime, double &makespan) const {
    std::vector<NodePtr> nodes;
    int total_dof = 0, total_dof_w_hand = 0;
    for (int i = 0; i < num_robots_; i++) {
        NodePtr node_i = getExecStartNode(i);
        nodes.push_back(node_i);
        total_dof += start_nodes_[i]->pose.joint_values.size();
        total_dof_w_hand += start_nodes_[i]->pose.hand_values.size();
    }
    total_dof_w_hand += total_dof;
    
    bool set_hand = false;
    if (total_dof == joint_traj.joint_names.size()) {
        set_hand = false;
    }
    else if(total_dof_w_hand == joint_traj.joint_names.size()) {
        set_hand = true;
    }
    else {
        log("Mismatch in the number of joints in the trajectory " + std::to_string(joint_traj.joint_names.size()) + 
            " and the number of joints " + std::to_string(total_dof) + " in the TPG.", LogLevel::ERROR);
        return;
    }

    joint_traj.points.clear();

    std::vector<std::vector<bool>> reached;
    std::vector<bool> reached_end;
    for (int i = 0; i < num_robots_; i++) {
        std::vector<bool> v(numNodes_[i], false);
        reached.push_back(v);
        reached_end.push_back(false);
    }

    int flowtime_i = 0;
    bool allReached = false;
    int j = 0;
    while(!allReached) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(joint_traj.joint_names.size());
        point.velocities.resize(joint_traj.joint_names.size());
        point.accelerations.resize(joint_traj.joint_names.size());
        point.time_from_start = ros::Duration(j * dt_);
        joint_traj.points.push_back(point);

        int dof_s = 0;
        for (int i = 0; i < num_robots_; i++) {
            RobotPose pose_j_t = nodes[i]->pose;
            for (int d = 0; d < pose_j_t.joint_values.size(); d++) {
                joint_traj.points[j].positions[dof_s + d] = pose_j_t.joint_values[d];
            }
            dof_s += pose_j_t.joint_values.size();
            if (set_hand) {
                for (int d = 0; d < pose_j_t.hand_values.size(); d++) {
                    joint_traj.points[j].positions[dof_s + d] = pose_j_t.hand_values[d];
                }
                dof_s += pose_j_t.hand_values.size();
            }
            reached[i][nodes[i]->timeStep] = true;
        }
        for (int i = 0; i < num_robots_; i++) {

            if (nodes[i]->Type1Next != nullptr) {
                bool safe = true;
                for (auto edge : nodes[i]->Type1Next->Type2Prev) {
                    if (reached[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] == false) {
                        safe = false;
                        break;
                    }
                }
                if (safe) {
                    nodes[i] = nodes[i]->Type1Next;
                }
            }
            else if (!reached_end[i]) {
                reached_end[i] = true;
                flowtime_i += j;
            }
        }

        allReached = true;
        for (int i = 0; i < num_robots_; i++) {
            allReached &= reached_end[i];
        }
        j++;
    }

    flowtime = flowtime_i * dt_;
    makespan = (j-1) * dt_;

    // compute velocities and accelerations with central difference
    for (int i = 1; i < joint_traj.points.size() - 1; i++) {
        for (int j = 0; j < joint_traj.joint_names.size(); j++) {
            joint_traj.points[i].velocities[j] = (joint_traj.points[i+1].positions[j] - joint_traj.points[i-1].positions[j]) / (2 * dt_);
            joint_traj.points[i].accelerations[j] = (joint_traj.points[i+1].positions[j] - 2 * joint_traj.points[i].positions[j] + joint_traj.points[i-1].positions[j]) / (dt_ * dt_);
        }
    }

    return;

}

NodePtr TPG::getExecStartNode(int robot_id) const {
    return start_nodes_[robot_id];
}

bool TPG::moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients) {
    // create one thread for each robot
    std::vector<std::thread> threads;

    executed_steps_.clear();
    for (int i = 0; i < num_robots_; i++) {
        NodePtr start_node = getExecStartNode(i);
        int start_t = start_node->timeStep;
        executed_steps_.push_back(std::make_unique<std::atomic<int>>(start_t));
    }

    for (int i = 0; i < num_robots_; i++) {
        threads.emplace_back(&TPG::moveit_async_execute_thread, this, std::ref(joint_names[i]), std::ref(clients[i]), i);
    }

    log("Waiting for all threads to finish...", LogLevel::INFO);
    for (auto &thread : threads) {
        thread.join();
    }

    return true;
}

void TPG::moveit_async_execute_thread(const std::vector<std::string> &joint_names, ros::ServiceClient &clients, int robot_id) {
    NodePtr node_i = getExecStartNode(robot_id);

    while (ros::ok()) {
        
        if (node_i->Type1Next == nullptr) {
            log("Robot " + std::to_string(robot_id) + " reached the end at step " + std::to_string(node_i->timeStep), LogLevel::INFO);
            return;
        }

        // check if we can execute the current node
        bool safe = true;
        for (auto edge : node_i->Type1Next->Type2Prev) {
            if (executed_steps_[edge->nodeFrom->robotId]->load() < edge->nodeFrom->timeStep) {
                safe = false;
            }
        }
        if (!safe) {
            ros::Duration(0.03).sleep();
            continue;
        }

        // if we need to execute a policy, execute
        if (isPolicyNode(node_i->Type1Next)) {
            NodePtr endNode;
            executePolicy(node_i->Type1Next, endNode);
            node_i = endNode;
            continue;
        }
        
        int j = 0;

        moveit_msgs::ExecuteKnownTrajectory srv;
        srv.request.wait_for_execution = true;
        
        auto &joint_traj = srv.request.trajectory.joint_trajectory;
        joint_traj.joint_names = joint_names;

        joint_traj.points.clear();
        bool stop = false;
        do {
            trajectory_msgs::JointTrajectoryPoint point;
            point.time_from_start = ros::Duration(j * dt_);
            point.positions.resize(joint_names.size());
            point.velocities.resize(joint_names.size());
            point.accelerations.resize(joint_names.size());

            for (int d = 0; d < node_i->pose.joint_values.size(); d++) {
                point.positions[d] = node_i->pose.joint_values[d];
            }
            joint_traj.points.push_back(point);

            j++;
            node_i = node_i->Type1Next;
            
            //stop = (node_i->Type1Next == nullptr) || (node_i->Type1Next->Type2Prev.size() > 0) || (node_i->Type2Next.size() > 0);
            stop = (node_i->Type1Next == nullptr);
            // check type2 edges
            if (!stop) {
                for (auto edge : node_i->Type1Next->Type2Prev) {
                    if (executed_steps_[edge->nodeFrom->robotId]->load() < edge->nodeFrom->timeStep) {
                        stop = true;
                    }
                }
            }
            // check adg policies
            if (!stop) {
                if (isPolicyNode(node_i->Type1Next)) {
                    stop = true;
                }
            }
        } while (!stop);
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(j * dt_);
        point.positions.resize(joint_names.size());
        point.velocities.resize(joint_names.size());
        point.accelerations.resize(joint_names.size());
        for (int d = 0; d < node_i->pose.joint_values.size(); d++) {
            point.positions[d] = node_i->pose.joint_values[d];
        }
        joint_traj.points.push_back(point);

        // compute velocities and accelerations with central difference
        for (int i = 1; i < joint_traj.points.size() - 1; i++) {
            for (int j = 0; j < joint_names.size(); j++) {
                joint_traj.points[i].velocities[j] = (joint_traj.points[i+1].positions[j] - joint_traj.points[i-1].positions[j]) / (2 * dt_);
                joint_traj.points[i].accelerations[j] = (joint_traj.points[i+1].positions[j] - 2 * joint_traj.points[i].positions[j] + joint_traj.points[i-1].positions[j]) / (dt_ * dt_);
            }
        }

        
        // execute the plan now
        bool retry = true;
        while (retry) {
            retry = false;
            // compute th error of the current joint state vs the start state
            double error = 0;
            if (joint_states_.size() > robot_id && joint_states_[robot_id].size() >= joint_traj.points[0].positions.size()) {
                log("Robot " + std::to_string(robot_id) + " start/current errors ", LogLevel::DEBUG);

                std::string error_str = "Error: ";
                for (int d = 0; d < joint_states_[robot_id].size(); d++) {
                    double error_d = std::abs(joint_states_[robot_id][d] - joint_traj.points[0].positions[d]);
                    error += error_d;
                    error_str += std::to_string(error_d) + " ";
                }
                log(error_str, LogLevel::DEBUG);
                //TODO: check if this hack that set the joint_states to the start state is still necessary
                for (int d = 0; d < joint_traj.points[0].positions.size(); d++) {
                    joint_traj.points[0].positions[d] = joint_states_[robot_id][d];
                }
            }

            // Call the service to execute the trajectory
            bool result = clients.call(srv);
            if (!result) {
                log("Failed to call service for robot " + std::to_string(robot_id), LogLevel::ERROR);
                return;
            }
            
            int error_code = srv.response.error_code.val;
            log("Robot " + std::to_string(robot_id) + " traj execute service, code " + std::to_string(error_code), LogLevel::DEBUG);
            if (error_code == moveit_msgs::MoveItErrorCodes::TIMED_OUT) {
                log("Timeout, retrying...", LogLevel::INFO);
                retry = true;
                ros::Duration(0.01).sleep();
            }
            else if (error_code < 0) {
                return;
            }
            else {
                log("Robot " + std::to_string(robot_id) + " segment " + std::to_string(node_i->timeStep)
                    +" success, moving to the next segment", LogLevel::INFO);
                //executed_steps_[robot_id]->fetch_add(j); // allow following conflict
            }
            
        }
    }
}

void TPG::update_joint_states(const std::vector<double> &joint_states, int robot_id)
{
    while (joint_states_.size() <= robot_id) {
        joint_states_.push_back(std::vector<double>());
    }
    for (int i = 0; i < joint_states.size(); i++) {
        if (joint_states_[robot_id].size() <= i) {
            joint_states_[robot_id].push_back(joint_states[i]);
        } else {
            joint_states_[robot_id][i] = joint_states[i];
        }
    }

    if (executed_steps_.size() > robot_id && executed_steps_[robot_id] != nullptr) {
        int next_step = executed_steps_[robot_id]->load() + 1;
        if (next_step < numNodes_[robot_id]) {
            // check if the robot has reached the next step
            // get the node at the next step (if it exists)
            NodePtr node_i = start_nodes_[robot_id];
            for (int i = 0; i < next_step; i++) {
                if (node_i->Type1Next != nullptr) {
                    node_i = node_i->Type1Next;
                }
            } 

            if (node_i == nullptr) {
                log("Robot " + std::to_string(robot_id) 
                + " should not be null at step " + std::to_string(next_step) + " / " + std::to_string(numNodes_[robot_id]), LogLevel::WARN);
                return;
            }

            if (isPolicyNode(node_i)) {
                return;
            }

            double error = 0;
            for (int i = 0; i < joint_states_[robot_id].size(); i++) {
                error += std::abs(joint_states_[robot_id][i] - node_i->pose.joint_values[i]);
            }
            log("Robot " + std::to_string(robot_id) + " error " + std::to_string(error), LogLevel::DEBUG);
            if (error < config_.joint_state_thresh) {
                executed_steps_[robot_id]->fetch_add(1);
                log("Robot " + std::to_string(robot_id) + " reached step " + std::to_string(next_step), LogLevel::DEBUG);
            }
        }
    }

}

bool TPG::isPolicyNode(NodePtr node) const {
    return false;
}


}
