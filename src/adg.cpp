#include "adg.h"
#include "logger.h"
#include "policy.h"

namespace tpg {

ShortcutSamplerADG::ShortcutSamplerADG(const TPGConfig &config, std::shared_ptr<ActivityGraph> act_graph,
                    const std::vector<std::vector<NodePtr>> &intermediate_nodes)
    : ShortcutSampler(config)
{
    act_graph_ = act_graph;
    intermediate_nodes_ = intermediate_nodes;
    skip_home_ = !config.sync_task;
}

void ShortcutSamplerADG::init(const std::vector<NodePtr> &start_nodes,
    const std::vector<int> &numNodes, const std::vector<std::vector<int>> &earliest_t,
    const std::vector<std::vector<NodePtr>> &timed_nodes)
{
    num_robots_ = start_nodes.size();
    timed_nodes_ = timed_nodes;
    nodes_.clear();
    act_ids_.clear();
    act_lengths_.clear();

    nodes_.resize(num_robots_);
    act_ids_.resize(num_robots_);
    act_lengths_.resize(num_robots_);

    // save all the nodes, and activity ids, and activity lengths

    for (int i = 0; i < num_robots_; i++) {
        auto node = start_nodes[i];
        int j = 0;

        std::vector<NodePtr> nodes_i;
        std::vector<int> act_ids_i;
        int act_id = 0;

        while (node != nullptr) {
            // add node
            nodes_i.push_back(node);

            // add activity id
            auto act_i = act_graph_->get(i, act_id);
            // we skip home activity
            while (skip_home_&& act_id < act_graph_->num_activities(i) - 1  && act_i->is_skippable()) {
                act_id++;
                act_i = act_graph_->get(i, act_id);
            }
            act_ids_i.push_back(act_id);

            // check if we need to switch to next activity
            if (intermediate_nodes_[i][act_id * 2 + 1]->timeStep == node->timeStep) {
                act_id++;
            }

            // iterate to next node
            j++;
            node = node->Type1Next;
        }
        nodes_[i] = nodes_i;
        act_ids_[i] = act_ids_i;

        act_lengths_[i].resize(act_graph_->num_activities(i), 0);
        for (int act_id : act_ids_[i]) {
            act_lengths_[i][act_id]++;
        }

    }
    numNodes_ = numNodes;

    resetFailedShortcuts();
}


bool ShortcutSamplerADG::sampleUniform(Shortcut &shortcut) {
    int i = std::rand() % num_robots_;

    int startNode = std::rand() % numNodes_[i];
    int act_id = act_ids_[i][startNode];
    int act_length = 0;
    shortcut.activity = act_graph_->get(i, act_id);

    // for (int j = 0; j <= act_id; j++) {
    //     act_length += act_lengths_[i][j];
    // }

    // if (startNode >= act_length - 2) {
    //     return false;
    // }
    // int length = std::rand() % (act_length - startNode - 2) + 2;
    // int endNode = p_ + length;
    for (int j = 0; j < act_id; j++) {
        act_length += act_lengths_[i][j];
    }
    int endNode = act_length + std::rand() % act_lengths_[i][act_id];
    if (endNode < startNode) {
        std::swap(startNode, endNode);
    }
    if (startNode + 1 >= endNode) {
        return false;
    }

    shortcut.ni = nodes_[i][startNode];
    shortcut.nj = nodes_[i][endNode];
    assert(shortcut.activity != nullptr);

    log("Sampled shortcut from robot " + std::to_string(i) + " activity " + shortcut.activity->type_string() + 
        " timestep " + std::to_string(shortcut.ni.lock()->timeStep) +
        " to timestep " + std::to_string(shortcut.nj.lock()->timeStep), LogLevel::DEBUG);
    
    return true;
}

bool ShortcutSamplerADG::sampleComposite(Shortcut &shortcut) {
    return sampleUniform(shortcut);
}

ShortcutIteratorADG::ShortcutIteratorADG(const TPGConfig &config, std::shared_ptr<ActivityGraph> act_graph,
                    const std::vector<std::vector<NodePtr>> &intermediate_nodes)
    : ShortcutIterator(config)
{
    act_graph_ = act_graph;
    skip_home_ = !config.sync_task;

    for (int i = 0; i < act_graph->num_robots(); i++) {
        std::vector<NodePtr> seg_node;
        int act_id = 0;
        while (act_id < act_graph->num_activities(i)) {
            seg_node.push_back(intermediate_nodes[i][act_id * 2]);
            while (skip_home_ && act_id < act_graph->num_activities(i) - 1 && act_graph->get(i, act_id)->is_skippable()) {
                act_id++;
            }
            seg_node.push_back(intermediate_nodes[i][act_id * 2 + 1]);
            act_id++;
        }
        seg_nodes_.push_back(seg_node);
    }
}

void ShortcutIteratorADG::init(const std::vector<NodePtr> &start_nodes,
                        const std::vector<int> &numNodes,
                        const std::vector<NodePtr> &end_nodes)
{
    // init the queue
    q_i = std::queue<NodePtr>();
    q_j = std::queue<NodePtr>();
    num_robots_ = start_nodes.size();
    start_nodes_ = start_nodes;
    end_nodes_ = end_nodes;
    numNodes_ = numNodes;

    // find all the node's start segment node and end segment node
    for (int i = 0; i < num_robots_; i++) {
        for (int seg_id = 0; seg_id < seg_nodes_[i].size()/2; seg_id ++) {
            auto node_i = seg_nodes_[i][seg_id*2];
            if (skip_home_ && act_graph_->get(i, node_i->actId)->is_skippable()) {
                if (backward_doubleloop) {
                    q_i.push(node_i);
                    q_j.push(seg_nodes_[i][seg_id*2+1]);
                }
                else {
                    q_i.push(node_i);
                    q_j.push(node_i->Type1Next);
                }
                rob_seg_idx.push(std::make_pair(i, seg_id));
                std::cout << node_i->timeStep << " " << seg_nodes_[i][seg_id*2+1]->timeStep << "\n";
            }   
        }
    }
    std::cout << "finished initializing shortcut iterator\n";
}

bool ShortcutIteratorADG::step_begin(Shortcut &shortcut) {
    NodePtr node_i = q_i.front();
    NodePtr node_j = q_j.front();
    int robot_id = rob_seg_idx.front().first;
    int seg_id = rob_seg_idx.front().second;
    q_i.pop();
    q_j.pop();
    rob_seg_idx.pop();
    
    auto seg_start = seg_nodes_[robot_id][seg_id*2];
    auto seg_end = seg_nodes_[robot_id][seg_id*2+ 1];

    if (backward_doubleloop) {
        if (node_i->timeStep == node_j->timeStep) {
            q_i.push(seg_start);
            q_j.push(seg_end);
            log("robot " + std::to_string(robot_id) + 
                " seg " + std::to_string(seg_id) + " restart outer loop", LogLevel::DEBUG);
            rob_seg_idx.push(std::make_pair(robot_id, seg_id));
            return false;
        }
        else if (node_i->timeStep + 1 == node_j->timeStep) {
            q_i.push(node_i->Type1Next);
            q_j.push(seg_end);
            log("robot " + std::to_string(robot_id) + 
                " seg " + std::to_string(seg_id) + " restart inner loop", LogLevel::DEBUG);
            rob_seg_idx.push(std::make_pair(robot_id, seg_id));
            return false;
        }
       
    }
    else {
        //std::cout << "robot " << robot_id << " seg " << seg_id << " " 
        //    << node_i->timeStep << " " << node_j->timeStep << " " << seg_end->timeStep << "\n";
        if (node_j->timeStep >= seg_end->timeStep) {
            if (node_i->timeStep < seg_end->timeStep - 1) {
                q_i.push(node_i->Type1Next);
                q_j.push(node_i->Type1Next->Type1Next);
                rob_seg_idx.push(std::make_pair(robot_id, seg_id));
                log("robot " + std::to_string(robot_id) + 
                " seg " + std::to_string(seg_id) + " restart inner loop", LogLevel::DEBUG);
            }
            else {
                q_i.push(seg_start);
                q_j.push(seg_start->Type1Next);
                rob_seg_idx.push(std::make_pair(robot_id, seg_id));
                log("robot " + std::to_string(robot_id) +
                " seg " + std::to_string(seg_id) + " restart outer loop", LogLevel::DEBUG);
            }
            return false;
        }
    }

    shortcut.ni = node_i;
    shortcut.nj = node_j;
    int act_id = node_i->actId;
    while (skip_home_ && act_id < act_graph_->num_activities(robot_id) - 1 && act_graph_->get(robot_id, act_id)->is_skippable()) {
        act_id++;
    }
    shortcut.activity = act_graph_->get(robot_id, act_id);
    rob_seg_idx.push(std::make_pair(robot_id, seg_id));
    return true;
}

void ShortcutIteratorADG::step_end(const Shortcut &shortcut) {
    ShortcutIterator::step_end(shortcut);
}


void ADG::reset()
{
    TPG::reset();
    intermediate_nodes_.clear();
}

ADG::ADG(std::shared_ptr<ActivityGraph> activity_graph) : act_graph_(activity_graph) {
    num_robots_ = act_graph_->num_robots();
    start_nodes_.resize(num_robots_);
    end_nodes_.resize(num_robots_);
    intermediate_nodes_.resize(num_robots_);
    numNodes_.resize(num_robots_, 0);
    solution_.resize(num_robots_);
    exec_start_act_.resize(num_robots_, 0);
}

bool ADG::init_from_asynctrajs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const MRTrajectory &trajectories) {
    dt_ = config.dt;
    config_ = config;
    
    instance->resetScene(true);
    
    std::vector<std::vector<int>> reached_t;
    reached_t.resize(num_robots_);

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_robots_; i++) {
        std::vector<NodePtr> nodes_i;
        std::vector<NodePtr> inter_nodes;
       
        for (int idx = 0; idx < trajectories[i].times.size(); idx++) {
            auto node = std::make_shared<Node>(i, idx);
            node->pose = trajectories[i].trajectory[idx];
            node->actId = trajectories[i].act_ids[idx];
            nodes_i.push_back(node);
            if (idx == 0) {
                int act_id_begin = trajectories[i].act_ids.front();
                for (int j = 0; j < act_id_begin; j++) {
                    inter_nodes.push_back(node);
                    inter_nodes.push_back(node);
                }
                inter_nodes.push_back(node);
            }
            else {
                if (trajectories[i].act_ids[idx - 1] != trajectories[i].act_ids[idx]) {
                    inter_nodes.push_back(nodes_i[idx-1]);
                    int skipped_act_count = trajectories[i].act_ids[idx] - trajectories[i].act_ids[idx - 1] - 1;
                    for (int j = 0; j < skipped_act_count; j++) {
                        inter_nodes.push_back(node);
                        inter_nodes.push_back(node);
                    }
                    inter_nodes.push_back(node);
                }
            }
            reached_t[i].push_back(trajectories[i].times[idx] / dt_);
        }
        inter_nodes.push_back(nodes_i.back());
        
        if (config_.sync_task == false) {
            // remove repeatedly waiting nodes in home and home_handover
            int num_acts = act_graph_->num_activities(i);
            for (int j = num_acts - 1; j >= 1; j--) {
                if (act_graph_->get(i, j)->is_skippable()) {
                    int stepa = inter_nodes[j*2]->timeStep;
                    int stepb = inter_nodes[j*2+1]->timeStep;
                    if ((stepb - stepa) <= 1) {
                        nodes_i.erase(nodes_i.begin() + stepa, nodes_i.begin() + stepb + 1);
                        inter_nodes.erase(inter_nodes.begin() + j*2, inter_nodes.begin() + j*2 + 2);
                        reached_t[i].erase(reached_t[i].begin() + stepa, reached_t[i].begin() + stepb + 1);
                        act_graph_->remove_act(i, j); 
                    }
                }
            }
        }

        int act_id = 0;
        for (int j = 1; j < nodes_i.size(); j++) {
            nodes_i[j-1]->Type1Next = nodes_i[j];
            nodes_i[j]->Type1Prev = nodes_i[j-1];
            nodes_i[j]->timeStep = j;
            nodes_i[j]->actId = act_id;
            if (nodes_i[j] == inter_nodes[act_id * 2 + 1]) {
                act_id++;   
            }
        }

        numNodes_[i] = nodes_i.size();
        start_nodes_[i] = nodes_i.front();
        end_nodes_[i] = nodes_i.back();
        intermediate_nodes_[i] = inter_nodes;
    
    }

    if (addTaskDeps() == false) {
        if (hasCycle()) {
            log("Naive TPG already has cycle after adding task deps", LogLevel::ERROR);
            return false;
        }
        return false;
    }

    // add collision dependencies type 2 edges
    if (findCollisionDeps(instance, reached_t, false) == false) {
        return false;
    }
    
    t_init_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6;
    t_start = std::chrono::high_resolution_clock::now();

    transitiveReduction();
    t_simplify_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6; 

    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }

    int numtype2edges = getTotalType2Edges(); 
    log("ADG initialized with " + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges in "
        + std::to_string(t_init_ + t_simplify_) + "s", LogLevel::HLINFO);

    findFlowtimeMakespan(pre_shortcut_flowtime_, pre_shortcut_makespan_);
    post_shortcut_flowtime_ = pre_shortcut_flowtime_;
    post_shortcut_makespan_ = pre_shortcut_makespan_;

    computePathLength(instance);
    double nowait_time = 0;
    for (int i = 0; i < num_robots_; i++) {
        nowait_time += end_nodes_[i]->timeStep * config_.dt;
    }
    wait_time_ = pre_shortcut_flowtime_ - nowait_time;
    log("Flowtime: " + std::to_string(pre_shortcut_flowtime_) + " Makespan: " + std::to_string(pre_shortcut_makespan_)
        + " Wait Time " + std::to_string(wait_time_), LogLevel::INFO);

    return true;
    
}

bool ADG::init_from_tpgs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs) {
    dt_ = config.dt;
    config_ = config;
    
    instance->resetScene(true);

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_robots_; i++) {
        std::vector<NodePtr> inter_nodes;
        std::vector<double> hand_values(instance->getHandDOF(i), 0.0);
        int tpg_id = 0;
        for (int act_id = 0; act_id < act_graph_->num_activities(i); act_id++) {
            std::shared_ptr<const Activity> act = act_graph_->get(i, act_id);
            NodePtr inter_start_node;
            NodePtr inter_end_node;

            if (act->type == Activity::Type::open_gripper || act->type == Activity::Type::close_gripper) {
                // these tasks do not have a trajectory, so we just set the start node and end node as before
                if (inter_nodes.empty()) {
                    log("gripper open/close task should not be the first task", LogLevel::ERROR);
                    return false;
                }
                auto last_node = inter_nodes.back();
                inter_start_node = std::make_shared<Node>(i, last_node->timeStep + 1);
                inter_start_node->actId = act_id;
                inter_start_node->pose = last_node->pose;
                hand_values = act->end_pose.hand_values;
                inter_start_node->pose.hand_values = hand_values;
                inter_end_node = inter_start_node;
                numNodes_[i]++;
            }
            else {
                inter_start_node = tpgs[tpg_id]->getStartNode(i);
                inter_end_node = tpgs[tpg_id]->getEndNode(i);

                NodePtr iter_node = inter_start_node;
                while (iter_node != inter_end_node) {
                    iter_node->timeStep += numNodes_[i];
                    iter_node->actId = act_id;
                    iter_node->pose.hand_values = hand_values;
                    iter_node = iter_node->Type1Next;
                }
                inter_end_node->actId = act_id;
                inter_end_node->timeStep += numNodes_[i];
                inter_end_node->pose.hand_values = hand_values;

                numNodes_[i] += tpgs[tpg_id]->getNumNodes(i);
                tpg_id++;
            }
            if (act_id > 0) {
                inter_nodes.back()->Type1Next = inter_start_node;
                inter_start_node->Type1Prev = inter_nodes.back();
            }

            inter_nodes.push_back(inter_start_node);
            inter_nodes.push_back(inter_end_node);

        }
        intermediate_nodes_[i] = inter_nodes;
        start_nodes_[i] = inter_nodes.front();
        end_nodes_[i] = inter_nodes.back();
    }
    
    if (addTaskDeps() == false) {
        return false;
    }

    // recompute the time step for each node
    std::vector<std::vector<int>> reached_t;
    std::vector<int> reached_end;
    int num_act = act_graph_->num_activities(0);
    for (int i = 0; i < num_robots_; i++) {
        if (num_act != act_graph_->num_activities(i)) {
            log("number of activities for each robot should be the same", LogLevel::ERROR);
            return false;
        }
    }
    findEarliestReachTimeSyncAct(num_act, reached_t, reached_end);
    
    // add collision dependencies type 2 edges
    if (findCollisionDeps(instance, reached_t, true) == false) {
        return false;
    }
    
    t_init_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6;
    t_start = std::chrono::high_resolution_clock::now();

    transitiveReduction();
    t_simplify_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6; 

    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }

    int numtype2edges = getTotalType2Edges(); 
    log("ADG initialized with " + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges in "
        + std::to_string(t_init_ + t_simplify_) + "s", LogLevel::HLINFO);

    findFlowtimeMakespan(pre_shortcut_flowtime_, pre_shortcut_makespan_);
    computePathLength(instance);
    double nowait_time = 0;
    for (int i = 0; i < num_robots_; i++) {
        nowait_time += end_nodes_[i]->timeStep * config_.dt;
    }
    wait_time_ = pre_shortcut_flowtime_ - nowait_time;

    log("Flowtime: " + std::to_string(pre_shortcut_flowtime_) + " Makespan: " + std::to_string(pre_shortcut_makespan_), LogLevel::INFO);

    return true;
}

bool ADG::addTaskDeps() {
    // add task dependencies type 2 edges
    idType2Edges_ = 10000;
    for (int i = 0; i < num_robots_; i++) {
        std::cout << "robot " << i << " num activities " << act_graph_->num_activities(i) << "\n";
        for (int act_id = 0; act_id < act_graph_->num_activities(i); act_id++) {
            std::shared_ptr<const Activity> act = act_graph_->get(i, act_id);
            if (act == nullptr) {
                log("activity is null, robot " + std::to_string(i) + " act " + std::to_string(act_id), LogLevel::ERROR);
                return false;
            }
            for (auto dep : act->type2_prev) {
                std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                edge->edgeId = idType2Edges_++;
                auto dep_end_node = intermediate_nodes_[dep->robot_id][dep->act_id * 2 + 1];
                auto cur_start_node = intermediate_nodes_[i][act_id * 2];
                assert(dep_end_node != nullptr);
                if (dep_end_node->Type1Next == nullptr) {
                    log("activity depend on another terminal activity, this is deadlock!", LogLevel::ERROR);
                    return false;
                }
                edge->nodeFrom = dep_end_node->Type1Next;
                edge->nodeTo = cur_start_node;
                dep_end_node->Type1Next->Type2Next.push_back(edge);
                cur_start_node->Type2Prev.push_back(edge);
            }

            //if (act->type != 2 || act->type != 3 || act->type != 5 || act->type != 8 || act->type != 9 || act->type != 11) {
                if (config_.sync_task && act_id > 0) {
                    // enforce that tasks are executed synchronously by adding type2 dependencies
                    auto cur_start_node = intermediate_nodes_[i][act_id * 2];
                    for (int j = 0; j < num_robots_; j++) {
                        if (i == j) {
                            continue;
                        }
                        if (act_id > act_graph_->num_activities(j)) {
                            continue;
                        }
                        
                        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                        edge->edgeId = idType2Edges_++;
                        auto dep_end_node = intermediate_nodes_[j][act_id * 2 - 1];
                        if (dep_end_node == nullptr) {
                            log("activity end node somehow does not exist, check code again..", LogLevel::ERROR);
                            return false;
                        }
                        edge->nodeFrom = dep_end_node;
                        edge->nodeTo = cur_start_node;
                        dep_end_node->Type2Next.push_back(edge);
                        cur_start_node->Type2Prev.push_back(edge);
                        
                    }
                }
            //}
        } 
    }
    return true;
}

bool ADG::findCollisionDeps(std::shared_ptr<PlanInstance> instance, const std::vector<std::vector<int>> &reached_t,
                            bool skip_sametask) {
    // for each pair of activities between each pair of robot
    // if the activity does not depend on each other, then
    // update the planning scene (add objects, attach objects, or detach objects)
    // run collision check for each pair of nodes between the two activities
    // add type 2 edge if there is collision
    idType2Edges_ = 20000;
 
    for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < num_robots_; j++) {
            if (i == j) {
                continue;
            }
            for (int act_id_i = 0; act_id_i < act_graph_->num_activities(i); act_id_i++) {
                auto act_i = act_graph_->get(i, act_id_i);
                
                // updated attached / detached object
                updateScene(instance, act_i);

                auto act_i_start_node = intermediate_nodes_[i][act_id_i * 2];
                auto act_i_end_node = intermediate_nodes_[i][act_id_i * 2 + 1];

                // // run bfs on the task graph
                // std::vector<std::vector<bool>> visited;
                // for (int k = 0; k < num_robots_; k++) {
                //     visited.push_back(std::vector<bool>(act_graph_->num_activities(i), false));
                // }
                // act_graph_->bfs(act_i, visited, true);
                // act_graph_->bfs(act_i, visited, false);

                auto tic = std::chrono::high_resolution_clock::now();
                // run bfs on the node graph
                std::vector<std::vector<bool>> visited;
                for (int k = 0; k < num_robots_; k++) {
                    std::vector<bool> v(numNodes_[k], false);
                    visited.push_back(v);
                }
                bfs(act_i_start_node, visited, false);

                // keep track of attached objects for robot j
                ActPtr attached_act_j = nullptr, detacht_act_j = nullptr;
                // remove any existing attached object for robot j, if any
                for (int act_id_j = 0; act_id_j < act_graph_->num_activities(j); act_id_j++) {
                    // updated attached / detached object
                    auto act_j = act_graph_->get(j, act_id_j);
                    // if (visited[j][act_id_j]) {
                    //     // skip if the two activities are dependent
                    //     continue;
                    // }
                    auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
                    auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];

                    if(reached_t[j][act_j_start_node->timeStep] > reached_t[i][act_i_end_node->timeStep]) {
                        log("Finished building edges to robot " + std::to_string(i) + " activity " + act_i->type_string() + " timestep " 
                            + std::to_string(act_i_end_node->timeStep), LogLevel::INFO);
                        break;
                    }
                    
                    if (visited[j][act_j_end_node->timeStep] || (skip_sametask && act_id_j == act_id_i)) {
                        // skip if the two activities are dependent
                        // skip if they are in the same tpg because type-2 dependencies would have already been build
                        if (act_j->obj_attached.size() > 0) {
                            attached_act_j = act_j;
                        }
                        if (act_j->obj_detached.size() > 0 && attached_act_j != nullptr) {
                            attached_act_j = nullptr;
                        }
                        continue;
                    }

                    if (attached_act_j != nullptr) {
                        updateScene(instance, attached_act_j, i);
                        attached_act_j = nullptr;
                    }
                    updateScene(instance, act_j, j);

                    if(config_.parallel) {
                        findCollisionDepsTaskParallel(i, j, act_i, act_j, act_i_start_node, act_j_start_node, act_i_end_node, act_j_end_node, instance, reached_t, visited);
                    }
                    else {
                        findCollisionDepsTask(i, j, act_i, act_j, act_i_start_node, act_j_start_node, act_i_end_node, act_j_end_node, instance, reached_t, visited);
                    }

                }
                auto attached_obj_j = instance->getAttachedObjects(j);
                if (attached_obj_j.size() > 0) {
                    Object obj = attached_obj_j[0];
                    instance->detachObjectFromRobot(obj.name, act_graph_->get(j, act_graph_->num_activities(j) - 1)->end_pose);
                    if (config_.print_contact) {
                        instance->updateScene();
                    }
                    if (act_graph_->get_last_obj(obj.name)->vanish) {
                        instance->removeObject(obj.name);
                        if (config_.print_contact) {
                            instance->updateScene();
                        }
                        log("remove object " + obj.name + " from the scene", LogLevel::DEBUG);
                    }
                }
            }
            auto attached_obj_i = instance->getAttachedObjects(i);
            if (attached_obj_i.size() > 0) {
                Object obj = attached_obj_i[0];
                instance->detachObjectFromRobot(obj.name, act_graph_->get(i, act_graph_->num_activities(i) - 1)->end_pose);
                if (config_.print_contact) {
                    instance->updateScene();
                }
                if (act_graph_->get_last_obj(obj.name)->vanish) {
                    instance->removeObject(obj.name);
                    if (config_.print_contact) {
                        instance->updateScene();
                    }
                    log("remove object " + obj.name + " from the scene", LogLevel::DEBUG);
                }
            }
        }
    }
    return true;
}

void ADG::findCollisionDepsTask(int i, int j, ActPtr act_i, ActPtr act_j,
                                NodePtr act_i_start_node, NodePtr act_j_start_node, NodePtr act_i_end_node,
                                NodePtr act_j_end_node, std::shared_ptr<PlanInstance> instance,
                                const std::vector<std::vector<int>> &reached_t,
                                const std::vector<std::vector<bool>> &visited)
{
    // check collision
    NodePtr iter_node_i = act_i_start_node;
    NodePtr iter_node_j_start = act_j_start_node;
    while (iter_node_i != nullptr && iter_node_i->timeStep <= act_i_end_node->timeStep) {
        NodePtr iter_node_j = iter_node_j_start;
        bool inCollision = false;
        while (iter_node_j != nullptr &&
            iter_node_j->timeStep <= act_j_end_node->timeStep &&
            reached_t[j][iter_node_j->timeStep] < reached_t[i][iter_node_i->timeStep]) 
        {
            if (visited[j][iter_node_j->timeStep]) {
                iter_node_j = iter_node_j->Type1Next;
                continue;
            }
            bool has_collision = instance->checkCollision({iter_node_i->pose, iter_node_j->pose}, true);
            if (has_collision) {
                if (config_.print_contact) {
                    //instance->printKnownObjects();
                    instance->checkCollision({iter_node_i->pose, iter_node_j->pose}, true, true);
                }
                inCollision = true;
            }
            else if (inCollision) {
                inCollision = false;
                std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                edge->edgeId = idType2Edges_++;
                edge->nodeFrom = iter_node_j;
                edge->nodeTo = iter_node_i;
                iter_node_j->Type2Next.push_back(edge);
                iter_node_i->Type2Prev.push_back(edge);
                iter_node_j_start = iter_node_j;
                log("add type 2 edge from robot " + std::to_string(j) + " activity " 
                    + act_j->type_string() + " timestep " + std::to_string(iter_node_j->timeStep)
                    + " " + std::to_string(reached_t[j][iter_node_j->timeStep])
                    + " to robot " + std::to_string(i) + " activity " 
                    + act_i->type_string() +  " timestep " + std::to_string(iter_node_i->timeStep)
                    + " " + std::to_string(reached_t[i][iter_node_i->timeStep]) , LogLevel::INFO);

            }
            iter_node_j = iter_node_j->Type1Next;
        }
        if (inCollision) {
            assert(iter_node_j != nullptr);
            std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
            edge->edgeId = idType2Edges_++;
            edge->nodeFrom = iter_node_j;
            edge->nodeTo = iter_node_i;
            iter_node_j->Type2Next.push_back(edge);
            iter_node_i->Type2Prev.push_back(edge);
            iter_node_j_start = iter_node_j;
            log("add type 2 edge (end act) from robot " + std::to_string(j) + " activity " 
                    + act_j->type_string() + " timestep " + std::to_string(iter_node_j->timeStep)
                    + " " + std::to_string(reached_t[j][iter_node_j->timeStep])
                    + " to robot " + std::to_string(i) + " activity " 
                    + act_i->type_string() +  " timestep " + std::to_string(iter_node_i->timeStep)
                    + " " + std::to_string(reached_t[i][iter_node_i->timeStep]) , LogLevel::INFO);

        }
        iter_node_i = iter_node_i->Type1Next;
    }
}

void ADG::findCollisionDepsTaskParallel(int i, int j, ActPtr act_i, ActPtr act_j,
    NodePtr act_i_start_node, NodePtr act_j_start_node, NodePtr act_i_end_node,
    NodePtr act_j_end_node, std::shared_ptr<PlanInstance> instance,
    const std::vector<std::vector<int>> &reached_t,
    const std::vector<std::vector<bool>> &visited)
{
    // First, create vectors of nodes for easier indexing
    std::vector<NodePtr> nodes_i, nodes_j;
    
    // Populate node vectors
    for (NodePtr node = act_i_start_node; 
         node != nullptr && node->timeStep <= act_i_end_node->timeStep; 
         node = node->Type1Next) {
        nodes_i.push_back(node);
    }
    
    // Populate nodes_j in reverse order
    for (NodePtr node = act_j_end_node; 
         node != nullptr && node->timeStep >= act_j_start_node->timeStep; 
         node = node->Type1Prev) {
        nodes_j.push_back(node);
    }

    std::vector<std::tuple<NodePtr, NodePtr, bool>> collision_results;
    std::mutex results_mutex;

    #pragma omp parallel
    {
        std::vector<std::tuple<NodePtr, NodePtr, bool>> thread_results;
        
        #pragma omp for
        for (int idx_i = 0; idx_i < nodes_i.size(); idx_i++) {
            NodePtr iter_node_i = nodes_i[idx_i];
            
            for (int idx_j = 0; idx_j < nodes_j.size(); idx_j++) {
                NodePtr iter_node_j = nodes_j[idx_j];
                
                if (reached_t[j][iter_node_j->timeStep] >= reached_t[i][iter_node_i->timeStep]) {
                    continue;
                }
                
                if (visited[j][iter_node_j->timeStep]) {
                    continue;
                }
                
                bool has_collision = instance->checkCollision(
                    {iter_node_i->pose, iter_node_j->pose}, true);
                
                if (has_collision) {
                    if (config_.print_contact) {
                        #pragma omp critical
                        {
                            instance->checkCollision(
                                {iter_node_i->pose, iter_node_j->pose}, true, true);
                        }
                    }
                    
                    // Since we're iterating backwards, this is our collision exit point
                    #pragma omp critical
                    {
                        thread_results.emplace_back(iter_node_j, iter_node_i, true);
                    }
                    break;  // Break inner loop as we found our collision exit point
                }
            }
        }
        
        #pragma omp critical
        {
            collision_results.insert(collision_results.end(), 
                thread_results.begin(), thread_results.end());
        }
    }
    
    // Process results and create edges as before
    for (const auto& [node_j, node_i, collision] : collision_results) {
        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
        edge->edgeId = idType2Edges_++;
        edge->nodeFrom = node_j;
        edge->nodeTo = node_i;
        node_j->Type2Next.push_back(edge);
        node_i->Type2Prev.push_back(edge);
        
        log("add type 2 edge from robot " + std::to_string(j) + " activity "
            + act_j->type_string() + " timestep " + std::to_string(node_j->timeStep)
            + " " + std::to_string(reached_t[j][node_j->timeStep])
            + " to robot " + std::to_string(i) + " activity "
            + act_i->type_string() + " timestep " + std::to_string(node_i->timeStep)
            + " " + std::to_string(reached_t[i][node_i->timeStep]) , LogLevel::DEBUG);
    }
}

void ADG::findEarliestReachTimeSyncAct(int num_act, std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end)
{
    reached_t.clear();
    reached_end.clear();
    std::vector<NodePtr> nodes;
    nodes.resize(num_robots_);
    reached_end.resize(num_robots_, -1);
    
    for (int i = 0; i < num_robots_; i++) {
        std::vector<int> v(numNodes_[i], -1);
        reached_t.push_back(v);
    }

    int flowtime_i = 0;
    int j = 0;
    for (int act_id = 0; act_id < num_act; act_id++) {
        for (int i = 0; i < num_robots_; i++) {
            nodes[i] = intermediate_nodes_[i][act_id * 2];
            reached_end[i] = -1;
        }

        // we synchronize the activities for all robots
        // i.e. a robot has to wait for all other robots to finish the previous activity
        bool allReached = false;
        while(!allReached) {
            for (int i = 0; i < num_robots_; i++) {
                if (reached_t[i][nodes[i]->timeStep] == -1) {
                    reached_t[i][nodes[i]->timeStep] = j;
                }
            }
            for (int i = 0; i < num_robots_; i++) {
                if (nodes[i]->timeStep < intermediate_nodes_[i][act_id * 2 + 1]->timeStep) {
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
    }
 

}


bool ADG::saveToDotFile(const std::string& filename) const {

    std::ofstream out(filename);
    out << "digraph G {" << std::endl;

    // define node attributes here
    out << "node [shape=ellipse];" << std::endl;
    out << "rankdir=LR;" << std::endl;

    // define all the nodes
    for (int i = 0; i < num_robots_; i++) {
        out << "subgraph cluster_" << i << " {" << std::endl;
        out << "label = \"Robot " << i << "\";" << std::endl;
        out << "rank=same;" << std::endl;
        NodePtr node_i = start_nodes_[i];
        std::vector<NodePtr> salient_nodes;
        while (node_i != nullptr) {
            if (node_i->Type1Prev == nullptr) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type1Next == nullptr) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type1Next->actId > node_i->actId &&
                act_graph_->get(i, node_i->Type1Next->actId)->type != act_graph_->get(i, node_i->actId)->type) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type2Prev.size() > 0) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type2Next.size() > 0) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            
            node_i = node_i->Type1Next;
        }
        assert(salient_nodes.size() > 0);

        node_i = salient_nodes[0];
        out << "n" << i << "_" << node_i->timeStep;
        for (int j = 1; j < salient_nodes.size(); j++) {
            out << " -> " << "n" << i << "_" << salient_nodes[j]->timeStep;
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

std::string ADG::get_r1_inhand_obj_name()
{
    return r1_in_hand_obj_name_;
}

std::string ADG::get_r2_inhand_obj_name()
{
    return r2_in_hand_obj_name_;
}

Eigen::MatrixXd ADG::get_r1_inhand_goal_q()
{
    return r1_in_hand_goal_q_;
}

Eigen::MatrixXd ADG::get_r2_inhand_goal_q()
{
    return r2_in_hand_goal_q_;
}

void ADG::update_joint_states(const std::vector<double> &joint_states, int robot_id)
{
    TPG::update_joint_states(joint_states, robot_id);
    if (policy_ != nullptr) {
        policy_->update_joint_states(joint_states, robot_id);
    }
    
    if (num_robots_ > executed_acts_.size()) {
        return;
    }
   
    int act_id = executed_acts_[robot_id]->load();
    if (act_id >= act_graph_->num_activities(robot_id)) {
        return;
    }
    auto act = act_graph_->get(robot_id, act_id);

    NodePtr node = intermediate_nodes_[robot_id][act_id * 2];
    if (isPolicyNode(node)) {
        // executed node is only updated after policy execution
        return;
    }

    // compare the joint values
    double error = 0;
    for (int i = 0; i < joint_states_[robot_id].size(); i++) {
        error += std::abs(joint_states_[robot_id][i] - act->end_pose.joint_values[i]);
    }
    
    if (error < config_.joint_state_thresh) {
        executed_acts_[robot_id]->fetch_add(1);
        log("Robot " + std::to_string(robot_id) + " finished activity " + act->type_string(), LogLevel::INFO);
        int act_id = executed_acts_[robot_id]->load();
        while (act_id < act_graph_->num_activities(robot_id) - 1 && act_graph_->get(robot_id, act_id)->is_skippable()){
            act_id ++;
            executed_acts_[robot_id]->fetch_add(1);
        }

        // update any attached object
        if (instance_ && act_id < act_graph_->num_activities(robot_id)) {
            auto act = act_graph_->get(robot_id, act_id);
            for (auto obj : act->obj_attached) {
                if (obj->handover && act->type == Activity::Type::home_receive) {
                    // the handover object is attached when it was detached
                    continue;
                }
                if (obj->vanish) {
                    instance_->addMoveableObject(obj->obj);
                    instance_->updateScene();
                }
                instance_->attachObjectToRobot(obj->obj.name, robot_id, obj->next_attach_link, act->start_pose);
                instance_->updateScene();
                log("attach object " + obj->obj.name + " to robot " + std::to_string(robot_id), LogLevel::INFO);
                if(robot_id == 0)
                {
                    r1_in_hand_obj_name_ = obj->obj.name;
                }
                if(robot_id == 1)
                {
                    r2_in_hand_obj_name_ = obj->obj.name;
                }
            }
            for (auto obj : act->obj_detached) {
                if(robot_id == 0)
                {
                    r1_in_hand_obj_name_ = "None";
                }
                if(robot_id == 1)
                {
                    r2_in_hand_obj_name_ = "None";
                }
                instance_->detachObjectFromRobot(obj->obj.name, act->start_pose);
                instance_->updateScene();
                if (obj->vanish) {
                    instance_->removeObject(obj->obj.name);
                    instance_->updateScene();
                }
                log("detach object " + obj->obj.name + " from robot " + std::to_string(robot_id), LogLevel::INFO);
                if (obj->handover && act->type == Activity::Type::handover_twist) {
                    // also attach the obj
                    auto home_receive_act = obj->next_attach;
                    instance_->attachObjectToRobot(obj->obj.name, home_receive_act->robot_id, obj->next_attach_link, home_receive_act->start_pose);
                    instance_->updateScene();
                    log("attach object " + obj->obj.name + " to robot " + std::to_string(home_receive_act->robot_id), LogLevel::INFO);
                    if (home_receive_act->robot_id == 0)
                    {
                        r1_in_hand_obj_name_ = obj->obj.name;
                    }
                    if (home_receive_act->robot_id == 1)
                    {
                        r2_in_hand_obj_name_ = obj->obj.name;
                    }
                }
            }
        }
        
    }
    
}

bool ADG::moveit_execute(std::shared_ptr<MoveitInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) 
{
    instance_ = instance;
    for (int i = 0; i < num_robots_; i++) {
        int act_id = exec_start_act_[i];
        executed_acts_.push_back(std::make_unique<std::atomic<int>>(act_id));
        while (act_id < act_graph_->num_activities(i) - 1 && act_graph_->get(i, act_id)->is_skippable()){
            act_id ++;
            executed_acts_[i]->fetch_add(1);
        }
    }
    return TPG::moveit_execute(instance, move_group);
}

bool ADG::moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients) 
{
    for (int i = 0; i < num_robots_; i++) {
        int act_id = exec_start_act_[i];
        executed_acts_.push_back(std::make_unique<std::atomic<int>>(act_id));
        while (act_id < act_graph_->num_activities(i) - 1 && act_graph_->get(i, act_id)->is_skippable()){
            act_id ++;
            executed_acts_[i]->fetch_add(1);
        }
    }
    return TPG::moveit_mt_execute(joint_names, clients);
}

void ADG::initSampler(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<NodePtr>> &timed_nodes) {
    shortcut_sampler_ = std::make_unique<ShortcutSamplerADG>(config_, act_graph_, intermediate_nodes_);
    shortcut_sampler_->init(start_nodes_, numNodes_, earliest_t, timed_nodes);
}

void ADG::initIterator() {
    shortcut_iterator_ = std::make_unique<ShortcutIteratorADG>(config_, act_graph_, intermediate_nodes_);
    shortcut_iterator_->init(start_nodes_, numNodes_, end_nodes_);
}

void ADG::updateScene(std::shared_ptr<PlanInstance> instance, ActPtr act, int existing_robot) const {
    std::vector<Object> attached_obj;
    if (existing_robot >= 0) {
        attached_obj = instance->getAttachedObjects(existing_robot);
    }

    int robot_id = act->robot_id;
    for (auto obj : act->obj_attached) {
        if (attached_obj.size() > 0 && obj->handover && obj->obj.name == attached_obj[0].name) {
            continue;
        }
        instance->moveRobot(robot_id, act->start_pose);
        if (config_.print_contact) {
            instance->updateScene();
        }
        if (obj->vanish && !instance->hasObject(obj->name())) {
            instance->addMoveableObject(obj->obj);
            if (config_.print_contact) {
                instance->updateScene();
            }
            log("add object " + obj->obj.name + " to the scene", LogLevel::DEBUG);
        }
        else {
            instance->moveObject(obj->obj);
            if (config_.print_contact) {
                instance->updateScene();
            }
        }
        instance->attachObjectToRobot(obj->obj.name, robot_id, obj->next_attach_link, act->start_pose);
        if (config_.print_contact) {
            instance->updateScene();
        }
    }
    for (auto obj : act->obj_detached) {
        if (attached_obj.size() > 0 && obj->handover && obj->obj.name == attached_obj[0].name) {
            continue;
        }
        log("detach object " + obj->obj.name + " from robot " + std::to_string(robot_id), LogLevel::DEBUG);
        instance->detachObjectFromRobot(obj->obj.name, act->start_pose);
        if (config_.print_contact) {
            instance->updateScene();
        }
         if (obj->vanish) {
            log("remove object " + obj->obj.name + " from the scene", LogLevel::DEBUG);
            instance->removeObject(obj->obj.name);
            if (config_.print_contact) {
                instance->updateScene();
            }
        }
    }
    
    for (auto col_node : act->collision_nodes) {
        instance->setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
    }
}

void ADG::checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
        const std::vector<std::vector<NodePtr>> &timedNodes) const {
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();
    // check if there is a shortcut between ni and nj
    assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);

    int robot_id = ni->robotId;

    // build collision environment
    ActPtr cur_act = shortcut.activity;
    assert(cur_act != nullptr);
    
    if (config_.print_contact) {
        instance->resetScene(true);
    }
    else {
        instance->resetScene(false);
    }
    // add all static objects that needs to be collision checked
    std::vector<ObjPtr> indep_objs = act_graph_->find_indep_obj(cur_act);
    for (auto obj : indep_objs) {
        if (instance->hasObject(obj->obj.name)) {
            Object obj_copy = obj->obj;
            obj_copy.name = obj->obj.name + "_copy";
            instance->addMoveableObject(obj_copy);
        }
        else {
            instance->addMoveableObject(obj->obj);
        }
        instance->setCollision(obj->obj.name, obj->obj.name, true);
        if (config_.print_contact) {
            instance->updateScene();
        }
    }

    for (int act_id = 0; act_id <= cur_act->act_id; act_id++) {
        // updated attached / detached object
        ActPtr act_j = act_graph_->get(robot_id, act_id);
        updateScene(instance, act_j);
    }

    int shortcutSteps = shortcut.path.size() + 2;
    for (int i = 1; i < shortcutSteps - 1; i++) {
        RobotPose pose_i = shortcut.path[i - 1];

        // check environment collision
        if (instance->checkCollision({pose_i}, false) == true) {
            if (config_.print_contact) {
                instance->checkCollision({pose_i}, false, true);
            }
            shortcut.col_type = CollisionType::STATIC; // collide with the evnrionment
            return;
        }
    }

    auto tic = std::chrono::high_resolution_clock::now();

    // find dependent parent and child nodes
    std::vector<std::vector<bool>> visited_act;
    for (int i = 0; i < num_robots_; i++) {
        visited_act.push_back(std::vector<bool>(act_graph_->num_activities(i), false));
    }
    act_graph_->bfs(cur_act, visited_act, true);
    act_graph_->bfs(cur_act, visited_act, false);

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
            for (int act_id_j = 0; act_id_j < act_graph_->num_activities(j); act_id_j++) {

                // updated attached / detached object
                auto act_j = act_graph_->get(j, act_id_j);
                updateScene(instance, act_j, robot_id);
                if (visited_act[j][act_id_j] == true) {
                    continue;
                }
                auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
                auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                while (!config_.sync_task && act_j->is_skippable() && act_id_j < act_graph_->num_activities(j) - 1) {
                    act_id_j ++;
                    act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                    act_j = act_graph_->get(j, act_id_j);
                }
                NodePtr node_j = act_j_start_node;
                while (node_j != nullptr && node_j->timeStep <= act_j_end_node->timeStep) {
                    if (visited[j][node_j->timeStep] == false) {
                        if (instance->checkCollision({ni->pose, node_j->pose}, true) ||
                            instance->checkCollision({nj->pose, node_j->pose}, true)) {
                            if (config_.print_contact) {
                                instance->checkCollision({ni->pose, node_j->pose}, true, true);
                                instance->checkCollision({nj->pose, node_j->pose}, true, true);
                            }
                            shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                            shortcut.n_robot_col = node_j;
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

    // for (int j = 0; j < num_robots_; j++) {
    //     Eigen::MatrixXi col_matrix_j(shortcutSteps, numNodes_[j]);
    //     col_matrix_j.setZero();
    //     col_matrix.push_back(col_matrix_j);
    // }

    auto t_bfs = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic).count();

    // check robot-robot collision
   
    for (int j = 0; j < num_robots_; j++) {
        if (j == ni->robotId) {
            continue;
        }
        for (int act_id_j = 0; act_id_j < act_graph_->num_activities(j); act_id_j++) {
            // updated attached / detached object
            auto act_j = act_graph_->get(j, act_id_j);
            updateScene(instance, act_j, robot_id);
            if (visited_act[j][act_id_j] == true) {
                continue;
            }
            auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
            auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];

            while (!config_.sync_task && act_j->is_skippable() && act_id_j < act_graph_->num_activities(j) - 1) {
                act_id_j ++;
                act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                act_j = act_graph_->get(j, act_id_j);
            }
            if (config_.parallel) {
                bool collision_found = false;
                std::vector<NodePtr> node_js;
                NodePtr node_j = act_j_start_node;
                while (node_j != nullptr && node_j->timeStep <= act_j_end_node->timeStep) {
                    node_js.push_back(node_j);
                    node_j = node_j->Type1Next;
                }

                for (int i = 1; i < shortcutSteps - 1; i++) {
                    RobotPose pose_i = shortcut.path[i - 1];
                    // Parallel region for the inner loop:
                    #pragma omp parallel default(none) shared(collision_found, shortcut, instance, config_, visited, j, pose_i, node_js) firstprivate(i)
                    { 
                        # pragma omp for
                        for (int k = 0; k < node_js.size(); k++) {
                            NodePtr node_j = node_js[k];
                            if (collision_found || visited[j][node_j->timeStep] == true) {
                                continue;
                            }

                            RobotPose pose_j = node_j->pose;
                            bool has_collision = instance->checkCollision({pose_i, pose_j}, true);
                            if (has_collision) {
                                #pragma omp critical
                                {
                                    if (!collision_found) {
                                        collision_found = true;
                                        if (config_.print_contact) {
                                            instance->checkCollision({pose_i, pose_j}, true, true);
                                        }
                                        shortcut.n_robot_col = node_j;
                                        shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                                    }
                                } // end omp critical
                            }
                        } // end pragma omp for

                    } // end pragma omp parallel
                    if (collision_found) {
                        return;
                    }
                }
            }
            else {
                for (int i = 1; i < shortcutSteps - 1; i++) {
                    RobotPose pose_i = shortcut.path[i - 1];
                    NodePtr node_j = act_j_start_node;
                    while (node_j != nullptr && node_j->timeStep <= act_j_end_node->timeStep) {
                        if (visited[j][node_j->timeStep] == false) {
                            
                            bool collide = instance->checkCollision({pose_i, node_j->pose}, true);
                            if (collide) {
                                if (config_.print_contact) {
                                    instance->checkCollision({pose_i, node_j->pose}, true, true);
                                }
                                shortcut.n_robot_col = node_j;
                                shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                                return;
                            }
                        }
                        node_j = node_j->Type1Next;
                    }
                }
            }
        }
    }

    shortcut.col_type = CollisionType::NONE;
    return;
}

bool ADG::isPolicyNode(NodePtr node) const {
    if (config_.run_policy == false) {
        return false;
    }
    int robot_id = node->robotId;
    Activity::Type type = act_graph_->get(robot_id, node->actId)->type;


    if (type == Activity::Type::drop_down || type == Activity::Type::pick_down ||
        type == Activity::Type::support || type == Activity::Type::handover_down ||
        type == Activity::Type::place_up || type == Activity::Type::press_down) {
        return true; 
    }
    return false;
}

bool ADG::executePolicy(const NodePtr &startNode, NodePtr &endNode) {
    int robot_id = startNode->robotId;
    int act_id = startNode->actId;
    Activity::Type type = act_graph_->get(robot_id, act_id)->type;

    bool success = false;
    log("robot " + std::to_string(robot_id) + " executing policy " + act_graph_->get(robot_id, act_id)->type_string(), LogLevel::INFO);
    executed_steps_[robot_id]->store(startNode->timeStep);
    if (type == Activity::Type::drop_down || type == Activity::Type::pick_down 
        || type == Activity::Type::handover_down || type == Activity::Type::place_up) {
        endNode = intermediate_nodes_[robot_id][act_id * 2 + 5];
        success = policy_->execute(startNode, endNode, type);
    }
    else if (type == Activity::Type::support || type == Activity::Type::press_down) {
        int last_sup_act_id = act_id;
        Activity::Type next_type = act_graph_->get(robot_id, last_sup_act_id + 1)->type;
        while (next_type == type) {
            last_sup_act_id++;
            next_type = act_graph_->get(robot_id, last_sup_act_id + 1)->type;
        }
        endNode = intermediate_nodes_[robot_id][last_sup_act_id * 2 + 1];
        success = policy_->execute(startNode, endNode, type);
    }

    executed_steps_[robot_id]->store(endNode->timeStep + 1);
    executed_acts_[robot_id]->store(endNode->actId + 1);
    int end_act_id = endNode->actId;
    log("robot " + std::to_string(robot_id) + " finished executing policy " 
        + act_graph_->get(robot_id, end_act_id)->type_string(), LogLevel::INFO);

    return success;
}

bool ADG::shiftPolicyNodeType2Edges() {
    for (int i = 0; i < num_robots_; i++) {
        NodePtr policy_start_node = nullptr;
        for (int act_id_i = 0; act_id_i < act_graph_->num_activities(i); act_id_i++) {
            auto act = act_graph_->get(i, act_id_i);
            Activity::Type type = act->type;
            if (type == Activity::Type::drop_down || type == Activity::Type::pick_down ||
                type == Activity::Type::support || type == Activity::Type::handover_down ||
                type == Activity::Type::place_up || type == Activity::Type::press_down) {
                    policy_start_node = intermediate_nodes_[i][act_id_i * 2];
            }
            if (policy_start_node != nullptr) {
                if (type == Activity::Type::drop_down || type == Activity::Type::pick_down ||
                    type == Activity::Type::support || type == Activity::Type::handover_down ||
                    type == Activity::Type::place_up || type == Activity::Type::press_down ||
                    type == Activity::Type::drop_twist || type == Activity::Type::pick_twist ||
                    type == Activity::Type::handover_twist || type == Activity::place_twist ||
                    type == Activity::Type::drop_twist_up || type == Activity::Type::pick_twist_up ||
                    type == Activity::Type::handover_twist_up || type == Activity::Type::place_twist_down ||
                    type == Activity::Type::support_pre || type == Activity::Type::press_up) {
                
                    NodePtr node_i = intermediate_nodes_[i][act_id_i * 2]->Type1Next;
                    NodePtr end_node = intermediate_nodes_[i][act_id_i * 2 + 1];
                    while (node_i->timeStep <= end_node->timeStep) {
                        for (auto edge : node_i->Type2Prev) {
                            edge->nodeTo = policy_start_node;
                            policy_start_node->Type2Prev.push_back(edge);
                            log("Moving type 2 edge -> " + act->type_string() + " " +  std::to_string(node_i->timeStep) + 
                                " to earlier node at timestep " + std::to_string(policy_start_node->timeStep), LogLevel::INFO);
                        }
                        node_i->Type2Prev.clear();
                        node_i = node_i->Type1Next;
                    }
                } else {
                    policy_start_node = nullptr;
                }
            }
        }
    }

    if (hasCycle()) {
        log("cycle detected after shifting policy nodes", LogLevel::ERROR);
        return false;
    }
    return true;
}

void ADG::setExecStartAct(int robot_id, int act_id) {
    exec_start_act_.resize(num_robots_);
    if (robot_id >= exec_start_act_.size()) {
        log("robot id " + std::to_string(robot_id) + " is out of range", LogLevel::ERROR);
    }
    exec_start_act_[robot_id] = act_id;
}

NodePtr ADG::getExecStartNode(int robot_id) const {
    int act_id = exec_start_act_[robot_id];
    if (robot_id >= intermediate_nodes_.size()) {
        log("robot id is out of range for intermediate_nodes_", LogLevel::ERROR);
        return nullptr;
    }
    if ((act_id * 2 + 1) >= intermediate_nodes_[robot_id].size()) {
        log("act id is out of range", LogLevel::ERROR);
        return nullptr;
    }
    return intermediate_nodes_[robot_id][act_id * 2];
}

int ADG::getExecutedAct(int robot_id) const {
    if (executed_acts_.size() == 0) {
        return 0;
    }
    if (robot_id >= executed_acts_.size()) {
        log("robot id " + std::to_string(robot_id) + " is out of range for executed acts", LogLevel::WARN);
        return -1;
    }
    return executed_acts_[robot_id]->load();
}



}