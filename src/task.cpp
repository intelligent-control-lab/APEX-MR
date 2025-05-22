#include "task.h"
#include "logger.h"
#include <fstream>
#include <queue>

const std::map<Activity::Type, std::string> Activity::enumStringMap = {
    {Activity::Type::home, "home"},
    {Activity::Type::pick_tilt_up, "pick_tilt_up"},
    {Activity::Type::pick_up, "pick_up"},
    {Activity::Type::pick_down, "pick_down"},
    {Activity::Type::pick_twist, "pick_twist"},
    {Activity::Type::pick_twist_up, "pick_twist_up"},
    {Activity::Type::drop_tilt_up, "drop_tilt_up"},
    {Activity::Type::drop_up, "drop_up"},
    {Activity::Type::drop_down, "drop_down"},
    {Activity::Type::drop_twist, "drop_twist"},
    {Activity::Type::drop_twist_up, "drop_twist_up"},
    {Activity::Type::support, "support"},
    {Activity::Type::support_pre, "support_pre"},
    {Activity::Type::pick, "pick"},
    {Activity::Type::drop, "drop"},
    {Activity::Type::open_gripper, "open_gripper"},
    {Activity::Type::close_gripper, "close_gripper"},
    {Activity::Type::home_receive, "home_receive"},
    {Activity::Type::receive, "receive"},
    {Activity::Type::home_handover, "home_handover"},
    {Activity::Type::handover_up, "handover_up"},
    {Activity::Type::handover_down, "handover_down"},
    {Activity::Type::handover_twist, "handover_twist"},
    {Activity::Type::handover_twist_up, "handover_twist_up"},
    {Activity::Type::place_tilt_down_pre, "place_tilt_down_pre"},
    {Activity::Type::place_tilt_down, "place_tilt_down"},
    {Activity::Type::place_down, "place_down"},
    {Activity::Type::place_up, "place_up"},
    {Activity::Type::place_twist, "place_twist"},
    {Activity::Type::place_twist_down, "place_twist_down"},
    {Activity::Type::receive_place, "receive_place"},
    {Activity::Type::press_up, "press_up"},
    {Activity::Type::press_down, "press_down"},

};

ActivityGraph::ActivityGraph(int num_robots) {
    num_robots_ = num_robots;
    activities_.resize(num_robots);

}

ActivityGraph::ActivityGraph(const ActivityGraph &other, int first_n_tasks) {
    num_robots_ = other.num_robots_;
    activities_.resize(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < first_n_tasks; j++) {
            activities_[i].push_back(other.activities_[i][j]);
        }
    }
    for (auto obj : other.obj_nodes_) {
        if (obj->next_attach == nullptr && obj->prev_detach == nullptr) {
            obj_nodes_.push_back(obj);
        }
        if (obj->next_attach != nullptr) {
            auto act = obj->next_attach;
            if (act->act_id < first_n_tasks) {
                obj_nodes_.push_back(obj);
            }
        }
        else if (obj->prev_detach != nullptr) {
            auto act = obj->prev_detach;
            if (act->act_id < first_n_tasks) {
                obj_nodes_.push_back(obj);
            }
        } 
    }
}

ActPtr ActivityGraph::add_act(int robot_id, Activity::Type type) {
    assert(robot_id < num_robots_);

    ActPtr activity = std::make_shared<Activity>(robot_id, type);
    activity->act_id = activities_[robot_id].size();
    
    activities_[robot_id].push_back(activity);

    if (activity->act_id > 0) {
        auto prev_act = activities_[robot_id][activity->act_id - 1];
        prev_act->type1_next = activity;
        activity->type1_prev = prev_act;
    }
    return activity;
}

ActPtr ActivityGraph::add_act(int robot_id, Activity::Type type, ActPtr type2_dep) {
    assert(robot_id < num_robots_ && type2_dep != nullptr);
    ActPtr activity = std::make_shared<Activity>(robot_id, type);
    activity->act_id = activities_[robot_id].size();
    add_type2_dep(activity, type2_dep);
    activities_[robot_id].push_back(activity);

    if (activity->act_id > 0) {
        auto prev_act = activities_[robot_id][activity->act_id - 1];
        prev_act->type1_next = activity;
        activity->type1_prev = prev_act;
    }
    return activity;
}

void ActivityGraph::remove_act(int robot_id, int act_id) {
    // check if this act has type2 dependencies
    auto act = activities_[robot_id][act_id];
    if (act->type2_next.size() > 0 || act->type2_prev.size() > 0) {
        log("cannot remove activity " + act->type_string() + " with type2 dependencies", LogLevel::ERROR);
        return;
    }
    if (act->collision_nodes.size() > 0) {
        log("cannot remove activity " + act->type_string() + " with collision nodes", LogLevel::ERROR);
        return;
    }
    
    // change the type1_next and type1_prev of the surrounding activities
    if (robot_id < num_robots_ && act_id < activities_[robot_id].size()) {
        if (act_id > 0) {
            auto prev_act = activities_[robot_id][act_id - 1];
            if (act_id < activities_[robot_id].size() - 1) {
                auto next_act = activities_[robot_id][act_id + 1];
                prev_act->type1_next = next_act;
                next_act->type1_prev = prev_act;
            }
            else {
                prev_act->type1_next = nullptr;
            }
        }
        else if (act_id < activities_[robot_id].size() - 1) {
            auto next_act = activities_[robot_id][act_id + 1];
            next_act->type1_prev = nullptr;
        }
        activities_[robot_id].erase(activities_[robot_id].begin() + act_id);
    }

    // change the act_id of the following activities
    for (int i = act_id; i < activities_[robot_id].size(); i++) {
        activities_[robot_id][i]->act_id = i;
    }
}

/* add a static object to the scene (no attached parent)*/
ObjPtr ActivityGraph::add_obj(const Object &obj) {
    ObjPtr obj_node = std::make_shared<ObjectNode>(obj, obj_nodes_.size());
    obj_nodes_.push_back(obj_node);
    return obj_node;
}

/* set the object node to be attached to a robot at the onset of selected activity */
void ActivityGraph::attach_obj(ObjPtr obj, const std::string &link_name, ActPtr act) {
    obj->next_attach = act;
    obj->next_attach_link = link_name;
    act->obj_attached.push_back(obj);
}

/* set the object node to be detached from a robot at the onset of selected activity */
void ActivityGraph::detach_obj(ObjPtr obj, ActPtr act) {
    obj->prev_detach = act;
    act->obj_detached.push_back(obj);
}

void ActivityGraph::add_type2_dep(ActPtr act, ActPtr dep) {
    if (act->robot_id != dep->robot_id) {
        act->add_type2_dep(dep);
        dep->add_type2_next(act);
    }
    else {
        log("type2 dependency is not allowed within the same robot", LogLevel::WARN);
    }
}


/* enable or disable collision checking between the object_node and the robot at the onset of selected activity */
void ActivityGraph::set_collision(const std::string &obj_name, const std::string &name, ActPtr act, bool allow) {
    SetCollisionNode node(obj_name, name, allow);
    act->collision_nodes.push_back(node);
}

ActPtr ActivityGraph::get(int robot_id, int act_id) {
    if (robot_id >= num_robots_ || act_id >= activities_[robot_id].size()) {
        return nullptr;
    }
    return activities_[robot_id][act_id];
}

std::shared_ptr<const Activity> ActivityGraph::get(int robot_id, int act_id) const {
    if (robot_id >= num_robots_ || act_id >= activities_[robot_id].size()) {
        return nullptr;
    }
    return activities_[robot_id][act_id];
}

ActPtr ActivityGraph::get_last_act(int robot_id) {
    if (robot_id < num_robots_) {
        return activities_[robot_id].back();
    }
    return nullptr;
}

ActPtr ActivityGraph::get_last_act(int robot_id, Activity::Type type) {
    if (robot_id < num_robots_) {
        for (int i = activities_[robot_id].size() - 1; i >= 0; i--) {
            if (activities_[robot_id][i]->type == type) {
                return activities_[robot_id][i];
            }
        }
    }
    return nullptr;
}

ObjPtr ActivityGraph::get_last_obj(const std::string &obj_name) {
    for (int i = obj_nodes_.size() - 1; i >= 0; i--) {
        if (obj_nodes_[i]->obj.name == obj_name) {
            return obj_nodes_[i];
        }
    }
    return nullptr;
}

std::vector<ObjPtr> ActivityGraph::get_start_obj_nodes() const {
    std::vector<ObjPtr> start_obj;
    for (auto obj : obj_nodes_) {
        if (obj->prev_detach == nullptr && !obj->vanish) {
            start_obj.push_back(obj);
        }
    }
    return start_obj;
}

std::vector<ObjPtr> ActivityGraph::get_end_obj_nodes() const {
    std::vector<ObjPtr> end_obj;
    for (auto obj : obj_nodes_) {
        if (obj->next_attach == nullptr && !obj->vanish) {
            end_obj.push_back(obj);
        }
    }
    return end_obj;
}

void ActivityGraph::minimizeWait(std::shared_ptr<PlanInstance> instance) {
    // minimize the wait time between activities by moving forward activities if possible
    
    std::vector<std::vector<ActPtr>> col_act(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        col_act[i].resize(activities_[i].size(), nullptr);
    }

    for (int i = 0; i < num_robots_; i++) {
        int l_min = 0;
        for (int j = 0; j < activities_[i].size(); j++) {
            ActPtr act = activities_[i][j];
            RobotPose end_pose = act->end_pose;
            for (int k = 0; k < num_robots_; k++) {
                if (k != i) {
                    int l_max = std::min((int)activities_[k].size(), j);
                    for (int l = l_max - 1; l >= l_min; l--) {
                        ActPtr other_act = activities_[k][l];
                        if (instance->checkCollision({end_pose, other_act->end_pose}, true) == true) {
                            col_act[i][j] = other_act;
                            l_min = l + 1;
                            log("collision detected between robot " + std::to_string(i) + " activity " + std::to_string(j) + " type " + act->type_string() +
                             " and robot " + std::to_string(k) + " activity " + std::to_string(l) + " type " + other_act->type_string(), LogLevel::INFO);
                            break;
                        }
                    }
                }
            }
        }
    }

    std::vector<ActPtr> start_act(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        start_act[i] = activities_[i][0];
    }

    int t = 0;
    bool all_reached = false;
    std::vector<int> at(num_robots_, 0);
    std::vector<int> at_prev(num_robots_, 0);
    
    while (!all_reached) {
        for (int i = 0; i < num_robots_; i++) {
            if (at[i] == activities_[i].size()) {
                continue;
            }
            ActPtr act_i = activities_[i][at[i]]; // current act
            ActPtr col_act_i = col_act[i][at[i]]; // any potential collision act
            
            // compute if this act can be skipeed
            double dist = instance->computeDistance(act_i->start_pose, act_i->end_pose);
            bool skip = act_i->is_skippable() || (act_i->type == Activity::Type::support) ||  (act_i->type == Activity::Type::receive);
            skip &= (act_i->type2_next.size() == 0);
            skip &= (dist < 1e-5);
            skip &= (act_i->collision_nodes.size() == 0);
            if (col_act_i != nullptr) {
                skip &= at_prev[col_act_i->robot_id] > col_act_i->act_id;
            }
            for (const auto act : act_i->type2_prev) {
                skip &= (at_prev[act->robot_id] > act->act_id);
            }

            while (skip) {
                // skip the act that is stationary
                if (act_i->type1_prev != nullptr) {
                    act_i->type1_prev->type1_next = act_i->type1_next;
                }
                else {
                    start_act[i] = act_i->type1_next;
                }
                if (act_i->type1_next != nullptr) {
                    act_i->type1_next->type1_prev = act_i->type1_prev;
                }
                act_i = act_i->type1_next;
                at[i]++;

                if (at[i] == activities_[i].size()) {
                    break;
                }
                
                // compute the next act can be skipped
                col_act_i = col_act[i][at[i]];
                dist = instance->computeDistance(act_i->start_pose, act_i->end_pose);

                skip = act_i->is_skippable() || (act_i->type == Activity::Type::support) ||  (act_i->type == Activity::Type::receive);
                skip &= (act_i->type2_next.size() == 0);
                skip &= (dist < 1e-5);
                skip &= (act_i->collision_nodes.size() == 0);
                if (col_act_i != nullptr) {
                    skip &= at_prev[col_act_i->robot_id] > col_act_i->act_id;
                }
                for (const auto act : act_i->type2_prev) {
                    skip &= (at_prev[act->robot_id] > act->act_id);
                }
    
            }

            // check if we can advance to the next activity
            if (at[i] < activities_[i].size()) {
                act_i = activities_[i][at[i]];
                bool advance = true;
                col_act_i = col_act[i][at[i]];
                if (col_act_i != nullptr) {
                    advance &= at_prev[col_act_i->robot_id] > col_act_i->act_id;
                }
                for (const auto act : act_i->type2_prev) {
                    advance &= (at_prev[act->robot_id] > act->act_id);
                }

                if (advance) {
                    at[i]++;
                }
                else {
                    // wait at the current activitiy
                    ActPtr last_act = act_i->type1_prev;
                    ActPtr act_i_copy = std::make_shared<Activity>(last_act->robot_id, last_act->type);
                    last_act->type1_next = act_i_copy;
                    act_i_copy->type1_prev = last_act;
                    act_i_copy->type1_next = act_i;
                    act_i->type1_prev = act_i_copy;
                    act_i_copy->start_pose = act_i->start_pose;
                    act_i_copy->end_pose = act_i->start_pose;
                }
            }
        }

        // udpate at_prev and compute if while loop ends
        all_reached = true;
        for (int i = 0; i < num_robots_; i++) {
            at_prev[i] = at[i];
            if (at[i] < activities_[i].size()) {
                all_reached = false;
            }
        }
        t++;
    }
    
    // relabel the index
    for (int i = 0; i < num_robots_; i++) {
        activities_[i].clear();
        ActPtr act = start_act[i];
        while (act != nullptr) {
            activities_[i].push_back(act);
            act->act_id = activities_[i].size() - 1;
            act = act->type1_next;
        }
    }


}



bool ActivityGraph::saveGraphToFile(const std::string &filename) const {
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
        for (int j = 0; j < activities_[i].size(); j++) {
            ActPtr act = activities_[i][j];
            out << "a" << i << "_" << act->act_id << " [label=\"" << act->type_string() << "\"];" << std::endl;
        }
        // activity type-1 edges
        ActPtr act = activities_[i][0];
        out << "a" << i << "_" << act->act_id;
        for (int j = 1; j < activities_[i].size(); j++) {
            ActPtr act = activities_[i][j];
            out << " -> " << "a" << i << "_" << act->act_id;
        }

        out << ";" << std::endl;
        out << "}" << std::endl;
    }

    // define type2 edges
    for (int i = 0; i < num_robots_; i++) {
        for (int act_id = 0; act_id < activities_[i].size(); act_id++) {
            auto act = activities_[i][act_id];
            for (auto dep : act->type2_prev) {
                out << "a" << dep->robot_id << "_" << dep->act_id << " -> " << "a" << act->robot_id << "_" << act->act_id << ";" << std::endl;
            }
        }
    }

    // define object nodes
    for (int i = 0; i < obj_nodes_.size(); i++) {
        out << "o" << i << " [label=\"" << obj_nodes_[i]->obj.name << "\"];" << std::endl;
        if (obj_nodes_[i]->prev_detach != nullptr) {
            out << "a" << obj_nodes_[i]->prev_detach->robot_id << "_" << obj_nodes_[i]->prev_detach->act_id << " -> o" << i << ";" << std::endl;
        }
        if (obj_nodes_[i]->next_attach != nullptr) {
            out << "o" << i << " -> a" << obj_nodes_[i]->next_attach->robot_id << "_" << obj_nodes_[i]->next_attach->act_id << ";" << std::endl;
        }
    }
    

    out << "}" << std::endl;
    out.close();

    std::string command = "dot -Tpng " + filename + " -o " + filename + ".png";
    int result = system(command.c_str());

    return result == 0;

}

bool ActivityGraph::bfs(ActPtr act_i, std::vector<std::vector<bool>> &visited, bool forward) const
{
    // BFS function to find all the dependent Activitys of act_i
    std::queue<ActPtr> q;
    q.push(act_i);
    visited[act_i->robot_id][act_i->act_id] = true;
    
    while (!q.empty()) {
        ActPtr act = q.front();
        q.pop();
        if (forward) {
            if (act->type1_next != nullptr && !visited[act->type1_next->robot_id][act->type1_next->act_id]) {
                q.push(act->type1_next);
                visited[act->type1_next->robot_id][act->type1_next->act_id] = true;
            }
            
            for (auto dep_act : act->type2_next) {
                if (!visited[dep_act->robot_id][dep_act->act_id]) {
                    q.push(dep_act);
                    visited[dep_act->robot_id][dep_act->act_id] = true;
                }
            }
        } 
        else {
            if (act->type1_prev != nullptr && !visited[act->type1_prev->robot_id][act->type1_prev->act_id]) {
                q.push(act->type1_prev);
                visited[act->type1_prev->robot_id][act->type1_prev->act_id] = true;
            }
            
            for (auto dep_act : act->type2_prev) {
                if (!visited[dep_act->robot_id][dep_act->act_id]) {
                    q.push(dep_act);
                    visited[dep_act->robot_id][dep_act->act_id] = true;
                }
            }
        }
    }
    return true;
}

std::vector<ObjPtr> ActivityGraph::find_indep_obj(ActPtr act) const {
    std::vector<bool> dependent(obj_nodes_.size(), false);

    std::vector<std::vector<bool>> visited(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        visited[i].resize(activities_[i].size(), false);
    }
    
    // do forward bfs search and mark any dependnet object 
    bfs(act, visited, true);
    for (auto obj : obj_nodes_) {
        if (obj->prev_detach != nullptr && visited[obj->prev_detach->robot_id][obj->prev_detach->act_id]) {
            dependent[obj->obj_node_id] = true;
        }
    }

    for (int i = 0; i < num_robots_; i++) {
        visited[i].assign(activities_[i].size(), false);
    }

    // do backward bfs search and mark any dependnet object
    bfs(act, visited, false);
    for (auto obj : obj_nodes_) {
        if (obj->next_attach != nullptr && visited[obj->next_attach->robot_id][obj->next_attach->act_id]) {
            dependent[obj->obj_node_id] = true;
        }
    }

    std::vector<ObjPtr> indep_obj;
    for (int i = 0; i < obj_nodes_.size(); i++) {
        // exclude object in the handover positions
        if (obj_nodes_[i]->next_attach != nullptr && obj_nodes_[i]->prev_detach != nullptr) {
            continue;
        }
        if (!dependent[i] && !obj_nodes_[i]->vanish) {
            indep_obj.push_back(obj_nodes_[i]);
        }
    }
        
    return indep_obj;

}

void concatSyncSolution(std::shared_ptr<PlanInstance> instance,
                        std::shared_ptr<ActivityGraph> act_graph,
                        const std::vector<MRTrajectory> &solutions,
                        double dt,
                        MRTrajectory &sync_solution)
{
    int num_robots = instance->getNumberOfRobots();
    sync_solution.clear();
    sync_solution.resize(num_robots);

    std::vector<double> sol_spans;
    for (int sol_id = 0; sol_id < solutions.size(); sol_id++) {
        double makespan = 0;
        if (num_robots != solutions[sol_id].size()) {
            log("number of robots in the solution does not match the number of robots in the instance", LogLevel::ERROR);
        }
        for (int i = 0; i < num_robots; i++) {
            makespan = std::max(makespan, solutions[sol_id][i].times.back());
        }
        sol_spans.push_back(makespan);
    }
    
    for (int i = 0; i < num_robots; i++) {
        sync_solution[i].robot_id = i;
        sync_solution[i].act_ids = std::vector<int>();
        sync_solution[i].trajectory = std::vector<RobotPose>();
        sync_solution[i].times = std::vector<double>();

        
        int sol_id = 0;
        double start_t = 0;
        for (int act_id = 0; act_id < act_graph->num_activities(i); act_id++) {
            auto act = act_graph->get(i, act_id);
            if (act->type == Activity::Type::open_gripper || act->type == Activity::Type::close_gripper) {
                if (act_id == 0) {
                    log("gripper open/close task should not be the first task", LogLevel::ERROR);
                }
                RobotPose gripper_pose = sync_solution[i].trajectory.back();
                gripper_pose.hand_values = act->end_pose.hand_values;;

                sync_solution[i].trajectory.push_back(gripper_pose);
                sync_solution[i].times.push_back(start_t);
                sync_solution[i].act_ids.push_back(act_id);
                sync_solution[i].cost = sync_solution[i].times.back();
                start_t += dt;
            }
            else {
                if (sol_id >= solutions.size()) {
                    log("solution vector does not have enough tasks for robot " + std::to_string(i), LogLevel::ERROR);
                }
                // append solutions[act_id][i].trajectory to the end of sync_solution[i].trajectory
                for (int j = 0; j < solutions[sol_id][i].trajectory.size(); j++) {
                    sync_solution[i].trajectory.push_back(solutions[sol_id][i].trajectory[j]);
                    sync_solution[i].times.push_back(solutions[sol_id][i].times[j] + start_t);
                    sync_solution[i].act_ids.push_back(act_id);
                }
                sync_solution[i].cost = sync_solution[i].times.back();
                start_t += (sol_spans[sol_id] + dt);
                sol_id ++;
            }
        }
    }
}