#ifndef APEX_MR_TASK_H
#define APEX_MR_TASK_H

#include "instance.h"
#include "planner.h"

class ObjectNode;
class SetCollisionNode;
typedef std::shared_ptr<ObjectNode> ObjPtr;

class Activity {
public:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & robot_id;
        ar & act_id;
        ar & type;
        ar & type2_prev;
        ar & type2_next;
        ar & type1_prev;
        ar & type1_next;
        ar & start_pose;
        ar & end_pose;
        ar & obj_detached;
        ar & obj_attached;
        ar & collision_nodes;
    }

    enum Type {
        home = 0,
        pick_tilt_up = 1, // lego
        pick_up = 2, // lego
        pick_down = 3, // lego
        pick_twist = 4, // lego
        pick_twist_up = 5, // lego
        drop_tilt_up = 7, // lego
        drop_up = 8, // lego
        drop_down = 9, // lego
        drop_twist = 10, // lego
        drop_twist_up = 11, // lego
        support = 13, // lego
        support_pre = 14, // lego
        pick = 15,
        drop = 16,
        open_gripper = 17,
        close_gripper = 18,
        home_receive = 19, // lego
        receive = 20, // lego
        home_handover = 21, // lego
        handover_up = 22, // lego
        handover_down = 23, // lego
        handover_twist = 24, // lego
        handover_twist_up = 25, // lego
        place_tilt_down_pre = 26, // lego
        place_tilt_down = 27, // lego
        place_down = 28, // lego
        place_up = 29, // lego
        place_twist = 30, // lego
        place_twist_down = 31, // lego
        receive_place = 32, // lego
        press_up = 33, // lego
        press_down = 34, // lego
    };

    static const std::map<Type, std::string> enumStringMap;

    Activity() = default;
    Activity(int robot_id, Type type) : robot_id(robot_id), type(type) {}
    void add_type2_dep(std::shared_ptr<Activity> type2_dep) {
        this->type2_prev.push_back(type2_dep);
    }
    void add_type2_next(std::shared_ptr<Activity> type2_next) {
        this->type2_next.push_back(type2_next);
    }

    std::string type_string() const {
        return enumStringMap.at(type);
    }

    bool is_skippable() const {
        return type == Type::home || type == Type::home_handover || type == Type::home_receive;
    }

    int robot_id;
    int act_id;
    Type type;
    std::vector<std::shared_ptr<Activity>> type2_prev;
    std::vector<std::shared_ptr<Activity>> type2_next;
    std::shared_ptr<Activity> type1_prev;
    std::shared_ptr<Activity> type1_next;
    RobotPose start_pose;
    RobotPose end_pose;
    std::vector<ObjPtr> obj_detached;
    std::vector<ObjPtr> obj_attached;
    std::vector<SetCollisionNode> collision_nodes;
};
typedef std::shared_ptr<Activity> ActPtr;


class ObjectNode {
public:
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & obj;
        ar & obj_node_id;
        ar & prev_detach;
        ar & next_attach;
        ar & next_attach_link;
        ar & vanish;
        ar & handover;
    }

    ObjectNode() = default;
    ObjectNode(const Object &obj, int id) : obj(obj), obj_node_id(id) {}

    std::string name() const {
        return obj.name;
    }

    Object obj;
    int obj_node_id;
    std::string next_attach_link;
    bool vanish = false; // vanish before attach or after detach
    bool handover = false; // used as a handover node
    std::shared_ptr<Activity> prev_detach;
    std::shared_ptr<Activity> next_attach;
};

class SetCollisionNode {
public:
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & obj_name;
        ar & link_name;
        ar & allow;
    }

    SetCollisionNode() = default;
    SetCollisionNode(const std::string &obj_name, const std::string &link_name, bool allow) 
        : obj_name(obj_name), link_name(link_name), allow(allow) {}
    std::string obj_name;
    std::string link_name;
    bool allow;
};


class ActivityGraph {
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & num_robots_;
        ar & activities_;
        ar & obj_nodes_;
    }
public:
    ActivityGraph() = default;
    ActivityGraph(int num_robots);

    ActivityGraph(const ActivityGraph &other, int first_n_tasks);
    
    ActPtr add_act(int robot_id, Activity::Type type);

    ActPtr add_act(int robot_id, Activity::Type type, ActPtr type2_dep);
    
    /* add a static object to the scene (no attached parent)*/
    ObjPtr add_obj(const Object &obj);

    /* set the object node to be attached to a robot at the onset of selected activity */
    void attach_obj(ObjPtr obj, const std::string &link_name, ActPtr act);

    /* set the object node to be detached from a robot at the onset of selected activity */
    void detach_obj(ObjPtr obj, ActPtr act);

    void add_type2_dep(ActPtr act, ActPtr dep);

    /* enable or disable collision checking between the object_node and the robot at the onset of selected activity */
    void set_collision(const std::string &obj_name, const std::string &link_name, ActPtr act, bool allow);

    void minimizeWait(std::shared_ptr<PlanInstance> instance);

    bool saveGraphToFile(const std::string &filename) const;

    ActPtr get(int robot_id, int act_id);
    std::shared_ptr<const Activity> get(int robot_id, int act_id) const;
    ActPtr get_last_act(int robot_id);
    ActPtr get_last_act(int robot_id, Activity::Type type);
    ObjPtr get_last_obj(const std::string &obj_name);

    int num_activities(int robot_id) const {
        return activities_[robot_id].size();
    }

    int num_robots() const {
        return num_robots_;
    }
    
    std::vector<ObjPtr> get_obj_nodes() const {
        return obj_nodes_;
    }

    std::vector<ObjPtr> get_start_obj_nodes() const;
    std::vector<ObjPtr> get_end_obj_nodes() const;

    bool bfs(ActPtr act_i, std::vector<std::vector<bool>> &visited, bool forward) const;

    std::vector<ObjPtr> find_indep_obj(ActPtr act) const;

    void remove_act(int robot_id, int act_id);

private:
    int num_robots_ = 0;
    std::vector<std::vector<ActPtr>> activities_;
    std::vector<ObjPtr> obj_nodes_;
    
};


void concatSyncSolution(std::shared_ptr<PlanInstance> instance,
                        std::shared_ptr<ActivityGraph> act_graph,
                        const std::vector<MRTrajectory> &solutions,
                        double dt,
                        MRTrajectory &sync_solution);

#endif // APEX_MR_TASK_H