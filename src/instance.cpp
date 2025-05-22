#include <instance.h>
#include <logger.h>

MoveitInstance::MoveitInstance(robot_state::RobotStatePtr kinematic_state,
                               const std::string &joint_group_name,
                               planning_scene::PlanningScenePtr planning_scene)
    : kinematic_state_(kinematic_state), joint_group_name_(joint_group_name), planning_scene_(planning_scene)
{
    planning_scene_->getPlanningSceneMsg(original_scene_);
    //planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), true);
}

void PlanInstance::setNumberOfRobots(int num_robots) {
    num_robots_ = num_robots;
    start_poses_.resize(num_robots);
    goal_poses_.resize(num_robots);
    robot_dof_.resize(num_robots);
    hand_dof_.resize(num_robots, 0);
}

void PlanInstance::setStartPose(int robot_id, const std::vector<double> &pose) {
    start_poses_[robot_id].robot_id = robot_id;
    start_poses_[robot_id].robot_name = robot_names_[robot_id];
    start_poses_[robot_id].joint_values = pose;
}

void PlanInstance::setGoalPose(int robot_id, const std::vector<double> &pose) {
    goal_poses_[robot_id].robot_id = robot_id;
    goal_poses_[robot_id].robot_name = robot_names_[robot_id];
    goal_poses_[robot_id].joint_values = pose;
}

void PlanInstance::setRobotDOF(int robot_id, size_t dof) {
    if (robot_id >= robot_dof_.size()) {
        robot_dof_.resize(robot_id + 1);
    }
    robot_dof_[robot_id] = dof;
}

void PlanInstance::setHandDof(int robot_id, size_t dof) {
    if (robot_id >= hand_dof_.size()) {
        hand_dof_.resize(robot_id + 1);
    }
    hand_dof_[robot_id] = dof;
}


size_t PlanInstance::getRobotDOF(int robot_id) const {
    return robot_dof_[robot_id];
}

size_t PlanInstance::getHandDOF(int robot_id) const {
    return hand_dof_[robot_id];
}

RobotPose PlanInstance::initRobotPose(int robot_id) const {
    RobotPose pose;
    pose.robot_id = robot_id;
    pose.robot_name = robot_names_[robot_id];
    pose.joint_values.resize(robot_dof_[robot_id]);
    pose.hand_values.resize(hand_dof_[robot_id], 0);
    return pose;
}

double PlanInstance::getVMax(int robot_id) {
    return v_max_;
}

void PlanInstance::setVmax(double vmax) {
    v_max_ = vmax;
}

int PlanInstance::numCollisionChecks() {
    int ans = num_collision_checks_;
    num_collision_checks_ = 0;
    return ans;
}

void MoveitInstance::setPadding(double padding) {
    planning_scene_->getCollisionEnvNonConst()->setPadding(padding);
    planning_scene_->propogateRobotPadding();
}

bool MoveitInstance::checkCollision(const std::vector<RobotPose> &poses, bool self, bool debug) {
    /* check if there is robot-robot or scene collision for a set of poses for some robots*/
    /* true if has collision, false if no collision*/
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = joint_group_name_;
    if (debug) {
        c_req.contacts = true;
        c_req.max_contacts = 10;
    }

    // set the robot state to the one we are checking
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    
    
    std::vector<double> all_joints;
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrixNonConst();

    //print the acm entry names
    // std::vector<std::string> acm_names;
    // acm.getAllEntryNames(acm_names);
    // for (const auto &entry : acm_names) {
    //     std::cout << entry << " ";
    // }
    // std::cout << std::endl;

    int index = 0;
    for (int i = 0; i < num_robots_; i++) {
        std::string group = robot_names_[i];

        // find if this robot is in collision with the environment
        bool checking_i = false;
        RobotPose pose;
        for (int j = 0; j < poses.size(); j++) {
            if (poses[j].robot_id == i) {
                checking_i = true;
                pose = poses[j];
                break;
            }
        }

        // set the acm for this robot to true if it is not checked for collision
        if (!checking_i) {
            auto links = kinematic_state_->getJointModelGroup(group)->getLinkModelNamesWithCollisionGeometry();
            for (const auto &link : links) {
                acm.setEntry(link, true);
            }
            if (hand_names_.size() > i) {
                auto hand_links = kinematic_state_->getJointModelGroup(hand_names_[i])->getLinkModelNamesWithCollisionGeometry();
                for (const auto &link : hand_links) {
                    acm.setEntry(link, true);
                }
            }
            // insert the joint values for this robot
            std::vector<double> dummy_values(getRobotDOF(i), 0.0);
            all_joints.insert(all_joints.end(), dummy_values.begin(), dummy_values.end());
        }
        else {
            auto links = kinematic_state_->getJointModelGroup(group)->getLinkModelNamesWithCollisionGeometry();
            // copy the joint values for this robot
            all_joints.insert(all_joints.end(), pose.joint_values.begin(), pose.joint_values.end());
        }
        
        index += start_poses_[i].joint_values.size();
    }

    robot_state.setJointGroupPositions(joint_group_name_, all_joints);

    c_res.clear();
    if (self) {
        //robot_state.updateCollisionBodyTransforms();
        //planning_scene_->getCollisionEnv()->checkSelfCollision(c_req, c_res, robot_state, acm);
        planning_scene_->checkSelfCollision(c_req, c_res, robot_state, acm);  
    } else {
        planning_scene_->checkCollision(c_req, c_res, robot_state, acm);
    }
    num_collision_checks_++;

    if (debug) {
        std::cout << "Number of contacts: " << c_res.contacts.size() << std::endl;
        for (const auto &contact : c_res.contacts) {
            std::cout << "Contact between " << contact.first.first << " and " << contact.first.second << std::endl;
        }
    }

    return c_res.collision;
}

double MoveitInstance::computeDistance(const RobotPose& a, const RobotPose &b) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);

    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);
    double distance = robot_state_a.distance(robot_state_b);
    return distance;
}

double MoveitInstance::computeDistance(const RobotPose& a, const RobotPose &b, int dim) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    assert(dim <= a.joint_values.size() && dim <= b.joint_values.size());
    std::string joint_name = kinematic_state_->getJointModelGroup(a.robot_name)->getActiveJointModelNames()[dim];
    const moveit::core::JointModel* joint = kinematic_state_->getJointModel(joint_name);
    double a_d = a.joint_values[dim];
    double b_d = b.joint_values[dim];
    double distance = joint->distance(&a_d, &b_d);
    return distance;
}

RobotPose MoveitInstance::interpolate(const RobotPose &a, const RobotPose&b, double t) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);

    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);

    moveit::core::RobotState res_state = planning_scene_->getCurrentStateNonConst();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    robot_state_a.interpolate(robot_state_b, t, res_state, joint_model_group);

    RobotPose res = initRobotPose(a.robot_id);
    res_state.copyJointGroupPositions(a.robot_name, res.joint_values);
    res.hand_values = a.hand_values;
    return res;
}

double MoveitInstance::interpolate(const RobotPose &a, const RobotPose&b, double t, int dim) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    assert(dim <= a.joint_values.size() && dim <= b.joint_values.size());

    std::string joint_name = kinematic_state_->getJointModelGroup(a.robot_name)->getActiveJointModelNames()[dim];
    const moveit::core::JointModel* joint = kinematic_state_->getJointModel(joint_name);
    double a_d = a.joint_values[dim];
    double b_d = b.joint_values[dim];
    double res;
    joint->interpolate(&a_d, &b_d, t, &res);
    return res;
}

bool MoveitInstance::connect(const RobotPose& a, const RobotPose& b, double col_step_size, bool debug) {
    /* check if a collision-free kinematic path exists from pose a to b for the robot (ignoring other robots)*/
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    
    // discretize and check for collision along the path
    double joint_distance = computeDistance(a, b);
    int num_steps = std::ceil(joint_distance / col_step_size);
    if (num_steps == 0) {
        log("Connecting two poses with zero distance", LogLevel::WARN);
        return false;
    }

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = a.robot_name;
    if (debug) {
        c_req.contacts = true;
        c_req.max_contacts = 10;
    }

    auto acm = planning_scene_->getAllowedCollisionMatrixNonConst();
    for (int i = 0; i < num_robots_; i++) {
        if (i != a.robot_id) {
            auto links = kinematic_state_->getJointModelGroup(robot_names_[i])->getLinkModelNamesWithCollisionGeometry();
            for (const auto &link : links) {
                acm.setEntry(link, true);
            }
            if (hand_names_.size() > i) {
                links = kinematic_state_->getJointModelGroup(hand_names_[i])->getLinkModelNamesWithCollisionGeometry();
                for (const auto &link : links) {
                    acm.setEntry(link, true);
                }
            }
        }
    }

    auto joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);

    for (int i = 0; i <= num_steps; i++) {
        c_res.clear();
        
        robot_state.setJointGroupPositions(a.robot_name, a.joint_values);
        robot_state_a.interpolate(robot_state_b, (double)i / num_steps, robot_state, joint_model_group);
        planning_scene_->checkCollision(c_req, c_res, robot_state, acm);
        num_collision_checks_++;
        if (c_res.collision) {
            if (debug) {
                for (const auto &contact : c_res.contacts) {
                    std::cout << "Collision detected at step " << i << " between ";
                    std::cout << contact.first.first << " and " << contact.first.second << std::endl;
                    RobotPose c = a;
                    robot_state.copyJointGroupPositions(c.robot_name, c.joint_values);
                    moveRobot(c.robot_id, c);
                    updateScene();
                    log("updated scene when connect collides", LogLevel::DEBUG);
                }
            }
            return false;
        }
    }

    return true;
}

bool MoveitInstance::steer(const RobotPose& a, const RobotPose& b, double max_dist, RobotPose& result, double col_step_size) {
    /* find a collision-free that steers the robot from a towards b for max_distance */
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    
    double joint_distance = computeDistance(a, b);
    if (joint_distance <= max_dist) {
        result = b;
        return true;
    }

    auto joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();

    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);
    robot_state_a.interpolate(robot_state_b, max_dist / joint_distance, robot_state, joint_model_group);
    result.robot_id = a.robot_id;
    result.robot_name = a.robot_name;
    result.joint_values.resize(a.joint_values.size());
    robot_state.copyJointGroupPositions(a.robot_name, result.joint_values);

    if (connect(a, result, col_step_size)) {
        return true;
    }
    else {
        return false;
    }
}

bool MoveitInstance::sample(RobotPose &pose) {
    /* sample a collision free pose for the robot (ignoring other robots)
    */

    // initialize the joint values vector
    std::string robot_name = robot_names_[pose.robot_id];
    pose.robot_name = robot_name;

    // get the bounds of the joint space for the robot
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(robot_name);
    const std::vector<const moveit::core::JointModel*> & joint_models = joint_model_group->getActiveJointModels();
    
    // boilerplate for checking collision
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = robot_name;

    bool in_collision = true;
    int attempt = 0;
    int max_attempts = 10;
    do {
        std::vector<double> joint_values;
        joint_values.reserve(joint_models.size());

        // sample each joint
        for (int i = 0; i < joint_models.size(); i++) {
            const auto &bounds = joint_models[i]->getVariableBounds();
            if (!bounds.empty()) {
                // assume the joint has only one variable
                std::uniform_real_distribution<double> distribution(bounds[0].min_position_, bounds[0].max_position_);
                joint_values.push_back(distribution(rng_));
            }
            else {
                // raise an error if the joint has no bounds
                throw std::runtime_error("Joint " + joint_models[i]->getName() + " has no bounds");
            }
        }

        // check collision
        c_res.clear();
        robot_state.setJointGroupPositions(robot_name, joint_values);
        planning_scene_->checkCollision(c_req, c_res, robot_state);
        num_collision_checks_++;

        in_collision = c_res.collision;
        if (!in_collision) {
            pose.joint_values = joint_values;
        }

        attempt ++;
    } while (in_collision && attempt < max_attempts);
    

    
    return !in_collision;
}

shape_msgs::SolidPrimitive PlanInstance::getPrimitive(const Object& obj) {
    shape_msgs::SolidPrimitive primitive;
    if (obj.shape == Object::Shape::Box) {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = obj.length;
        primitive.dimensions[primitive.BOX_Y] = obj.width;
        primitive.dimensions[primitive.BOX_Z] = obj.height;
    }
    else if (obj.shape == Object::Shape::Cylinder) {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = obj.length;
        primitive.dimensions[primitive.CYLINDER_RADIUS] = obj.radius;
    }
    return primitive;
}

geometry_msgs::Pose PlanInstance::getPose(const Object& obj) {
    geometry_msgs::Pose pose;
    pose.position.x = obj.x;
    pose.position.y = obj.y;
    pose.position.z = obj.z;
    pose.orientation.x = obj.qx;
    pose.orientation.y = obj.qy;
    pose.orientation.z = obj.qz;
    pose.orientation.w = obj.qw;
    return pose;
}

void MoveitInstance::addMoveableObject(const Object& obj) {
    if (objects_.find(obj.name) != objects_.end()) {
        log("Object " + obj.name + " already exists in the scene", LogLevel::ERROR);
        return;
    }
    log("Adding object " + obj.name + " to the scene", LogLevel::DEBUG);
    moveit_msgs::CollisionObject co;
    co.header.frame_id = obj.parent_link;
    co.header.stamp = ros::Time::now();
    co.id = obj.name;

    objects_[obj.name] = obj;

    shape_msgs::SolidPrimitive primitive = getPrimitive(obj);
    geometry_msgs::Pose world_pose = getPose(obj);
    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(world_pose);
    co.operation = co.ADD;
 
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::setObjectColor(const std::string &name, double r, double g, double b, double a) {
    if (objects_.find(name) == objects_.end()) {
        return;
    }

    moveit_msgs::ObjectColor oc;
    oc.id = name;
    oc.color.r = r;
    oc.color.g = g;
    oc.color.b = b;
    oc.color.a = a;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.object_colors.push_back(oc);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client_.call(srv);
}

void MoveitInstance::moveObject(const Object& obj) {
    log("Moving object " + obj.name + " to " + std::to_string(obj.x) + " " + std::to_string(obj.y) + " " + std::to_string(obj.z), LogLevel::DEBUG);
    if (objects_.find(obj.name) == objects_.end()) {
        addMoveableObject(obj);
        return;
    }

    moveit_msgs::CollisionObject co;
    co.header.frame_id = obj.parent_link;
    co.header.stamp = ros::Time::now();
    co.id = obj.name;
    
    co.pose.position.x = obj.x;
    co.pose.position.y = obj.y;
    co.pose.position.z = obj.z ;
    co.pose.orientation.x = obj.qx;
    co.pose.orientation.y = obj.qy;
    co.pose.orientation.z = obj.qz;
    co.pose.orientation.w = obj.qw;

    co.operation = co.MOVE;
 
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::removeObject(const std::string &name)
{
    // find the object in the scene
    auto it = objects_.find(name);
    if (it != objects_.end()) {
        // delete the object in objects list
        objects_.erase(it);
        log("Removing object " + name + " from the scene", LogLevel::DEBUG);
    }
    else {
        log("Object " + name + " not found in the scene", LogLevel::ERROR);
        return;
    }

    // remove the object from the scene
    moveit_msgs::CollisionObject co;
    co.id = name;
    co.operation = co.REMOVE;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::moveRobot(int robot_id, const RobotPose& pose) {
    log("Moving robot " + robot_names_[robot_id], LogLevel::DEBUG);
    moveit_msgs::PlanningScene cur_scene;
    planning_scene_->getPlanningSceneMsg(cur_scene);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    auto joint_names = planning_scene_->getRobotModel()->getJointModelGroup(robot_names_[robot_id])->getActiveJointModelNames();
    for (int i = 0; i < pose.joint_values.size(); i++) {
        planning_scene.robot_state.joint_state.name.push_back(joint_names[i]);
        planning_scene.robot_state.joint_state.position.push_back(pose.joint_values[i]);
    }
    if (pose.hand_values.size() > 0 && hand_names_.size() > robot_id) {
        auto handjoint_names = planning_scene_->getRobotModel()->getJointModelGroup(hand_names_[robot_id])->getActiveJointModelNames();
        for (int i = 0; i < pose.hand_values.size(); i++) {
            planning_scene.robot_state.joint_state.name.push_back(handjoint_names[i]);
            planning_scene.robot_state.joint_state.position.push_back(pose.hand_values[i]);
        }
    }
    planning_scene.robot_state.attached_collision_objects = cur_scene.robot_state.attached_collision_objects;
    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const RobotPose& pose) {
    /*
    directly attach the object to the robot
    */
   if (objects_.find(name) == objects_.end()) {
       log("Object " + name + " not found in the scene", LogLevel::ERROR);
       return;
   }
    Object &obj = objects_[name];
    if (obj.state == Object::State::Attached) {
        log("Object " + name + " is already attached to a robot " + robot_names_[obj.robot_id] 
            + " cannot attach to robot " + std::to_string(robot_id), LogLevel::ERROR);
        return;
    }
    log("Attaching object " + name + " to robot " + robot_names_[robot_id] + " at link " + link_name, LogLevel::DEBUG);

    obj.state = Object::State::Attached;
    obj.robot_id = robot_id;
    std::string old_parent_link = obj.parent_link;
    obj.parent_link = link_name;

    // calculate relative pose
    // obj.x_attach = 0.006;
    // obj.y_attach = 0.0;
    // obj.z_attach = 0.07;

    // update in moveit
    moveit_msgs::AttachedCollisionObject co;
    co.link_name = obj.parent_link;
    co.object.header.frame_id = obj.parent_link;
    co.object.header.stamp = ros::Time::now();
    co.object.id = name;
    co.object.operation = co.object.ADD;

    // geometry_msgs::Pose relative_pose;
    // relative_pose.position.x = obj.x_attach;
    // relative_pose.position.y = obj.y_attach;
    // relative_pose.position.z = obj.z_attach;
    // relative_pose.orientation.x = obj.qx_attach;
    // relative_pose.orientation.y = obj.qy_attach;
    // relative_pose.orientation.z = obj.qz_attach;
    // relative_pose.orientation.w = obj.qw_attach;
        
    // shape_msgs::SolidPrimitive primitive;
    // if (obj.shape == Object::Shape::Box) {
    //     primitive.type = primitive.BOX;
    //     primitive.dimensions.resize(3);
    //     primitive.dimensions[primitive.BOX_X] = obj.length;
    //     primitive.dimensions[primitive.BOX_Y] = obj.width;
    //     primitive.dimensions[primitive.BOX_Z] = obj.height;
    // } 

    // co.object.primitives.push_back(primitive);
    // co.object.primitive_poses.push_back(relative_pose);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    // if (old_parent_link == "base") {
    //     moveit_msgs::CollisionObject co_remove;
    //     co_remove.id = obj.name;
    //     co_remove.header.frame_id = old_parent_link;
    //     co_remove.operation = co_remove.REMOVE;
    //     planning_scene.world.collision_objects.push_back(co_remove);
    // } else {
    //     moveit_msgs::AttachedCollisionObject co_remove;
    //     co_remove.object.id = obj.name;
    //     co_remove.link_name = old_parent_link;
    //     co_remove.object.operation = co_remove.object.REMOVE;
    //     planning_scene.robot_state.attached_collision_objects.push_back(co_remove);
    // }
    planning_scene.robot_state.attached_collision_objects.push_back(co);
    planning_scene.robot_state.is_diff = true;
    
    auto joint_names = planning_scene_->getRobotModel()->getJointModelGroup(robot_names_[robot_id])->getActiveJointModelNames();
    for (int i = 0; i < pose.joint_values.size(); i++) {
        planning_scene.robot_state.joint_state.name.push_back(joint_names[i]);
        planning_scene.robot_state.joint_state.position.push_back(pose.joint_values[i]);
    }
    if (pose.hand_values.size() > 0 && hand_names_.size() > robot_id) {
        auto handjoint_names = planning_scene_->getRobotModel()->getJointModelGroup(hand_names_[robot_id])->getActiveJointModelNames();
        for (int i = 0; i < pose.hand_values.size(); i++) {
            planning_scene.robot_state.joint_state.name.push_back(handjoint_names[i]);
            planning_scene.robot_state.joint_state.position.push_back(pose.hand_values[i]);
        }
    }
    
    planning_scene_->usePlanningSceneMsg(planning_scene);

    planning_scene_diff_ = planning_scene;

}

void MoveitInstance::detachObjectFromRobot(const std::string& name, const RobotPose& pose) {
    if (objects_.find(name) == objects_.end()) {
        log("Object " + name + " not found in the scene", LogLevel::ERROR);
        return;
    }
    Object &obj = objects_[name];
    if (obj.state != Object::State::Attached) {
        log("Object " + name + " is not attached to any robot", LogLevel::ERROR);
        return;
    }

    log("Detaching object " + name + " from robot " + robot_names_[obj.robot_id], LogLevel::DEBUG);
    obj.state = Object::State::Static;
    obj.robot_id = -1;
    std::string old_parent_link = obj.parent_link;
    obj.parent_link = "base";

    moveit_msgs::AttachedCollisionObject co_remove;
    co_remove.object.id = name;
    co_remove.link_name = old_parent_link;
    co_remove.object.operation = co_remove.object.REMOVE;

    // moveit_msgs::CollisionObject co;
    // co.id = obj.name;
    // co.header.frame_id = "base";
    // co.operation = co.ADD;

    // geometry_msgs::Pose world_pose;
    // world_pose.position.x = obj.x;
    // world_pose.position.y = obj.y;
    // world_pose.position.z = obj.z;
    // world_pose.orientation.x = obj.qx;
    // world_pose.orientation.y = obj.qy;
    // world_pose.orientation.z = obj.qz;
    // world_pose.orientation.w = obj.qw;
        
    // shape_msgs::SolidPrimitive primitive;
    // if (obj.shape == Object::Shape::Box) {
    //     primitive.type = primitive.BOX;
    //     primitive.dimensions.resize(3);
    //     primitive.dimensions[primitive.BOX_X] = obj.length;
    //     primitive.dimensions[primitive.BOX_Y] = obj.width;
    //     primitive.dimensions[primitive.BOX_Z] = obj.height;
    // }

    // co.object.primitives.push_back(primitive);
    // co.object.primitive_poses.push_back(world_pose);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    //planning_scene.world.collision_objects.push_back(co);
    planning_scene.robot_state.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.push_back(co_remove);
    auto joint_names = planning_scene_->getRobotModel()->getJointModelGroup(robot_names_[pose.robot_id])->getActiveJointModelNames();
    for (int i = 0; i < pose.joint_values.size(); i++) {
        planning_scene.robot_state.joint_state.name.push_back(joint_names[i]);
        planning_scene.robot_state.joint_state.position.push_back(pose.joint_values[i]);
    }
    if (pose.hand_values.size() > 0 && hand_names_.size() > pose.robot_id) {
        auto handjoint_names = planning_scene_->getRobotModel()->getJointModelGroup(hand_names_[pose.robot_id])->getActiveJointModelNames();
        for (int i = 0; i < pose.hand_values.size(); i++) {
            planning_scene.robot_state.joint_state.name.push_back(handjoint_names[i]);
            planning_scene.robot_state.joint_state.position.push_back(pose.hand_values[i]);
        }
    }

    planning_scene_->usePlanningSceneMsg(planning_scene);

    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::updateScene() {
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene_diff_;
    bool success = planning_scene_diff_client_.call(srv);
    if (!success || !srv.response.success) {
        log("Failed to update the planning scene", LogLevel::WARN);
    }
}

void MoveitInstance::resetScene(bool reset_sim) {
    // for (const auto& pair : objects_) {
    //     if (pair.second.state == Object::State::Attached) {
    //         detachObjectFromRobot(pair.first, initRobotPose(pair.second.robot_id));
    //         if (reset_sim) {
    //             updateScene();
    //         }
    //     }
    // }

    // std::vector<std::string> keys;
    // for (const auto& pair : objects_) {
    //    keys.push_back(pair.first);
    // }
    // for (const auto& key : keys) {
    //     removeObject(key);
    //     if (reset_sim) {
    //         updateScene();
    //     }
    // }

    planning_scene_->setPlanningSceneMsg(original_scene_);
    if (reset_sim) {
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = original_scene_;
        planning_scene_diff_client_.call(srv);
    }
    objects_.clear();
}

void MoveitInstance::printKnownObjects() const {
    planning_scene_->printKnownObjects();
}

bool MoveitInstance::setCollision(const std::string& obj_name, const std::string& link_name, bool allow) {

    // Get the Allowed Collision Matrix (ACM)
    // Use the PlanningSceneMonitor to get the current planning scene

    moveit_msgs::AllowedCollisionMatrix acm;
    planning_scene_->getAllowedCollisionMatrixNonConst().getMessage(acm);

    if (std::find(acm.entry_names.begin(), acm.entry_names.end(), obj_name) == acm.entry_names.end()) {
        // object is not in the ACM, add a new row and column
        acm.entry_names.push_back(obj_name);

        // first visit all esiting rows and add a new column
        for (size_t i = 0; i < acm.entry_values.size(); i++) {
            if (acm.entry_names[i] == link_name) {
                acm.entry_values[i].enabled.push_back(allow);
            }
            // by default we disallow object to object collision
            else if (objects_.find(acm.entry_names[i]) != objects_.end()) {
                acm.entry_values[i].enabled.push_back(false);
            }
            else {
                acm.entry_values[i].enabled.push_back(false);
            }
        }
        // add a new row
        moveit_msgs::AllowedCollisionEntry new_entry;
        for (size_t i = 0; i < acm.entry_names.size(); i++) {
            if (acm.entry_names[i] == link_name) {
                new_entry.enabled.push_back(allow);
            }
            else if (objects_.find(acm.entry_names[i]) != objects_.end()) {
                new_entry.enabled.push_back(false);
            }
            else {
                new_entry.enabled.push_back(false);
            }
        }
        acm.entry_values.push_back(new_entry);
    }
    else {
        // directly modify the entry
        for (size_t i = 0; i < acm.entry_names.size(); i++) {
            for (size_t j = 0; j < acm.entry_names.size(); j++) {
                if (acm.entry_names[i] == obj_name && acm.entry_names[j] == link_name) {
                    acm.entry_values[i].enabled[j] = allow;
                }
                if (acm.entry_names[i] == link_name && acm.entry_names[j] == obj_name) {
                    acm.entry_values[i].enabled[j] = allow;
                }
            }
        }
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.allowed_collision_matrix = acm;
    planning_scene_->usePlanningSceneMsg(planning_scene);

    planning_scene_diff_ = planning_scene;

    return true;
}