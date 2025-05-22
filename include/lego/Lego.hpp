#pragma once
#include "lego/Utils/Math.hpp"
#include "lego/Utils/FileIO.hpp"
#include "gazebo_msgs/SetModelState.h"

namespace lego_manipulation
{
namespace lego
{
struct lego_brick{
    std::string brick_name;
    int height;
    int width;
    double x;
    double y;
    double z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
    double cur_x;
    double cur_y;
    double cur_z;
    int press_side;
    int press_offset;
    bool fixed;
    Eigen::Quaterniond cur_quat;
    bool in_stock;
    std::map<std::string, std::string> top_connect;
    std::map<std::string, std::string> bottom_connect;
};

struct lego_plate{
    int height;
    int width;
    Eigen::Matrix4d pose;
};

class Lego
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<Lego> Ptr;
        typedef std::shared_ptr<Lego const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        std::map<std::string, lego_brick> brick_map_;
        Json::Value lego_library_;
        Json::Value config_;
        ros::ServiceClient client_; 
        gazebo_msgs::SetModelState setmodelstate_;
        double brick_height_m_ = 0.0096;
        double lever_wall_height_ = 0.0032;
        double brick_len_offset_ = 0.0000;
        double P_len_ = 0.008;
        double EPS_ = 0.00001;
        double knob_height_ = 0.0017;
        lego_plate assemble_plate_;
        lego_plate storage_plate_;
        double table_width_ = 0.6985;
        double table_length_ = 1.219;
        int r1_robot_dof_ = 6;
        int r2_robot_dof_ = 6;
        Eigen::Matrix4d world_base_frame_;
        Eigen::Matrix4d world_T_base_inv_;
        Eigen::MatrixXd thetamax_; 
        Eigen::MatrixXd thetamax_rad_;

        Eigen::MatrixXd r1_DH_; // size = [n_joint + n_ee, 4]
        Eigen::Matrix4d r1_ee_inv_, r1_tool_inv_, r1_tool_assemble_inv_, r1_tool_disassemble_inv_, r1_tool_alt_inv_, r1_tool_alt_assemble_inv_, r1_tool_handover_assemble_inv_;
        Eigen::MatrixXd r1_DH_tool_;
        Eigen::MatrixXd r1_DH_tool_assemble_;
        Eigen::MatrixXd r1_DH_tool_disassemble_;
        Eigen::MatrixXd r1_DH_tool_alt_;
        Eigen::MatrixXd r1_DH_tool_alt_assemble_;
        Eigen::MatrixXd r1_DH_tool_handover_assemble_;
        Eigen::MatrixXd r1_base_frame_;
        Eigen::Matrix4d r1_T_base_inv_;

        Eigen::MatrixXd r2_DH_; // size = [n_joint + n_ee, 4]
        Eigen::Matrix4d r2_ee_inv_, r2_tool_inv_, r2_tool_assemble_inv_, r2_tool_disassemble_inv_, r2_tool_alt_inv_, r2_tool_alt_assemble_inv_, r2_tool_handover_assemble_inv_;
        Eigen::MatrixXd r2_DH_tool_;
        Eigen::MatrixXd r2_DH_tool_assemble_;
        Eigen::MatrixXd r2_DH_tool_disassemble_;
        Eigen::MatrixXd r2_DH_tool_alt_;
        Eigen::MatrixXd r2_DH_tool_alt_assemble_;
        Eigen::MatrixXd r2_DH_tool_handover_assemble_;
        Eigen::MatrixXd r2_base_frame_;
        Eigen::Matrix4d r2_T_base_inv_;

        void update_all_top_bricks(const std::string& brick_name, const Eigen::Matrix4d& dT);
        void update(const std::string& brick_name, const Eigen::Matrix4d& T);
        void calc_brick_loc(const lego_brick& brick, const lego_plate& plate, const int& orientation,
                            const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z, 
                            Eigen::Matrix4d& out_pose);
        bool is_top_connect(const lego_brick& b1, const lego_brick& b2);
        bool is_bottom_connect(const lego_brick& b1, const lego_brick& b2);
        bool bricks_overlap(const lego_brick& b1, const lego_brick& b2);
        void get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry);


    public:
        Lego();
        ~Lego(){}
                
        // Operations
        void setup(const std::string& env_setup_fname, const std::string& lego_lib_fname, const std::string &plate_calib_fname, const bool& assemble, const Json::Value& task_json, const std::string& world_base_fname,
                   const std::string& r1_DH_fname, const std::string& r1_DH_tool_fname, const std::string& r1_DH_tool_disassemble_fname, 
                   const std::string& r1_DH_tool_assemble_fname, const std::string& r1_DH_tool_alt_fname, 
                   const std::string& r1_DH_tool_alt_assemble_fname, const std::string &r1_DH_tool_handover_assemble_fname, const std::string& r1_base_fname, 
                   const std::string& r2_DH_fname, const std::string& r2_DH_tool_fname, const std::string& r2_DH_tool_disassemble_fname, 
                   const std::string& r2_DH_tool_assemble_fname, const std::string& r2_DH_tool_alt_fname, 
                   const std::string& r2_DH_tool_alt_assemble_fname, const std::string &r2_DH_tool_handover_assemble_fname, const std::string& r2_base_fname, 
                   const ros::ServiceClient& cli);
        void set_robot_base(const std::string& r1_base_fname, const std::string& r2_base_fname);
        void set_DH(const std::string& r1_DH_fname, const std::string& r2_DH_fname);
        void set_DH_tool(const std::string& r1_DH_tool_fname, const std::string& r2_DH_tool_fname);
        void set_DH_tool_assemble(const std::string& r1_DH_tool_assemble_fname, const std::string& r2_DH_tool_assemble_fname);
        void set_DH_tool_disassemble(const std::string& r1_DH_tool_disassemble_fname, const std::string& r2_DH_tool_disassemble_fname);
        void set_DH_tool_alt(const std::string& r1_DH_tool_alt_fname, const std::string& r2_DH_tool_alt_fname);
        void set_DH_tool_alt_assemble(const std::string& r1_DH_tool_alt_assemble_fname, const std::string& r2_DH_tool_alt_assemble_fname);
        void set_DH_tool_handover_assemble(const std::string& r1_DH_tool_handover_assemble_fname, const std::string& r2_DH_tool_handover_assemble_fname);
        void print_manipulation_property();
        void set_assemble_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);
        void set_storage_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);
        void set_world_base(const std::string& world_base_fname);

        void update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                           const bool& joint_rad, const std::string& brick_name, const int& mode);
        std::string get_brick_name_by_id(const int& id, const int& seq_id);
        std::string get_brick_name_by_id(const int& id, const std::string& seq_id);
        void update_brick_connection();
        void calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                  const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                  const int& orientation, const int& press_side, const int& press_offset, Eigen::MatrixXd& T);
        void calc_brick_sup_pose(int robot_id, const int &sup_x, const int &sup_y, const int &sup_z, const int &sup_ori, const double &z_offset, Eigen::MatrixXd &T);
        
        void brick_pose_in_stock(const std::string& name, const int& press_side, const int& press_offset, Eigen::Matrix4d& T);
        void support_pose_down_pre(const int& x, const int& y, const int& z, const int& ori, Eigen::Matrix4d& T);
        void support_pose_down(const int& x, const int& y, const int& z, const int& ori, Eigen::Matrix4d& T);
        void support_pose(const int& x, const int& y, const int& z, const int& ori, Eigen::Matrix4d& T);
        void assemble_pose_from_top(const int& press_x, const int& press_y, const int& press_z, const int& press_ori, const int& press_side, Eigen::Matrix4d& T);
        bool joint_in_range(const math::VectorJd& theta, const bool& is_rad);
        math::VectorJd IK(const math::VectorJd& cur_q, const Eigen::Matrix4d& goal_T, const Eigen::MatrixXd& DH, const Eigen::Matrix4d& T_base, const Eigen::Matrix4d& T_base_inv,
                          const Eigen::Matrix4d& T_tool_inv, const bool& joint_rad, bool& status);

        int brick_instock(const std::string& name) {return brick_map_[name].in_stock;};
        int robot_dof_1() {return r1_robot_dof_;};
        int robot_dof_2() {return r2_robot_dof_;};
        Eigen::MatrixXd world_base_frame() {return world_base_frame_;};
        Eigen::MatrixXd world_base_inv() {return world_T_base_inv_;};
        Eigen::MatrixXd robot_DH_r1() {return r1_DH_;};
        Eigen::MatrixXd robot_DH_tool_r1() {return r1_DH_tool_;};
        Eigen::MatrixXd robot_DH_tool_assemble_r1() {return r1_DH_tool_assemble_;};
        Eigen::MatrixXd robot_DH_tool_alt_r1() {return r1_DH_tool_alt_;};
        Eigen::MatrixXd robot_DH_tool_alt_assemble_r1() {return r1_DH_tool_alt_assemble_;};
        Eigen::MatrixXd robot_DH_tool_disassemble_r1() {return r1_DH_tool_disassemble_;};
        Eigen::MatrixXd robot_DH_tool_handover_assemble_r1() {return r1_DH_tool_handover_assemble_;};
        Eigen::MatrixXd robot_base_r1() {return r1_base_frame_;};
        Eigen::Matrix4d robot_base_inv_r1() {return r1_T_base_inv_;};
        Eigen::Matrix4d robot_ee_inv_r1() {return r1_ee_inv_;};
        Eigen::Matrix4d robot_tool_inv_r1() {return r1_tool_inv_;};
        Eigen::Matrix4d robot_tool_assemble_inv_r1() {return r1_tool_assemble_inv_;};
        Eigen::Matrix4d robot_tool_alt_inv_r1() {return r1_tool_alt_inv_;};
        Eigen::Matrix4d robot_tool_alt_assemble_inv_r1() {return r1_tool_alt_assemble_inv_;};
        Eigen::Matrix4d robot_tool_disassemble_inv_r1() {return r1_tool_disassemble_inv_;};
        Eigen::Matrix4d robot_tool_handover_assemble_inv_r1() {return r1_tool_handover_assemble_inv_;};
        Eigen::MatrixXd robot_DH_r2() {return r2_DH_;};
        Eigen::MatrixXd robot_DH_tool_r2() {return r2_DH_tool_;};
        Eigen::MatrixXd robot_DH_tool_assemble_r2() {return r2_DH_tool_assemble_;};
        Eigen::MatrixXd robot_DH_tool_disassemble_r2() {return r2_DH_tool_disassemble_;};
        Eigen::MatrixXd robot_DH_tool_alt_r2() {return r2_DH_tool_alt_;};
        Eigen::MatrixXd robot_DH_tool_alt_assemble_r2() {return r2_DH_tool_alt_assemble_;};
        Eigen::MatrixXd robot_DH_tool_handover_assemble_r2() {return r2_DH_tool_handover_assemble_;};
        Eigen::MatrixXd robot_base_r2() {return r2_base_frame_;};
        Eigen::Matrix4d robot_base_inv_r2() {return r2_T_base_inv_;};
        Eigen::Matrix4d robot_ee_inv_r2() {return r2_ee_inv_;};
        Eigen::Matrix4d robot_tool_inv_r2() {return r2_tool_inv_;};
        Eigen::Matrix4d robot_tool_assemble_inv_r2() {return r2_tool_assemble_inv_;};
        Eigen::Matrix4d robot_tool_alt_inv_r2() {return r2_tool_alt_inv_;};
        Eigen::Matrix4d robot_tool_alt_assemble_inv_r2() {return r2_tool_alt_assemble_inv_;};
        Eigen::Matrix4d robot_tool_disassemble_inv_r2() {return r2_tool_disassemble_inv_;};
        Eigen::Matrix4d robot_tool_handover_assemble_inv_r2() {return r2_tool_handover_assemble_inv_;};

        bool robot_is_static(math::VectorJd robot_qd, math::VectorJd robot_qdd, const int& robot_dof);
        bool robot_reached_goal(math::VectorJd robot_q, math::VectorJd goal, const int& robot_dof);
        Eigen::Matrix4d assemble_plate_pose() {return assemble_plate_.pose;};
        Eigen::Matrix4d storage_plate_pose() {return storage_plate_.pose;};
        void calc_bric_asssemble_pose(const std::string &name, const int& brick_loc_x,
                            const int& brick_loc_y, const int& brick_loc_z, const int& orientation,
                            Eigen::Matrix4d& out_pose);
        double brick_height() {return brick_height_m_;};
        double lever_wall_height() {return lever_wall_height_;};
        double knob_height() {return knob_height_;};

        std::vector<std::string> get_brick_names();
        std::vector<std::string> get_active_bricks_names();
        std::vector<std::string> get_fixed_bricks_names();
        std::vector<std::string> get_brick_names_by_type(int id);
        std::vector<std::string> get_brick_above(const std::string& brick_name);
        std::vector<std::string> get_brick_below(const std::string& brick_name);
        void get_brick_sizes_by_type(const int& id, int &height, int &width);
        void brick_dimension_from_name(const std::string& b_name, int& height, int& width);

        void get_init_brick_xyzo(const std::string& brick_name, int& x, int& y, int& z, int &ori);
        geometry_msgs::Pose get_init_brick_pose(const std::string& brick_name);
        geometry_msgs::Pose get_curr_brick_pose(const std::string& brick_name);
        geometry_msgs::Pose get_table_pose();
        void get_brick_sizes(const std::string& brick_name, double& x, double& y, double& z);
        void get_table_size(double& x, double& y, double& z);
        bool is_press_pt_in_bound(const std::string& brick_name, int press_side, int press_offset);
        std::vector<std::vector<std::vector<std::string>>> gen_world_grid_from_graph(const Json::Value& task_json, int task_idx, int wx, int wy, int wz);
        
        void get_press_pt(int brick_x, int brick_y, int brick_type, int brick_ori, int press_side, int press_offset,
                    int &press_pt_x, int &press_pt_y, int &press_ori);
        void get_sup_side_ori(int support_ori, int &sup_press_side, int &sup_brick_ori);
        void get_lego_twist_next(const Json::Value &task_json, int task_idx, const std::string &brick_name, std::vector<std::string> &side_bricks);
        void get_lego_next(int press_x, int press_y, int brick_z, int press_side, int brick_ori, int brick_type, const std::string &brick_name,
                    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &side_bricks);
        void get_lego_below(int brick_x, int brick_y, int brick_z, int brick_ori, int brick_type, 
                    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &below_bricks);
        void get_lego_above(int brick_x, int brick_y, int brick_z, int brick_ori, int brick_type, 
                    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &above_bricks);   
        void lego_pose_from_press_pose(const math::VectorJd& theta, int robot_id, int brick_id, int press_side, int press_offset, 
            Eigen::Matrix4d &brick_loc);
};
}
}