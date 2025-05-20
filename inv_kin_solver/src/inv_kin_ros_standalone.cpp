#include "rclcpp/rclcpp.hpp"
#include <chrono>  
#include <string>
#include <vector>
#include "optimizer_SQP.hpp"
#include "optimization.h"

class StandAloneInvKinSolver : public rclcpp::Node
{
public:
    StandAloneInvKinSolver() : Node("standalone_inv_kin_node")
    {
        this->declare_parameter("x_ee", 0.1);
        this->declare_parameter("y_ee", 0.1);
        this->declare_parameter("z_ee", 0.1);

        x_ee_pos = this->get_parameter("x_ee").as_double();
        y_ee_pos = this->get_parameter("y_ee").as_double();
        z_ee_pos = this->get_parameter("z_ee").as_double();


        RCLCPP_INFO(this->get_logger(), "Solving IK for EE position: [%.3f, %.3f, %.3f]", x_ee_pos, y_ee_pos, z_ee_pos);

        auto start = std::chrono::high_resolution_clock::now();

        alglib::real_1d_array q_prev = "[0,0,0,0]";
        
        // Call the IK solver
        alglib::real_1d_array joint_angles = solve_Ik(x_ee_pos, y_ee_pos, z_ee_pos, q_prev);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;

        // Call the FK solver
        est_ee_pos = forward_kinematics(joint_angles);

        // Log the solution
        std::string result_str = joint_angles.tostring(6);
        RCLCPP_INFO(this->get_logger(), "Joint angles: %s", result_str.c_str());
        RCLCPP_INFO(this->get_logger(), "IK computation took %.6f seconds", duration.count());
        RCLCPP_INFO(this->get_logger(), "Estimated EE position: [%.3f, %.3f, %.3f]", est_ee_pos.at(0), est_ee_pos.at(1), est_ee_pos.at(2));
    }

private:
    double x_ee_pos, y_ee_pos, z_ee_pos;
    std::vector<double> est_ee_pos;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StandAloneInvKinSolver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}