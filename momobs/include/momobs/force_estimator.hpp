#include <pinocchio/multibody/data.hpp>
#include "pinocchio/algorithm/model.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <stdexcept> 
#include <vector>
#include <unordered_set>


using JointStateDict = std::map<std::string, double>;

namespace momobs
{

class ForceEstimator {

    public:

        ForceEstimator();

        void initModel(pinocchio::Model pin_model);

        void setFeetOnGround(std::map<std::string, bool> is_on_ground);
        void findFeetFrames(std::vector<std::string> joint_names);
        void setFeetFrames(std::vector<std::string> feet_frames);
        void updateJacobians(JointStateDict q, Eigen::MatrixXd F, Eigen::MatrixXd IC);
        std::vector<Eigen::VectorXd> calculateForces(Eigen::VectorXd r_int, Eigen::VectorXd r_ext, Eigen::Quaterniond orientation);
        void setNumContacts(int num_contacts);


    private:

        bool initialized = false;
        bool include_torques = false;

        pinocchio::Model model;
        pinocchio::Data data;

        std::vector<Eigen::MatrixXd> J;
        std::vector<Eigen::MatrixXd> J_fb;
        std::vector<Eigen::MatrixXd> J_lin, J_ang;

        std::vector<std::string> feet_frames_;

        int num_contacts_ = 0;

        std::vector<bool> is_on_ground_;

        std::vector<Eigen::VectorXd> F_;

        Eigen::MatrixXd to_pinv_lin, to_pinv_ang;
        Eigen::VectorXd r_lin, r_ang;



};
    
} // namespace momobs
