#include <pinocchio/multibody/data.hpp>
#include "pinocchio/algorithm/model.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <stdexcept> 
#include <vector>
#include <unordered_set>


using JointStateDict = std::map<std::string, double>;

namespace haptiquad
{

class ForceEstimator {

    public:

        ForceEstimator();

        void initModel(pinocchio::Model pin_model);
        void setBaseFrame(std::string base_link);

        // void setFeetOnGround(std::map<std::string, bool> is_on_ground);
        void findFeetFrames(std::vector<std::string> joint_names);
        void setFeetFrames(std::vector<std::string> feet_frames);
        void updateJacobians(JointStateDict q, Eigen::MatrixXd F, Eigen::MatrixXd IC);
        std::map<std::string, Eigen::VectorXd> calculateForces(Eigen::VectorXd r_int, Eigen::VectorXd r_ext, Eigen::Quaterniond orientation);
        std::tuple<Eigen::VectorXd, Eigen::VectorXd> calculateResidualsFromForces(std::map<std::string, Eigen::VectorXd> forces);
        void setNumContacts(int num_contacts);

        std::vector<std::string> getFeetFrames();


    private:

        bool initialized = false;
        bool is_pseudo = false;
        // bool include_torques = false;

        pinocchio::Model model;
        pinocchio::Data data;

        Eigen::Quaterniond orientation_;

        std::vector<Eigen::MatrixXd> J, J_fb;
        // std::vector<Eigen::MatrixXd> J_fb, J_full_fb, J_w_fb, J_w_full_fb;
        Eigen::MatrixXd J_w, J_w_fb;
        // std::vector<Eigen::MatrixXd> J_lin, J_ang;

        std::vector<std::string> feet_frames_;
        std::string base_frame;

        size_t num_contacts_ = 0;

        // std::vector<bool> is_on_ground_;

        std::map<std::string, Eigen::VectorXd> F_;

        // Eigen::MatrixXd to_pinv_lin, to_pinv_ang;
        // Eigen::VectorXd r_lin, r_ang;

        Eigen::MatrixXd to_pinv;
        Eigen::VectorXd r;
        Eigen::VectorXd forces;



};
    
} // namespace haptiquad
