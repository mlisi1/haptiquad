#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include "pinocchio/algorithm/model.hpp"
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/algorithm/rnea.hpp>
// #include <pinocchio/algorithm/crba.hpp>

#include <vector>
#include <stdexcept> 

using JointStateDict = std::map<std::string, double>;


namespace momobs {

class MomentumObserver {

    public:

        MomentumObserver();

        void initModel(pinocchio::Model pin_model);

        void setInteralGain(float k_int);
        void setExternalGain(float k_ext);

        void updateJointStates(JointStateDict q, JointStateDict q_dot, JointStateDict torques);
        void updateBaseState(Eigen::VectorXd v0, Eigen::Quaterniond orientation);

        void enableTimeScaling(double expected_dt, double threshold);
        void disableTimeScaling();

        Eigen::VectorXd subtractFrictionTorque(Eigen::VectorXd torques);

        std::tuple<Eigen::VectorXd, Eigen::VectorXd> getResiduals(double dt);



    private:

        bool initialized = false;

        float k_int, k_ext = 1.0;


        pinocchio::Model model;
        pinocchio::Data data;

        Eigen::VectorXd q_, q_dot_, torques_;
        Eigen::VectorXd integral_int, integral_ext, r_int, r_ext;

        Eigen::MatrixXd K_int, K_ext;

        Eigen::MatrixXd pin_H_dot;

        Eigen::MatrixXd H, F, IC, H_dot, F_dot, IC_dot;
        Eigen::VectorXd C, p0c;

        Eigen::MatrixXd H_fb, H_dot_fb;
        Eigen::VectorXd C_fb;



        //Time rescaling
        bool rescale = false;
        double scale_factor = 1.0;
        double expected_dt_ = 0.0;
        double threshold_ = 0.0;     




};

}