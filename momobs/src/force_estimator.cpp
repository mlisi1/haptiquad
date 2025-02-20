#include <momobs/force_estimator.hpp>

momobs::ForceEstimator::ForceEstimator() {}

void momobs::ForceEstimator::initModel(pinocchio::Model pin_model) {

    model = pin_model;
    data = pinocchio::Data(model);

    r_lin = Eigen::VectorXd::Zero(model.nv-3);
    r_ang = Eigen::VectorXd::Zero(model.nv-3);

    initialized = true;

}



void momobs::ForceEstimator::setNumContacts(int num_contacts) {
    
    if (!initialized) {
        throw std::runtime_error("[ForceEstimator]: Error - ForceEstimator has not been initialized yet.");
    }

    num_contacts_ = num_contacts;

    J.resize(num_contacts);
    J_fb.resize(num_contacts);
    J_lin.resize(num_contacts);
    J_ang.resize(num_contacts);
    F_.resize(num_contacts);
    

}


void momobs::ForceEstimator::findFeetFrames(std::vector<std::string> joint_names) {

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }

    std::unordered_set<std::string> seen;
    std::unordered_set<std::string> uniquePrefixes;
    for (const auto& name : joint_names) {
        if (name.length() >= 2) {
            std::string prefix = name.substr(0, 2);
            // Add to footNames only if the prefix is not already seen
            if (seen.find(prefix) == seen.end()) {
                seen.insert(prefix);
                feet_frames_.push_back(prefix + "_FOOT");
            }
        }
    }

    if (feet_frames_.size() != num_contacts_) {
        throw std::runtime_error("[ForceEstimator]: Error - findFeetFrames(): feet frames size and number of contacts do not match (" + 
            std::to_string(feet_frames_.size()) + ", " + std::to_string(num_contacts_) + ")" );
    }

}


void momobs::ForceEstimator::setFeetFrames(std::vector<std::string> feet_frames) {

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }
    feet_frames_ = feet_frames;

    if (feet_frames_.size() != num_contacts_) {
        throw std::runtime_error("[ForceEstimator]: Error - findFeetFrames(): feet frames size and number of contacts do not match (" + 
            std::to_string(feet_frames_.size()) + ", " + std::to_string(num_contacts_) + ")" );
    }

}



void momobs::ForceEstimator::setFeetOnGround(std::map<std::string, bool> is_on_ground) {

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }

    if (feet_frames_.size() == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - setFeetOnGround(): feet frames has not been specified yet.");
    }

    if (feet_frames_.size() != is_on_ground.size()) {
        throw std::runtime_error("[ForceEstimator]: Error - setFeetOnGround(): argument size and feet frames size do not match.");
    }

    is_on_ground_.resize(num_contacts_);

    for (int i=0; i<num_contacts_; i++) {
        is_on_ground_[i] = is_on_ground[feet_frames_[i]];
    }

}





void momobs::ForceEstimator::updateJacobians(JointStateDict q, Eigen::MatrixXd F, Eigen::MatrixXd IC) {

    if (!initialized) {
        throw std::runtime_error("[ForceEstimator]: Error - ForceEstimator has not been initialized yet.");
    }

    if (q.size() != model.nv-6) {
        throw std::runtime_error("[ForceEstimator]: Error - updateJacobians(): argument dimension(s) (q) does not match model's.");
    }

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }

    if (F.rows() != 6 || F.cols() != model.nv-6) {
        throw std::runtime_error("[ForceEstimator]: Error - updateJacobians(): argument dimension(s) (F) does not match model's.");
    }

    if (IC.rows() != 6 || IC.cols() != 6) {
        throw std::runtime_error("[ForceEstimator]: Error - updateJacobians(): argument dimension(s) (IC) are wrong.");
    }


    Eigen::VectorXd q_ = Eigen::VectorXd::Zero(model.nq);
    q_(6) = 1.0;

    for (int i=0; i<q.size(); i++) {
        q_(i+7) = q[model.names[i+2]];
    }

    pinocchio::forwardKinematics(model, data, q_);
    pinocchio::framesForwardKinematics(model, data, q_);
    pinocchio::updateFramePlacements(model, data);



    for (int i=0; i<num_contacts_; i++) {

        J[i] = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q_, model.getFrameId(feet_frames_[i]), pinocchio::LOCAL_WORLD_ALIGNED, J[i]);
        J[i] = J[i].block(0,6, 6, model.nv-6);

        J_fb[i] = (J[i].transpose() - F.transpose() * IC.inverse()).transpose();
        J_lin[i] = J[i].block(0,0, 3, model.nv-6);
        J_ang[i] = J[i].block(3,0, 3, model.nv-6);
    }

}



std::vector<Eigen::VectorXd> momobs::ForceEstimator::calculateForces(Eigen::VectorXd r_int, Eigen::VectorXd r_ext, Eigen::Quaterniond orientation) {

    if (!initialized) {
        throw std::runtime_error("[ForceEstimator]: Error - ForceEstimator has not been initialized yet.");
    }

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }

    if (is_on_ground_.size() == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - ground contact has not been set.");
    }

    int num_forces = 0;
    for (int i=0; i<num_contacts_; i++) {
        if (!is_on_ground_[i]) {
        } else {
            num_forces += 1;
        }
    }


    if (num_forces == 0) {

        for (int i=0; i<num_contacts_; i++) {
            F_[i] = Eigen::VectorXd::Zero(6);
        }
        return F_;
    }


    to_pinv_lin = Eigen::MatrixXd::Zero(model.nv-3, 3*num_forces);
    to_pinv_ang = Eigen::MatrixXd::Zero(model.nv-3, 3*num_forces);

    r_lin.head<3>() = r_ext.head<3>();
    r_ang.head<3>() = r_ext.tail<3>();

    r_lin.tail(model.nv-6) = r_int;
    r_ang.tail(model.nv-6) = r_int;


    int off = 0;

    for (int i=0; i<num_contacts_; i++) {

        if (!is_on_ground_[i]) {
            off++;
            continue;
        } else {

            to_pinv_lin.block(0, (3*(i-off)), 3,3) = orientation.toRotationMatrix().transpose();
            to_pinv_lin.block(3, (3*(i-off)), model.nv-6, 3) = J_lin[i].transpose() * orientation.toRotationMatrix().transpose();

            to_pinv_ang.block(0, (3*(i-off)), 3,3) = orientation.toRotationMatrix().transpose();
            to_pinv_ang.block(3, (3*(i-off)), model.nv-6, 3) = J_ang[i].transpose() * orientation.toRotationMatrix().transpose();

        }
    }

    Eigen::VectorXd forces = to_pinv_lin.completeOrthogonalDecomposition().pseudoInverse() * r_lin;
    Eigen::VectorXd torques = to_pinv_ang.completeOrthogonalDecomposition().pseudoInverse() * r_ang;

    off = 0;

    for (int i=0; i<num_contacts_; i++) {

        F_[i] = Eigen::VectorXd::Zero(6);

        if (!is_on_ground_[i]) {
            off+=1;
            continue;            
        } else {
            F_[i].head<3>() = forces.segment(3*(i-off), 3);

            if (include_torques) {
                F_[i].tail<3>() = torques.segment(3*(i-off), 3);
            }
        }
    }

    return F_;

}