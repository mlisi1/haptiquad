#include <haptiquad/force_estimator.hpp>



Eigen::Matrix3d hat(const Eigen::Vector3d vector) {

    Eigen::Matrix3d hat_form;
    hat_form <<    0,              -vector.z(),   vector.y(),
                    vector.z(),     0,             -vector.x(),
                    -vector.y(),    vector.x(),    0;

    return hat_form;

}


Eigen::MatrixXd spatialTransform(const Eigen::Matrix3d rotation, const Eigen::Vector3d translation) {

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6,6);

    R.block<3,3>(0,0) = rotation;
    R.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
    R.block<3,3>(3,0) = - rotation * hat(translation);
    R.block<3,3>(3,3) = rotation;

    return R;
}





haptiquad::ForceEstimator::ForceEstimator() {}

void haptiquad::ForceEstimator::initModel(pinocchio::Model pin_model) {

    model = pin_model;
    data = pinocchio::Data(model);

    // r_lin = Eigen::VectorXd::Zero(model.nv-3);
    // r_ang = Eigen::VectorXd::Zero(model.nv-3);
    r = Eigen::VectorXd::Zero(model.nv);
    base_frame = "";

    initialized = true;

}


void haptiquad::ForceEstimator::setBaseFrame(std::string base_link) {

    base_frame = base_link;

}



void haptiquad::ForceEstimator::setNumContacts(int num_contacts) {
    
    if (!initialized) {
        throw std::runtime_error("[ForceEstimator]: Error - ForceEstimator has not been initialized yet.");
    }

    num_contacts_ = num_contacts;
    to_pinv = Eigen::MatrixXd::Zero(model.nv, 3 * (num_contacts_ + 2));

    if (to_pinv.rows() == to_pinv.cols()) {
        is_pseudo = false;
    } else {
        is_pseudo = true;
    }

    J.resize(num_contacts);
    J_fb.resize(num_contacts);   

}


void haptiquad::ForceEstimator::findFeetFrames(std::vector<std::string> joint_names) {

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


void haptiquad::ForceEstimator::setFeetFrames(std::vector<std::string> feet_frames) {

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }
    feet_frames_ = feet_frames;

    if (feet_frames_.size() != num_contacts_) {
        throw std::runtime_error("[ForceEstimator]: Error - findFeetFrames(): feet frames size and number of contacts do not match (" + 
            std::to_string(feet_frames_.size()) + ", " + std::to_string(num_contacts_) + ")" );
    }

}


std::vector<std::string>haptiquad::ForceEstimator::getFeetFrames() {

    if (!initialized) {
        throw std::runtime_error("[ForceEstimator]: Error - ForceEstimator has not been initialized yet.");
    }

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }

    if (feet_frames_.size() == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - getFeetFrames(): feet frames has not been specified yet.");
    }

    return feet_frames_;

}



// void haptiquad::ForceEstimator::setFeetOnGround(std::map<std::string, bool> is_on_ground) {

//     if (num_contacts_ == 0) {
//         throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
//     }

//     if (feet_frames_.size() == 0) {
//         throw std::runtime_error("[ForceEstimator]: Error - setFeetOnGround(): feet frames has not been specified yet.");
//     }

//     if (feet_frames_.size() != is_on_ground.size()) {
//         throw std::runtime_error("[ForceEstimator]: Error - setFeetOnGround(): argument size and feet frames size do not match.");
//     }

//     is_on_ground_.resize(num_contacts_);

//     for (int i=0; i<num_contacts_; i++) {
//         is_on_ground_[i] = is_on_ground[feet_frames_[i]];
//     }

// }





void haptiquad::ForceEstimator::updateJacobians(JointStateDict q, Eigen::MatrixXd F, Eigen::MatrixXd IC) {

    if (!initialized) {
        throw std::runtime_error("[ForceEstimator]: Error - ForceEstimator has not been initialized yet.");
    }

    if (q.size() != (size_t) model.nv-6) {
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

    if (base_frame == "") {
        throw std::runtime_error("[ForceEstimator]: Error - updateJacobians(): base_frame was not set.");
    }


    Eigen::VectorXd q_ = Eigen::VectorXd::Zero(model.nq);
    q_(6) = 1.0;

    for (size_t i=0; i<q.size(); i++) {
        q_(i+7) = q[model.names[i+2]];
    }

    pinocchio::forwardKinematics(model, data, q_);
    pinocchio::framesForwardKinematics(model, data, q_);
    pinocchio::updateFramePlacements(model, data);



    for (size_t i=0; i<num_contacts_; i++) {

        J[i] = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q_, model.getFrameId(feet_frames_[i]), pinocchio::LOCAL_WORLD_ALIGNED, J[i]);
        J_fb[i] = J[i];
        J[i] = J[i].block(0,6, 6, model.nv-6);
        J_fb[i].block(0,6, 6, model.nv-6) = (J[i].transpose() - F.transpose() * IC.inverse()).transpose();
        
    }

    J_w = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, q_, model.getFrameId(base_frame), pinocchio::LOCAL_WORLD_ALIGNED, J_w);
    J_w_fb = J_w;
    J_w = J_w.block(0,6, 6, model.nv-6);
    J_w_fb.block(0,6, 6, model.nv-6) = (J_w.transpose() - F.transpose() * IC.inverse()).transpose();

}



std::map<std::string, Eigen::VectorXd> haptiquad::ForceEstimator::calculateForces(Eigen::VectorXd r_int, Eigen::VectorXd r_ext, Eigen::Quaterniond orientation) {

    if (!initialized) {
        throw std::runtime_error("[ForceEstimator]: Error - ForceEstimator has not been initialized yet.");
    }

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }

    // if (is_on_ground_.size() == 0) {
    //     throw std::runtime_error("[ForceEstimator]: Error - ground contact has not been set.");
    // }

    // int num_forces = 0;
    // for (int i=0; i<num_contacts_; i++) {
    //     if (!is_on_ground_[i]) {
    //     } else {
    //         num_forces += 1;
    //     }
    // }

    orientation_ = orientation;

    // if (num_forces == 0) {

    //     for (int i=0; i<num_contacts_; i++) {
    //         F_[feet_frames_[i]] = Eigen::VectorXd::Zero(6);
    //     }
    //     return F_;
    // }


    // to_pinv_lin = Eigen::MatrixXd::Zero(model.nv-3, 3*num_forces);
    // to_pinv_ang = Eigen::MatrixXd::Zero(model.nv-3, 3*num_forces);
    

    // r_lin.head<3>() = r_ext.head<3>();
    // r_ang.head<3>() = r_ext.tail<3>();

    // r_lin.tail(model.nv-6) = r_int;
    // r_ang.tail(model.nv-6) = r_int;
    r.head<6>() = r_ext;
    r.tail(model.nv-6) = r_int;

    Eigen::MatrixXd orient_inv = orientation_.toRotationMatrix().transpose();
    Eigen::MatrixXd spatial_orient_inv = spatialTransform(orient_inv, Eigen::Vector3d::Zero());

    
    // int off = 0;

    // for (int i=0; i<num_contacts_; i++) {

    //     if (!is_on_ground_[i]) {
    //         off++;
    //         continue;
    //     } else {

    //         to_pinv_lin.block(0, (3*(i-off)), 3,3) = orient_inv;
    //         to_pinv_lin.block(3, (3*(i-off)), model.nv-6, 3) = J_lin[i].transpose() * orient_inv;

    //         to_pinv_ang.block(0, (3*(i-off)), 3,3) = orient_inv;
    //         to_pinv_ang.block(3, (3*(i-off)), model.nv-6, 3) = J_ang[i].transpose() * orient_inv;

    //     }
    // }

    for (size_t i=0; i<num_contacts_; i++) {

        to_pinv.block(0, (3*i), model.nv, 3) = J_fb[i].transpose() * orient_inv;

    }

    to_pinv.block(0, (3*num_contacts_), model.nv, 6) = J_w_fb.transpose() * spatial_orient_inv;


    if (is_pseudo) {

        forces = to_pinv.completeOrthogonalDecomposition().pseudoInverse() * r;

    } else {

        forces = to_pinv.inverse() * r;

    }
    // Eigen::VectorXd forces = to_pinv_lin.completeOrthogonalDecomposition().pseudoInverse() * r_lin;
    // Eigen::VectorXd torques = to_pinv_ang.completeOrthogonalDecomposition().pseudoInverse() * r_ang;

    // off = 0;

    // for (int i=0; i<num_contacts_; i++) {

    //     F_[feet_frames_[i]] = Eigen::VectorXd::Zero(6);

    //     if (!is_on_ground_[i]) {
    //         off+=1;
    //         continue;            
    //     } else {
    //         F_[feet_frames_[i]].head<3>() = forces.segment(3*(i-off), 3);

    //         if (include_torques) {
    //             F_[feet_frames_[i]].tail<3>() = torques.segment(3*(i-off), 3);
    //         }
    //     }
    // }
    for (size_t i=0; i<num_contacts_; i++) {
        F_[feet_frames_[i]].head<3>() = forces.segment(3*i, 3);
    }

    F_["base_wrench"] = forces.segment(3*num_contacts_, 6);


    return F_;

}




std::tuple<Eigen::VectorXd, Eigen::VectorXd> haptiquad::ForceEstimator::calculateResidualsFromForces(std::map<std::string, Eigen::VectorXd> forces) {

    if (!initialized) {
        throw std::runtime_error("[ForceEstimator]: Error - ForceEstimator has not been initialized yet.");
    }

    if (num_contacts_ == 0) {
        throw std::runtime_error("[ForceEstimator]: Error - number of contacts has not been set yet.");
    }

    if (num_contacts_ != forces.size()) {
        throw std::runtime_error("[ForceEstimator]: Error - calculateResidualsFromForces(): number of forces (" + std::to_string(forces.size()) + ") is inconsistent with the number of contacts (" + std::to_string(num_contacts_) + ").");
    }

    if (forces[feet_frames_[0]].size() != 6) {
        throw std::runtime_error("[ForceEstimator]: Error - calculateResidualsFromForces(): the forces are supposed to be spatial, with a dimension of 6.");
    }

    Eigen::VectorXd ext_r = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd int_r = Eigen::VectorXd::Zero(model.nv-6);

    Eigen::MatrixXd orient_inv = orientation_.toRotationMatrix().transpose();

    for (size_t i=0; i<num_contacts_; i++) {

        if (forces.find(feet_frames_[i]) == forces.end()) {
            throw std::runtime_error("[ForceEstimator]: Error - missing force data for " + feet_frames_[i]);
        }

        ext_r.head<3>() += orient_inv * forces[feet_frames_[i]].head<3>();
        ext_r.tail<3>() += orient_inv * forces[feet_frames_[i]].tail<3>();

        int_r += J_fb[i].transpose() * orient_inv * forces[feet_frames_[i]].head<3>();
        // int_r += J_lin[i].transpose() * orient_inv * forces[feet_frames_[i]].head<3>();
        // int_r += J_ang[i].transpose() * orient_inv * forces[feet_frames_[i]].tail<3>();

    }

    ext_r.head<3>() += orient_inv * forces["base_wrench"].head<3>();
    ext_r.tail<3>() += orient_inv * forces["base_wrench"].tail<3>();
    int_r += J_w_fb.transpose() * orient_inv * forces["base_wrench"];

    return std::make_tuple(int_r, ext_r);

}