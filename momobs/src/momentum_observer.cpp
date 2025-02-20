#include <momobs/momentum_observer.hpp>

momobs::MomentumObserver::MomentumObserver() {}

void momobs::MomentumObserver::setInternalGain(float k_int) {

    //Throw error
    if (!initialized) {
        throw std::runtime_error("[MomentumObserver]: Error - MomentumObserver has not been initialized yet.");
    }

    K_int = Eigen::MatrixXd::Identity(model.nv-6, model.nv-6) * k_int;

}



void momobs::MomentumObserver::setExternalGain(float k_ext) {

    if (!initialized) {
        throw std::runtime_error("[MomentumObserver]: Error - MomentumObserver has not been initialized yet.");
    }

    K_ext = Eigen::MatrixXd::Identity(6, 6) * k_ext;

}



void momobs::MomentumObserver::initModel(pinocchio::Model pin_model) {

    model = pin_model;
    data = pinocchio::Data(model);


    q_ = Eigen::VectorXd::Zero(model.nq);
    q_dot_ = Eigen::VectorXd::Zero(model.nv);
    torques_ = Eigen::VectorXd::Zero(model.nv-6);

    integral_int = Eigen::VectorXd::Zero(model.nv-6);
    r_int = Eigen::VectorXd::Zero(model.nv -6);
        
    integral_ext = Eigen::VectorXd::Zero(6);
    r_ext = Eigen::VectorXd::Zero(6);

    K_int = Eigen::MatrixXd::Identity(model.nv-6, model.nv-6) * k_int;
    K_ext = Eigen::MatrixXd::Identity(6, 6) * k_ext;

    H = Eigen::MatrixXd::Zero(model.nv-6, model.nv-6);
    H_dot = Eigen::MatrixXd::Zero(model.nv-6, model.nv-6);
    F = Eigen::MatrixXd::Zero(6, model.nv-6);
    F_dot = Eigen::MatrixXd::Zero(6, model.nv-6);
    IC = Eigen::MatrixXd::Zero(6,6);
    IC_dot = Eigen::MatrixXd::Zero(6,6);
    H_fb = Eigen::MatrixXd::Zero(model.nv-6, model.nv-6);
    H_dot_fb = Eigen::MatrixXd::Zero(model.nv-6, model.nv-6);

    C = Eigen::VectorXd::Zero(model.nv-6);
    p0c = Eigen::VectorXd::Zero(6);
    C_fb = Eigen::VectorXd::Zero(model.nv-6);

    initialized = true;

}



void momobs::MomentumObserver::updateJointStates(JointStateDict q, JointStateDict q_dot, JointStateDict torques) {

    if (!initialized) {
        throw std::runtime_error("[MomentumObserver]: Error - MomentumObserver has not been initialized yet.");
    }

    if (q.size() != static_cast<size_t>(model.nv-6) || 
        q_dot.size() != static_cast<size_t>(model.nv-6) || 
        torques.size() != static_cast<size_t>(model.nv-6)) 
    {
        throw std::runtime_error("[MomentumObserver]: Error - updateJointStates(): arguments size does not match model size.");
    }

    //TODO: Check for matching names in the map

    for (int i=0; i<q.size(); i++) {

        q_(i+7) = q[model.names[i+2]];
        q_dot_(i+6) = q_dot[model.names[i+2]];
        torques_(i) = torques[model.names[i+2]];

    }

}




void momobs::MomentumObserver::updateBaseState(Eigen::VectorXd v0, Eigen::Quaterniond orientation) {

    if (!initialized) {
        throw std::runtime_error("[MomentumObserver]: Error - MomentumObserver has not been initialized yet.");
    }

    if (v0.size() != 6) {
        throw std::runtime_error("[MomentumObserver]: Error - updateBaseState(): v0 is expected to be a spatial velocity with dimension 6");
    }

    q_(3) = orientation.x();
    q_(4) = orientation.y();
    q_(5) = orientation.z();
    q_(6) = orientation.w();

    q_dot_.head<6>() << v0;

}


std::tuple<Eigen::VectorXd, Eigen::VectorXd> momobs::MomentumObserver::getResiduals(double dt) {

    if (!initialized) {
        throw std::runtime_error("[MomentumObserver]: Error - MomentumObserver has not been initialized yet.");
    }

    //TODO: lighten computating functions
    pinocchio::computeAllTerms(model, data, q_, q_dot_);
    pinocchio::computeCoriolisMatrix(model, data, q_, q_dot_);

    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    pin_H_dot = data.C + data.C.transpose();

    IC = data.M.block(0,0, 6, 6);
    F = data.M.block(0,6, 6, model.nv-6);
    H = data.M.block(6,6, model.nv-6, model.nv-6);

    C = data.nle.tail(model.nv-6);
    p0c = data.nle.head<6>();

    H_dot = pin_H_dot.block(6, 6, model.nv-6, model.nv-6);
    F_dot = pin_H_dot.block(0, 6, 6, model.nv-6);
    IC_dot = pin_H_dot.block(0, 0, 6, 6);

    C_fb = C - F.transpose() * IC.inverse() * p0c;
    H_fb = H - F.transpose() * IC.inverse() * F;
    H_dot_fb = H_dot - F_dot.transpose() * IC.inverse() * F - F.transpose() * IC.inverse() * F_dot
                            - F.transpose() * (-IC.inverse() * IC_dot * IC.inverse()) * F;

    torques_ = subtractFrictionTorque(torques_);

    if (dt > threshold_) {
        scale_factor = expected_dt_ / dt;
    }

    if (!rescale) { scale_factor = 1.0; }


    integral_int += (torques_ + H_dot_fb * q_dot_.tail(model.nv-6) - C_fb + r_int) * dt * scale_factor;
    integral_ext += (IC_dot * q_dot_.head<6>() + F_dot * q_dot_.tail(model.nv - 6) - p0c + r_ext) * dt * scale_factor;

    r_int = K_int * (H_fb * q_dot_.tail(model.nv - 6) - integral_int); 
    r_ext = K_ext * (IC * q_dot_.head<6>() + F * q_dot_.tail(model.nv-6) - integral_ext);


    return std::make_tuple(r_int, r_ext);

}


void momobs::MomentumObserver::disableTimeScaling() {
    rescale = false;
}

void momobs::MomentumObserver::enableTimeScaling(double expected_dt, double threshold) {

    rescale = true;
    expected_dt_ = expected_dt;
    threshold_ = threshold;

}


Eigen::VectorXd momobs::MomentumObserver::subtractFrictionTorque(Eigen::VectorXd torques) {

    //TODO: implement friction model
    
    return torques;
}