#include <momobs/spatial_momentum_observer.hpp>


bool spatial::momobs::MomentumObserver::setInteralGain(float k_int) {

    //Throw error
    if (!initialized) { return false; }

    K_int = Eigen::MatrixXd::Identity(model.n, model.n) * k_int;
    return true;

}



bool spatial::momobs::MomentumObserver::setExternalGain(float k_ext) {

    //Throw error
    if (!initialized) { return false; }

    K_ext = Eigen::MatrixXd::Identity(6, 6) * k_ext;
    return true;

}



void spatial::momobs::MomentumObserver::initModel(pinocchio::Model pin_model) {

    model = model::SpatialFloatingModel(pin_model, true);

    q_ = Eigen::VectorXd::Zero(model.n);
    q_dot_ = Eigen::VectorXd::Zero(model.n);
    torques_ = Eigen::VectorXd::Zero(model.n);

    integral_int = Eigen::VectorXd::Zero(model.n);
    r_int = Eigen::VectorXd::Zero(model.n);
        
    integral_ext = Eigen::VectorXd::Zero(6);
    r_ext = Eigen::VectorXd::Zero(6);

    K_int = Eigen::MatrixXd::Identity(model.n, model.n) * k_int;
    K_ext = Eigen::MatrixXd::Identity(6, 6) * k_ext;

    H = Eigen::MatrixXd::Zero(model.n, model.n);
    H_dot = Eigen::MatrixXd::Zero(model.n, model.n);
    F = Eigen::MatrixXd::Zero(6, model.n);
    F_dot = Eigen::MatrixXd::Zero(6, model.n);
    IC = Eigen::MatrixXd::Zero(6,6);
    IC_dot = Eigen::MatrixXd::Zero(6,6);
    H_fb = Eigen::MatrixXd::Zero(model.n, model.n);
    H_dot_fb = Eigen::MatrixXd::Zero(model.n, model.n);

    C = Eigen::VectorXd::Zero(model.n);
    p0c = Eigen::VectorXd::Zero(6);
    C_fb = Eigen::VectorXd::Zero(model.n);

    initialized = true;

}



bool spatial::momobs::MomentumObserver::updateJointStates(JointStateDict q, JointStateDict q_dot, JointStateDict torques) {

    if (!initialized) { return false; }

    if (q.size() != static_cast<size_t>(model.n) || 
        q_dot.size() != static_cast<size_t>(model.n) || 
        torques.size() != static_cast<size_t>(model.n)) 
    {
        return false;
    }

    //TODO: Check for matching names in the map

    for (int i=0; i<q.size(); i++) {

        q_(i) = q[model.names[i]];
        q_dot_(i) = q_dot[model.names[i]];
        torques_(i) = torques[model.names[i]];

    }

    return true;

}




bool spatial::momobs::MomentumObserver::updateBaseState(Eigen::VectorXd v0, Eigen::Quaterniond orientation) {

    if (!initialized) { return false; }

    if (v0.size() != 6) { return false; }

    q_(3) = orientation.x();
    q_(4) = orientation.y();
    q_(5) = orientation.z();
    q_(6) = orientation.w();

    q_dot_.head<6>() << v0;

    return true;

}


std::tuple<Eigen::VectorXd, Eigen::VectorXd> spatial::momobs::MomentumObserver::getResiduals(double dt) {

    if (!initialized) {
        return std::make_tuple(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));
    }

    std::tie(C, p0c) = dynamics::fb_rnea(model, q_, q_dot_);
    std::tie(H, H_dot, F, F_dot, IC, IC_dot) = dynamics::fb_crba(model, q_, q_dot_);

    C_fb = C - F.transpose() * IC.inverse() * p0c;
    H_fb = H - F.transpose() * IC.inverse() * F;
    H_dot_fb = H_dot - F_dot.transpose() * IC.inverse() * F - F.transpose() * IC.inverse() * F_dot
                            - F.transpose() * (-IC.inverse() * IC_dot * IC.inverse()) * F;


    //TODO: Add friction model
    if (dt > threshold_) {
        scale_factor = expected_dt_ / dt;
    }

    if (!rescale) { scale_factor = 1.0; }


    integral_int += (torques_ + H_dot_fb * q_dot_.tail(model.n) - C_fb + r_int) * dt * scale_factor;
    integral_ext += (IC_dot * q_dot_.head<6>() + F_dot * q_dot_.tail(model.n) - p0c + r_ext) * dt * scale_factor;

    r_int = K_int * (H_fb * q_dot_.tail(model.n) - integral_int); 
    r_ext = K_ext * (IC * q_dot_.head<6>() + F * q_dot_.tail(model.n) - integral_ext);


    return std::make_tuple(r_int, r_ext);

}


void spatial::momobs::MomentumObserver::disableTimeScaling() {
    rescale = false;
}

void spatial::momobs::MomentumObserver::enableTimeScaling(double expected_dt, double threshold) {

    rescale = true;
    expected_dt_ = expected_dt;
    threshold_ = threshold;

}