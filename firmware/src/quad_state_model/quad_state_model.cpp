#define GLM_ENABLE_EXPERIMENTAL 
#include <stdlib.h>
#include <tuple>
#include <math.h>
#include "quad_state_model.hpp"

using glm::vec3; using glm::fquat; using glm::normalize; using glm::cross; using glm::length; using glm::normalize;

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void QuadModel::updateAttitude(const fquat& attitude_){
    attitude = attitude_;
}

vec3 QuadModel::feedforwardTau(const fquat& targetQuat, const vec3& w) const {
    fquat q_err = glm::conjugate(attitude) * targetQuat;
    vec3 err_vec(2*q_err.x * sgn(q_err.w), 2*q_err.y * sgn(q_err.w), 2*q_err.z * sgn(q_err.w));

    return inertia * (wKP * err_vec - wKD * w) + cross(w, inertia*w);
}

void QuadModel::performMotorMixing(const float T, const vec3& tau) {
    motorPWM = motorMatrixInv * vec4(T, tau);
}

std::tuple<float, fquat> QuadModel::applyFlatnessMap(const vec3& a_net_world, const vec3& yaw) const {
    vec3 t = m * (a_net_world - g_);
    
    vec3 h(yaw.x, yaw.y, 0.0);
    if (length(h) < 1e-6) h = vec3(1.0, 0.0, 0.0);

    float T = length(t);
    vec3 z = normalize(t);
    vec3 y = normalize(cross(z, h));
    vec3 x = cross(y, z);

    mat3 R(x, y, z);

    return {T, normalize(glm::quat_cast(R))};
}

std::tuple<float, fquat> QuadModel::applyFlatnessMap(const vec3& a_net_world) const {
    return QuadModel::applyFlatnessMap(a_net_world, a_net_world);
}