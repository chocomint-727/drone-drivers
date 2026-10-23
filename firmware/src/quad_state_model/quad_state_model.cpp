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

void QuadModel::updateAttitude(const vec3& angularRate, const vec3& accelerations, const vec3& northHeading, float dt, float beta_g, float beta_b) {
    vec3 a_norm = normalize(accelerations);
    vec3 n_norm = normalize(northHeading);

    // prep commonly used quaternion terms for easier code writing
    float q1 = attitude.w;
    float q2 = attitude.x;
    float q3 = attitude.y;
    float q4 = attitude.z;

    float _2q1 = 2.0 * q1;
    float _2q2 = 2.0 * q2;
    float _2q3 = 2.0 * q3;
    float _2q4 = 2.0 * q4;
    
    float _4q2 = 4.0 * q2;
    float _4q3 = 4.0 * q3;

    float _2q22 = _2q2 * q2; float _2q33 = _2q3 * q3; float _2q44 = _2q4 * q4;
    float _2q12 = _2q1 * q2; float _2q13 = _2q1 * q3; float _2q14 = _2q1 * q4;
    float _2q23 = _2q2 * q3; float _2q24 = _2q2 * q4; float _2q34 = _2q3 * q4;

    vec3 bodyG(
        _2q24 - _2q13, 
        _2q12 + _2q34, 
        1.0f - _2q22 - _2q33
    );
    vec3 f_err_g = bodyG - a_norm;

    vec3 h(
        n_norm.x * (1.0f - _2q33 - _2q44) + n_norm.y * (_2q23 - _2q14) + n_norm.z *(_2q24 + _2q13),
        n_norm.x * (_2q23 + _2q14) + n_norm.y * (1.0f - _2q22 - _2q44) + n_norm.z * (_2q34 - _2q12),
        n_norm.x * (_2q24 - _2q13) + n_norm.y * (_2q34 + _2q1) + n_norm.z * (1.0f - _2q22 - _2q33)
    );
    vec3 b(sqrt(h.x * h.x + h.y * h.y), 0.0, h.z);
    vec3 bodyB(
        b.x * (1.0f - _2q33 - _2q44) + b.z * (_2q24 - _2q13),
        b.x * (_2q23 - _2q14) + b.z * (_2q34 + _2q12),
        b.x * (_2q24 + _2q13) + b.z * (1.0f - _2q22 - _2q33)
    );
    vec3 f_err_b = bodyB - n_norm;

    fquat gradient_G(
       -_2q3 * f_err_g.x + _2q2 * f_err_g.y,
        _2q4 * f_err_g.x + _2q1 * f_err_g.y - _4q2 * f_err_g.z,
       -_2q1 * f_err_g.x + _2q4 * f_err_g.y - _4q3 * f_err_g.z,
        _2q2 * f_err_g.x + _2q3 * f_err_g.y
    );

    fquat gradient_B(
        -_2q3 * f_err_b.x + _2q2 * f_err_b.y,
         _2q4 * f_err_b.x + _2q1 * f_err_b.y - _4q2 * f_err_b.z,
        -_2q1 * f_err_b.x + _2q4 * f_err_b.y - _4q3 * f_err_b.z,
         _2q2 * f_err_b.x + _2q3 * f_err_b.y
    );

    float gnorm = length(gradient_G);
    if (gnorm > 1e-9f) gradient_G = gradient_G / gnorm;
    float bnorm = length(gradient_B);
    if (bnorm > 1e-9f) gradient_B = gradient_B / bnorm;

    fquat q_omega = 0.5f * attitude * fquat(0.0, angularRate.x, angularRate.y, angularRate.z);   
    q_omega -= (beta_g * gradient_G) + (beta_b * gradient_B);
    attitude += q_omega * dt;
    attitude = normalize(attitude);
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