#define GLM_ENABLE_EXPERIMENTAL 
#include <stdlib.h>
#include <tuple>
#include <math.h>
#include "attitude_estimator.hpp"

using glm::vec3; using glm::fquat; using glm::normalize; using glm::cross; using glm::length; using glm::normalize;


void AttitudeEstimator::madgwickMarg(const vec3& angularRate, const vec3& accelerations, const vec3& northHeading, float dt, float beta_g, float beta_b) {
    vec3 a_norm = normalize(accelerations);
    vec3 n_norm = normalize(northHeading);

    float qw = attitude_.w;
    float qx = attitude_.x;
    float qy = attitude_.y;
    float qz = attitude_.z;

    float _2qw = 2.0f * qw;
    float _2qx = 2.0f * qx;
    float _2qy = 2.0f * qy;
    float _2qz = 2.0f * qz;
    
    float _4qw = 4.0f * qw;
    float _4qx = 4.0f * qx;
    float _4qy = 4.0f * qy;
    float _4qz = 4.0f * qz;
    
    float _2qxx = _2qx * qx; float _2qyy = _2qy * qy; float _2qzz = _2qz * qz;
    float _2qwx = _2qw * qx; float _2qwy = _2qw * qy; float _2qwz = _2qw * qz;
    float _2qxy = _2qx * qy; float _2qxz = _2qx * qz; float _2qyz = _2qy * qz;

    vec3 bodyG(
        _2qxz - _2qwy, 
        _2qwx + _2qyz, 
        1.0f - _2qxx - _2qyy
    );
    vec3 f_err_g = bodyG - a_norm;

    fquat gradient_G(
       -_2qy * f_err_g.x + _2qx * f_err_g.y,
        _2qz * f_err_g.x + _2qw * f_err_g.y - _4qx * f_err_g.z,
       -_2qw * f_err_g.x + _2qz * f_err_g.y - _4qy * f_err_g.z,
        _2qx * f_err_g.x + _2qy * f_err_g.y
    );

    
    vec3 h(
        n_norm.x * (1.0f - _2qyy - _2qzz) + n_norm.y * (_2qxy - _2qwz) + n_norm.z * (_2qxz + _2qwy),
        n_norm.x * (_2qxy + _2qwz) + n_norm.y * (1.0f - _2qxx - _2qzz) + n_norm.z * (_2qyz - _2qwx),
        n_norm.x * (_2qxz - _2qwy) + n_norm.y * (_2qyz + _2qwx) + n_norm.z * (1.0f - _2qxx - _2qyy)
    );
    float bx = sqrt(h.x * h.x + h.y * h.y);
    vec3 bodyB(
        bx * (1.0f - _2qyy - _2qzz) + h.z * (_2qxz - _2qwy),
        bx * (_2qxy - _2qwz) + h.z * (_2qyz + _2qwx),
        bx * (_2qxz + _2qwy) + h.z * (1.0f - _2qxx - _2qyy)
    );
    vec3 f_err_b = bodyB - n_norm;

    vec3 e = normalize(cross(a_norm, n_norm));
    vec3 bodyE(
        _2qxy + _2qwz,
        1.0f - _2qxx - _2qzz,
        _2qyz - _2qwx
    );
    vec3 f_err_e = bodyE - e;
    fquat gradient_E(
        _2qz * f_err_e.x - _2qx * f_err_e.z,
        _2qy * f_err_e.x - _2qw * f_err_e.z - _4qx * f_err_e.y,
        _2qx * f_err_e.x + _2qz * f_err_e.z,
        _2qw * f_err_e.x + _2qy * f_err_e.z - _4qz * f_err_e.y
    );


    // float gnorm = length(gradient_G);
    // if (gnorm > 1e-9f) gradient_G = gradient_G / gnorm;
    // float enorm = length(gradient_E);
    // if (enorm > 1e-9f) gradient_E = gradient_E / enorm;

    fquat q_omega = 0.5f * attitude_ * fquat(0.0, angularRate.x, angularRate.y, angularRate.z);   
    q_omega -= (beta_g * gradient_G) + (beta_b * gradient_E);
    attitude_ += q_omega * dt;
    attitude_ = normalize(attitude_);
}

void AttitudeEstimator::madgwickIMU(const vec3& angularRate, const vec3& accelerations, float dt, float beta_g) {
    vec3 a_norm = normalize(accelerations);

    float qw = attitude_.w;
    float qx = attitude_.x;
    float qy = attitude_.y;
    float qz = attitude_.z;

    float _2qw = 2.0f * qw;
    float _2qx = 2.0f * qx;
    float _2qy = 2.0f * qy;
    float _2qz = 2.0f * qz;
    
    float _4qw = 4.0f * qw;
    float _4qx = 4.0f * qx;
    float _4qy = 4.0f * qy;
    float _4qz = 4.0f * qz;
    
    float _2qxx = _2qx * qx; float _2qyy = _2qy * qy; float _2qzz = _2qz * qz;
    float _2qwx = _2qw * qx; float _2qwy = _2qw * qy; float _2qwz = _2qw * qz;
    float _2qxy = _2qx * qy; float _2qxz = _2qx * qz; float _2qyz = _2qy * qz;

    vec3 bodyG(
        _2qxz - _2qwy, 
        _2qwx + _2qyz, 
        1.0f - _2qxx - _2qyy
    );
    vec3 f_err_g = bodyG - a_norm;

    fquat gradient_G(
       -_2qy * f_err_g.x + _2qx * f_err_g.y,
        _2qz * f_err_g.x + _2qw * f_err_g.y - _4qx * f_err_g.z,
       -_2qw * f_err_g.x + _2qz * f_err_g.y - _4qy * f_err_g.z,
        _2qx * f_err_g.x + _2qy * f_err_g.y
    );

    float gnorm = length(gradient_G);
    if (gnorm > 1e-9f) gradient_G = gradient_G / gnorm;

    fquat q_omega = 0.5f * attitude_ * fquat(0.0, angularRate.x, angularRate.y, angularRate.z);   
    q_omega -= (beta_g * gradient_G);
    attitude_ += q_omega * dt;
    attitude_ = normalize(attitude_);
}