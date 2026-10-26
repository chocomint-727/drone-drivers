#pragma once
#include <stdlib.h>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

using glm::vec3; using glm::fquat; using glm::vec4; using glm::mat3;using glm::mat4; using glm::cross; using glm::inverse;

class QuadModel {
    public:
        inline QuadModel(float mass, fquat attitude_, mat3 inertia_, mat4 motorMatrix_, double gravity = 9.81) {
            m = mass;
            attitude = attitude_;
            inertia = inertia_;
            motorMatrixInv = inverse(motorMatrix_);
            motorPWM = vec4();
            g_ = glm::vec3(0, 0, -gravity);
        }

        int setGains(const mat3& wKP, const mat3& wKD);

        void updateAttitude(const fquat& attitude_);
        vec3 feedforwardTau(const fquat& targetQuat, const vec3& w) const;
        void performMotorMixing(const float T, const vec3& tau);
        std::tuple<float, fquat> applyFlatnessMap(const glm::vec3& a_net_world) const;
        std::tuple<float, fquat> applyFlatnessMap(const glm::vec3& a_net_world, const glm::vec3& yaw) const;

    private:
        float m;
        
        vec3 g_;
        fquat attitude;
        mat3 inertia;
        mat4 motorMatrixInv;
        mat3 wKP;
        mat3 wKD;
        vec4 motorPWM;
};