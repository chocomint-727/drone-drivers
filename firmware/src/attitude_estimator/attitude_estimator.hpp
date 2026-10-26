#pragma once
#include <stdlib.h>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

using glm::vec3; using glm::fquat; using glm::vec4; using glm::mat3;using glm::mat4; using glm::cross; using glm::inverse;

class AttitudeEstimator {
    public:
        fquat attitude_;

        inline AttitudeEstimator(fquat attitude, double gravity = 9.81) {
            attitude_ = attitude;
            g_ = glm::vec3(0, 0, -gravity);
        }

        void madgwickMarg(const vec3& angularRate, const vec3& accelerations, const vec3& northHeading, float dt, float beta_g, float beta_b);
        void madgwickIMU(const vec3& angularRate, const vec3& accelerations, float dt, float beta_g);

    private:
        vec3 g_;
};