#pragma once
#include <stdlib.h>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

using glm::vec3; using glm::fquat; using glm::vec4; using glm::mat3;using glm::mat4; using glm::cross; using glm::inverse;

class AttitudeEstimator {
    public:

        inline AttitudeEstimator(double gravity = 9.81) {
            g_ = glm::vec3(0, 0, -gravity);
        }

        fquat madgwickMarg(const fquat& attitude_, const vec3& angularRate, const vec3& accelerations, const vec3& northHeading, float dt, float beta_g, float beta_b) const;
        fquat madgwickIMU(const fquat& attitude_, const vec3& angularRate, const vec3& accelerations, float dt, float beta_g) const;

    private:
        vec3 g_;
};