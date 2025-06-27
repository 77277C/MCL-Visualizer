#pragma once
#include <cmath>
#include <string>


constexpr float radToDeg(float rad) { return rad * 180 / M_PI; }
constexpr float degToRad(float deg) { return deg * M_PI / 180; }

namespace lemlib {

class Pose {
    public:
        float x;
        float y;
        float theta;

        Pose(float x, float y, float theta = 0) : x(x), y(y), theta(theta) {}

        Pose operator-(const Pose& other) const {
            return {x - other.x, y - other.y, theta - other.theta};
        }
        Pose operator+=(const Pose& other) {
            x += other.x;
            y += other.y;
            theta += other.theta;
            return *this;
        }

};

/**
 * @brief Format a pose
 *
 * @param pose
 * @return std::string
 */
std::string format_as(const Pose& pose);
} // namespace lemlib

lemlib::Pose odomPose(0, 0, 0); // the pose of the robot
namespace lemlib {

    Pose getPose(bool radians = false) {
        if (radians) return odomPose;
        else return Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
    }

    void setPose(Pose pose, bool radians = false) {
        if (radians) odomPose = pose;
        else odomPose = lemlib::Pose(pose.x, pose.y, degToRad(pose.theta));
    }

    void update(Pose pose) {
        odomPose += pose;
    }
} // namespace lemlib

namespace lemlib {

    class Chassis {
    public:
        void setPose(float x, float y, float theta, bool radians) {
            setPose(Pose(x, y, theta), radians);
        }

        void setPose(Pose pose, bool radians) { lemlib::setPose(pose, radians); }

        Pose getPose(bool radians = false, bool standardPos = false) {
            Pose pose = lemlib::getPose(true);
            if (standardPos) pose.theta = M_PI_2 - pose.theta;
            if (!radians) pose.theta = radToDeg(pose.theta);
            return pose;
        }
    };
}