#pragma once

#include <cmath>
#include <optional>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "constants.hpp"

inline float angleDiff(float a, float b) {
    return std::remainder(b - a, 2 * M_PI);
}

// Cheap approximation of normpdf (normal distribution PDF)
float normPdf(float x, float mu = 0.0, float sigma = 1.0) {
    constexpr float invSqrt2Pi = 0.3989422804014327; // 1 / sqrt(2 * pi)
    float z = (x - mu) / sigma;
    return (invSqrt2Pi / sigma) * std::exp(-0.5 * z * z);
}

class Distance {
public:
    explicit Distance(Eigen::Vector3f offset)
        : distance(0), stddev(0), offset(std::move(offset)) {}

    void update(const float dist, const float conf) {
        distance = dist;
        stddev = 0.2 * distance / std::sqrt(conf / 64.0);
    }

     [[nodiscard]] std::optional<float> getProbability(const Eigen::Vector3f& pose) const {
        if (distance > 9999 * MM_TO_IN) {
            return std::nullopt;
        }

        // Rotate only the x,y part of the offset by the particle heading (pose.z())
        Eigen::Vector2f rotatedOffset =  Eigen::Rotation2Df(pose.z()) * offset.head<2>();

        // Compose the new pose by adding rotated offset to position and adding offset.z() to angle
        Eigen::Vector3f position = pose;
        position.head<2>() += rotatedOffset;
        position.z() += offset.z();

        const auto angle = position.z();

        // Predict the value of the distance sensor by finding the closest wall to it
        // then dividing the distance from the wall by cos(theta) to find the hypotenuse
        // which represents the predicted distance sensor reading
        float predicted = 50.0;

        if (const auto theta = std::fabs(angleDiff(0, angle)); theta < M_PI_2) {
            predicted = std::min(predicted, (WALL - position.x()) / std::cos(theta));
        }

        if (const auto theta = std::fabs(angleDiff(M_PI_2, angle)); theta < M_PI_2) {
            predicted = std::min(predicted, (WALL - position.y()) / std::cos(theta));
        }

        if (const auto theta = std::fabs(angleDiff(M_PI, angle)); theta < M_PI_2) {
            predicted = std::min(predicted, (position.x() + WALL) / std::cos(theta));
        }

        if (const auto theta = std::fabs(angleDiff(3 * M_PI_2, angle)); theta < M_PI_2) {
            predicted = std::min(predicted, (position.y() + WALL) / std::cos(theta));
        }

        return normPdf(distance, predicted, stddev);
    }


    Eigen::Vector3f offset;

    float distance;
    float stddev;
};
