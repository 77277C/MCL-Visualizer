#pragma once

#include <memory>

#include "lemlib/api.hpp"
#include "particle_filter.hpp"


template<size_t N>
class ParticleFilterChassis: public lemlib::Chassis {
public:
    explicit ParticleFilterChassis(std::vector<std::unique_ptr<Distance>>& pfSensors) {
        pf = std::make_unique<ParticleFilter<N>>(pfSensors, [this]() {
            return this->getPose(true, true).theta;
        });
    }

    void setPose(float x, float y, float theta, bool radians = false, bool resetParticles = true) {
        Chassis::setPose(x, y, theta, radians);
        if (resetParticles) {
            pf->initNormDist({x, y});
        }
    }

    void odomUpdate(const lemlib::Pose& change) {
        constexpr bool radians = false;

        lemlib::update(change);
        const lemlib::Pose after = getPose(radians);

        std::uniform_real_distribution<> xDistribution{change.x - DRIVE_NOISE * std::fabs(change.x),
                                                          change.x + DRIVE_NOISE * std::fabs(change.x)};
        std::uniform_real_distribution<> yDistribution{change.y - DRIVE_NOISE * std::fabs(change.y),
                                                    change.y + DRIVE_NOISE * std::fabs(change.y)};

        // Angle distribution doesn't factor in current theta because odom change is already rotated
        // to be in (x, y) format
        static std::uniform_real_distribution angleDistribution(-ANGLE_NOISE, ANGLE_NOISE);
        static auto randomGen = pf->getRandomGen();

        pf->update([&]() {
            const auto noisyX = xDistribution(randomGen);
            const auto noisyY = yDistribution(randomGen);
            const auto noisyAngleDelta = angleDistribution(randomGen);

            // Create a vector from noisyX and noisyY and rotate it by possible angular noise
            return Eigen::Rotation2Df(noisyAngleDelta) * Eigen::Vector2f({noisyX, noisyY});
        });

        const auto prediction = pf->getPrediction();
        setPose(prediction.x(), prediction.y(), after.theta, radians, false);
    }

    std::unique_ptr<ParticleFilter<N>> pf;
};



