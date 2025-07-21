#pragma once

#include <memory>

#include "lemlib/api.hpp"
#include "particle_filter.hpp"


template<size_t N>
class ParticleFilterChassis: public lemlib::Chassis {
public:
    explicit ParticleFilterChassis(const std::vector<Distance*>& pfSensors)
    : pf(ParticleFilter<N>(pfSensors))  {}

    void setPose(float x, float y, float theta, bool radians = false, bool resetParticles = true) {
        Chassis::setPose(x, y, theta, radians);
        if (resetParticles) {
            pf.initNormDist({x, y, getPose(true, true).theta});
        }
    }

    void odomUpdate(const lemlib::Pose& change) {
        lemlib::update(change);


        std::normal_distribution<> xDistribution(0, DRIVE_NOISE * std::fabs(change.x) + 0.001);
        std::normal_distribution<> yDistribution(0, DRIVE_NOISE * std::fabs(change.y) + 0.001);
        std::normal_distribution<> angleDistribution(0, ANGLE_NOISE * std::fabs(change.theta) + 0.001);

        static auto& randomGen = pf.getRandomGen();

        // No risk of dangling reference
        pf.update([&xDistribution, &yDistribution, &angleDistribution, &change]() {
            const auto noisyX = change.x + xDistribution(randomGen);
            const auto noisyY = change.y + yDistribution(randomGen);
            const auto noisyTheta = change.theta + angleDistribution(randomGen);

            return Eigen::Vector3f(noisyX, noisyY, noisyTheta);
        });

        // Set the pose to be the filters prediction
        const auto prediction = pf.getPrediction();
        setPose(prediction.x(), prediction.y(), M_PI_2 - prediction.z(), true, false);
    }


    ParticleFilter<N> pf;
    unsigned long long updateTimeMicros = 0;
};



