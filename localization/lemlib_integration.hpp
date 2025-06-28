#pragma once

#include <memory>

#include "lemlib/api.hpp"
#include "particle_filter.hpp"


template<size_t N>
class ParticleFilterChassis: public lemlib::Chassis {
public:
    explicit ParticleFilterChassis(std::vector<std::unique_ptr<Distance>>& pfSensors) {
        pf = std::make_unique<ParticleFilter<N>>(pfSensors);
    }

    void setPose(float x, float y, float theta, bool radians = false, bool resetParticles = true) {
        Chassis::setPose(x, y, theta, radians);
        if (resetParticles) {
            pf->initNormDist({x, y, getPose(true, true).theta});
        }
    }

    void odomUpdate(const lemlib::Pose& change) {
        lemlib::update(change);


        std::uniform_real_distribution<> xDistribution{change.x - DRIVE_NOISE * std::fabs(change.x),
                                                          change.x + DRIVE_NOISE * std::fabs(change.x)};
        std::uniform_real_distribution<> yDistribution{change.y - DRIVE_NOISE * std::fabs(change.y),
                                                    change.y + DRIVE_NOISE * std::fabs(change.y)};
        std::uniform_real_distribution angleDistribution(change.theta - ANGLE_NOISE * std::fabs(change.theta),
                                                          change.theta + ANGLE_NOISE * std::fabs(change.theta));

        static auto randomGen = pf->getRandomGen();

        pf->update([&]() {
            const auto noisyX = xDistribution(randomGen);
            const auto noisyY = yDistribution(randomGen);
            const auto noisyTheta = angleDistribution(randomGen);

            // Create a vector from noisyX and noisyY and rotate it by possible angular noise
            return Eigen::Vector3f(noisyX, noisyY, noisyTheta);
        });

        // Set the pose to be the filters prediction
        const auto prediction = pf->getPrediction();
        setPose(prediction.x(), prediction.y(), M_PI_2 - prediction.z(), true, false);
    }


    std::unique_ptr<ParticleFilter<N>> pf;
    unsigned long long updateTimeMicros= 0;
};



