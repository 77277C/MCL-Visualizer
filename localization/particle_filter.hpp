#pragma once

#include <random>

#include "distance.hpp"
#include "constants.hpp"


typedef struct {
    Eigen::Vector3f location = Eigen::Vector3f::Zero();
    float weight = 1.0;
} Particle;


template<size_t N>
class ParticleFilter {
public:
    explicit ParticleFilter(const std::vector<Distance*>& sensors)
        : sensors(sensors) {}

    [[nodiscard]] Eigen::Vector3f getPrediction() const {
        return prediction;
    }

    [[nodiscard]] std::ranlux24_base& getRandomGen() {
        return randomGen;
    }

    void update(
        const std::function<Eigen::Vector3f()>& getPrediction
    ) {
        // Add the prediction to each particle
        for (auto& particle : particles) {
            particle.location += getPrediction();
        }

        distanceSinceUpdate += getPrediction().norm();
        // Return if robot hasn't moved to avoid particle convergence
        if (
            distanceSinceUpdate < 2.0
        ) {
            // Still form a prediction to ensure the moved distance from odom is accounted for
            prediction = formPrediction();
            return;
        }

        for (size_t i = 0; i < N; i++) {
            // Place particle at random point in field if out of field
            if (!isPoseInField(particles[i].location)) {
                particles[i].location.x() = fieldDistribution(randomGen);
                particles[i].location.y() = fieldDistribution(randomGen);
            }

            // Weight the particle
            particles[i].weight = weighParticle(particles[i].location);
        }

        resample();

        distanceSinceUpdate = 0.0;
    }

    [[nodiscard]] float weighParticle(const Eigen::Vector3f& particle) const {
        float combinedWeight = 1.0;

        // Multiply the combined weight by the probability of each sensor
        for (auto& sensor : sensors) {
            const auto weight = sensor->getProbability(particle);
            if (weight.has_value()) {
                combinedWeight *= weight.value();
            }
        }

        return combinedWeight;
    }

    static bool isPoseInField(const Eigen::Vector3f& pose) {
        return (
            pose.x() < WALL &&
            pose.y() < WALL &&
            pose.x() > -WALL &&
            pose.y() > -WALL
        );
    }

    void resample() {
        float totalWeight = 0.0;

        // Sum all particle weights and make a copy of the current set.
        // Think of each particle's weight as a slice in a pie chart of "trust" — bigger slices = more believable.
        for (size_t i = 0; i < N; i++) {
            totalWeight += particles[i].weight;
            oldParticles[i] = particles[i];
        }

        // Compute the average weight across all particles.
        // This defines the spacing between our sampling points on the pie chart.
        const float avgWeight = totalWeight / static_cast<float>(N);

        // Choose a random starting offset within [0, avg_weight)
        // This is like randomly placing the first pointer on the pie chart.
        std::uniform_real_distribution<> distribution{0.0, avgWeight};
        const auto randomWeight = distribution(randomGen);

        size_t j = 0;
        float cumulativeWeight = 0.0;

        // Resample N particles using low-variance resampling.
        // We place N evenly spaced markers on the cumulative weight pie chart, starting from random_weight.
        for (size_t i = 0; i < N; i++) {
            const auto weight = static_cast<float>(i) * avgWeight + randomWeight;

            // Walk through the cumulative weights until we find the particle that spans the current weight.
            // Particles with bigger weight slices will get hit more often — they're more likely to survive.
            while (cumulativeWeight < weight) {
                if (j >= N) {
                    break;
                }
                cumulativeWeight += particles[j].weight;
                j++;
            }

            // Copy the selected particle's location into the new set.
            // This is effectively cloning a "good guess" multiple times.
            particles[i].location.x() = oldParticles[j-1].location.x();
            particles[i].location.y() = oldParticles[j-1].location.y();
            particles[i].location.z() = oldParticles[j-1].location.z();
        }

        // Estimate the new predicted position as the mean of all resampled particles.
        prediction = formPrediction();
    }

    Eigen::Vector3f formPrediction() {
        float xSum = 0.0;
        float ySum = 0.0;
        float thetaSum = 0.0;

        for (auto& particle : particles) {
            xSum += particle.location.x();
            ySum += particle.location.y();
            thetaSum += particle.location.z();
        }

        // Estimate the new predicted position as the mean of all resampled particles.
        return {xSum / static_cast<float>(N), ySum / static_cast<float>(N), thetaSum / static_cast<float>(N)};
    }

    void initNormDist(const Eigen::Vector3f& mean) {
        std::normal_distribution<> posDist{0, STARTING_POS_VARIANCE};
        std::normal_distribution<> angleDist{0, STARTING_ANGLE_VARIANCE};

        for (auto &&particle : this->particles) {
            particle.location = mean + Eigen::Vector3f(posDist(randomGen), posDist(randomGen), angleDist(randomGen));
        }

        distanceSinceUpdate += 2.0 * distanceSinceUpdate;
    }


    Eigen::Vector3f prediction{};

    std::array<Particle, N> particles;
    std::array<Particle, N> oldParticles;
    std::vector<Distance*> sensors;

    std::ranlux24_base randomGen;
    std::uniform_real_distribution<> fieldDistribution{-WALL, WALL};

    float distanceSinceUpdate = 0.0;
};
