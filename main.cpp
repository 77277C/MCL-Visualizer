#define _USE_MATH_DEFINES
#include <iostream>
#include "localization/lemlib_integration.hpp"


int main() {
    std::unique_ptr<Distance> left = std::make_unique<Distance>(Eigen::Vector3f(0, 0, M_PI_2));
    std::unique_ptr<Distance> front = std::make_unique<Distance>(Eigen::Vector3f(0, 0, 0));
    std::unique_ptr<Distance> right = std::make_unique<Distance>(Eigen::Vector3f(0, 0, -M_PI_2));

    std::vector<std::unique_ptr<Distance>> distances;
    distances.push_back(std::move(left));
    distances.push_back(std::move(front));
    distances.push_back(std::move(right));

    ParticleFilterChassis<500> chassis(distances);
    chassis.setPose(0, 0, 90, false, true);

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line == "exit") break;

        std::istringstream iss(line);
        std::string code;
        iss >> code;


        if (code == "change") {
            float dx, dy, dtheta;
            iss >> dx >> dy >> dtheta;
            chassis.odomUpdate({dx, dy, -dtheta});
        }
        if (code == "get") {
            for (int i = 0; i < 500; i++) {
                std::cout << "particle "<< i << " " << chassis.pf->particles[i].location.x() << " " << chassis.pf->particles[i].location.y() << " " << chassis.pf->particles[i].weight << std::endl;
            }
            std::cout.flush();
        }
        if (code == "left") {
            float dist; iss >> dist;
            chassis.pf->sensors.at(0)->update(dist, 95.0);
        }
        if (code == "front") {
            float dist; iss >> dist;
            chassis.pf->sensors.at(1)->update(dist, 95.0);
        }
        if (code == "right") {
            float dist; iss >> dist;
            chassis.pf->sensors.at(2)->update(dist, 95.0);
        }
        if (code == "pose") {
            lemlib::Pose pose = chassis.getPose(true);
            std::cout << "pose " << pose.x << " " << pose.y << " " << pose.theta << std::endl;
            std::cout.flush();
        }
    }

    return 0;
}
