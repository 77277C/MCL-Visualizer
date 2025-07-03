#define _USE_MATH_DEFINES
#include <iostream>
#include "localization/lemlib_integration.hpp"


#define N 500


int main() {
    Distance left(Eigen::Vector3f(0, 0, M_PI_2));
    Distance front(Eigen::Vector3f(0, 0, 0));
    Distance right(Eigen::Vector3f(0, 0, -M_PI_2));

    std::vector<Distance*> distances = {&left, &front, &right};

    ParticleFilterChassis<N> chassis(distances);
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
            for (int i = 0; i < N; i++) {
                std::cout << "particle "<< i << " " << chassis.pf.particles[i].location.x() << " " << chassis.pf.particles[i].location.y() << " " << chassis.pf.particles[i].weight << std::endl;
            }
            std::cout.flush();
        }
        if (code == "left") {
            float dist; iss >> dist;
            left.update(dist, 63.0);
        }
        if (code == "front") {
            float dist; iss >> dist;
            front.update(dist, 63.0);
        }
        if (code == "right") {
            float dist; iss >> dist;
            right.update(dist, 63.0);
        }
        if (code == "pose") {
            lemlib::Pose pose = chassis.getPose(true);
            std::cout << "pose " << pose.x << " " << pose.y << " " << pose.theta << std::endl;
            std::cout.flush();
        }
    }

    return 0;
}
