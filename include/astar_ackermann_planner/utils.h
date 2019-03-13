//
// Created by alex on 15.02.19.
//

#ifndef ASTAR_ACKERMANN_PLANNER_UTILS_H
#define ASTAR_ACKERMANN_PLANNER_UTILS_H

#include <geometry_msgs/PoseStamped.h>
#include <ostream>

namespace astar_ackermann_planner {

    /**
     * Container for storing pose of a 2D object in space.
     */
    struct Pose {
        double x;
        double y;
        double th;

        Pose();

        Pose(double x, double y, double th);

        Pose(const geometry_msgs::PoseStamped &pose);

        geometry_msgs::PoseStamped toPoseStamped();

        bool operator==(const Pose &other) const;

        friend std::ostream& operator<<(std::ostream &out, const Pose &pos);

    };

    /**
     * Container for storing object position and associated cost.
     */
    struct PoseWithDist {
        double dist;
        Pose pose;

        PoseWithDist(double dist, const Pose &pose) : dist(dist), pose(pose) {}

        bool operator<(const PoseWithDist &other) const;

        bool operator==(const PoseWithDist &other) const;
    };

    /**
     * Conainer for storing Costmap coordinates.
     */
    struct Cell {
        uint x;
        uint y;
        uint th;

        Cell(): Cell(0, 0, 0) {}

        Cell(uint x, uint y, uint th) : x(x), y(y), th(th) {}

        bool operator==(const Cell &p) const {
            return x == p.x && y == p.y && th == p.th;
        }
    };

    /**
     * Compute Euclidean distance between two positions.
     */
    double euclid_dist(const Pose &pose1, const Pose &pose2);

    double normalize_angle(double angle);
}

namespace std {

    template<>
    struct hash<astar_ackermann_planner::Cell> {
        std::size_t operator()(const astar_ackermann_planner::Cell &p) const {
            return (p.x << 26) + (p.y << 13) + p.th;
        }
    };

    template<>
    struct hash<astar_ackermann_planner::Pose> {
        std::size_t operator()(const astar_ackermann_planner::Pose &pose) const {
            return (size_t(pose.x) << 40) +
                   (size_t(pose.y) << 20) + size_t(pose.th);
        }
    };

}



#endif //ASTAR_ACKERMANN_PLANNER_UTILS_H
