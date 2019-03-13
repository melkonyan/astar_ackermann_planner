//
// Created by alex on 13.02.19.
//

#ifndef ASTAR_PLANNER_ASTAR_H
#define ASTAR_PLANNER_ASTAR_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>
// Abstract global planner from move_base
#include <nav_core/base_global_planner.h>
#include <vector>
#include <unordered_map>

#include "astar_planner/costmap.h"
#include "astar_planner/utils.h"

namespace astar_planner {

    class AStarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        AStarPlanner();

        ~AStarPlanner();

        /**
          * Initialization function for the AStarPlanner object
          * @param  name The name of this planner
          * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
          */
        void initialize(std::string name,
                        costmap_2d::Costmap2DROS *costmap_ros) override;

        /**
         * Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
        */
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan) override;

    public:
        // Methods that are left public for unit tests

        /**
         * Initialization function for the AStarPlanner object
         */
        void initialize(std::string name, Costmap *costmap);

        bool makePlan(const Pose &start, const Pose &goal, std::vector<Pose> &path);

        /**
         * Get poses reachable from the given pose by integrating a discrete set of controls
         * over a short period of time.
         * @param pos Pose from which to start.
         * @return a vector of reachable poses.
         */
        std::vector<PoseWithDist> getNeighbors(const Pose &pos) const;

    private:
        /**
         * Reconstruct the path to the goal position given a set of child/parent pairs.
         * A child position was visited from the parent position when running the planning algorithm.
         * @param parents a map of child/parent relationships. Keys are children, values are parents.
         * @param goal_pos goal position, is assumed to have no children.
         * @param path conainer where the path is be stored.
         *              The first element is the start position, the last element - goal position.
         */
        void getPath(const std::unordered_map<Pose, Pose> &parents,
                     const Pose &goal_pos,
                     std::vector<Pose> &path) const;

        void publishPlan(std::vector<geometry_msgs::PoseStamped> &path);

        /**
         * Estimated distance between two poses.
         */
        double distEstimate(const Pose &pose1, const Pose &pose2) const;

        /**
         * Snap pose to a costmap cell.
         */
        Cell getCell(const Pose &pos) const;

        void loadParameters();

        bool validateParameters() const;

        bool hasReachedGoal(const Pose &pos, const Pose &goal);

        bool checkBounds(const Pose &pos) const;

        PoseWithDist goStraight(const Pose &pos) const;

        PoseWithDist turnLeft(const Pose &pos, double angle) const;

        PoseWithDist turnRight(const Pose &pos, double angle) const;

    private:
        std::string name_;
        Costmap *costmap_;
        std::string global_frame_;
        ros::Publisher plan_publisher_;
        int max_allowed_time_;

        double step_size_;
        double turning_radius_;
        double goal_tolerance_;
        uint angle_discretization_level_;
    };


}

#endif //ASTAR_PLANNER_ASTAR_H
