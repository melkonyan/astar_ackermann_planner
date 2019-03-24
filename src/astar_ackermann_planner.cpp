//
// Created by alex on 13.02.19.
//

#include <pluginlib/class_list_macros.h>

#include <chrono>
#include <ctime>
#include <math.h>
#include <queue>
#include <unordered_map>

#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <astar_ackermann_planner/astar_ackermann_planner.h>
#include <astar_ackermann_planner/utils.h>

PLUGINLIB_EXPORT_CLASS(astar_ackermann_planner::AStarAckermannPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace astar_ackermann_planner {

    AStarAckermannPlanner::AStarAckermannPlanner() :
        name_(""), costmap_(nullptr), step_size_(0.0), turning_radius_(0.0), global_frame_("") {}

    void AStarAckermannPlanner::initialize(std::string name,
                                           costmap_2d::Costmap2DROS *costmap_ros) {
        global_frame_ = costmap_ros->getGlobalFrameID();
        initialize(name, new CostmapAdapter(costmap_ros->getCostmap()));
    };

    void AStarAckermannPlanner::initialize(std::string name, astar_ackermann_planner::Costmap *costmap) {
        name_ = name;
        costmap_ = costmap;
        ros::NodeHandle n;
        plan_publisher_ = n.advertise<nav_msgs::Path>(name + "/global_plan", 1);
        loadParameters();
        angle_discretization_level_ = computeAngleDiscretizationLevel(step_size_);
        ROS_INFO("AStarAckermannPlanner initialized with name '%s' and angle discretization level=%d.",
                 name_.c_str(), angle_discretization_level_);
        initHolonomicDistmap();
        computeHolonomicDistmap();
    }

    void AStarAckermannPlanner::loadParameters() {
        ros::NodeHandle nh("~" + name_);
        nh.param<double>("turning_radius", turning_radius_, 0.0);
        nh.param<double>("step_size", step_size_, 0.0);
        nh.param<int>("max_allowed_time", max_allowed_time_, 20);
        nh.param<double>("goal_tolerance", goal_dist_tolerance_, 0.5);
        nh.param<double>("goal_orientation_tolerance", goal_orientation_tolerance_, M_PI / 30);
        double holonomic_distmap_size_meters;
        nh.param<double>("holonomic_distmap_size", holonomic_distmap_size_meters, 80);
        nh.param<double>("holonomic_distmap_resolution", holonomic_distmap_resolution_, 1.0);
        holonomic_distmap_grid_size_ = uint(holonomic_distmap_size_meters * holonomic_distmap_resolution_);
    }

    bool AStarAckermannPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                         const geometry_msgs::PoseStamped &goal,
                                         std::vector<geometry_msgs::PoseStamped> &plan) {
        std::vector<Pose> positions;
        bool foundPlan;
        try {
            foundPlan = makePlan(Pose(start), Pose(goal), positions);
        } catch (exception &ex) {
            ROS_FATAL("AStarAckermannPlanner exception occured %s", ex.what());
            throw ex;
        }
        if (!foundPlan) {
            return false;
        }
        ros::Time plan_time = ros::Time::now();
        for (auto position : positions) {
            auto pose = position.toPoseStamped();
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame_;
            plan.push_back(pose);
        }
        publishPlan(plan);
        return true;
    }

    void AStarAckermannPlanner::publishPlan(std::vector<geometry_msgs::PoseStamped> &path) {
        nav_msgs::Path gui_path;
        gui_path.header.frame_id = global_frame_;
        gui_path.header.stamp = ros::Time::now();
        gui_path.poses.resize(path.size());
        std::copy(path.begin(), path.end(), gui_path.poses.begin());
        plan_publisher_.publish(gui_path);
    }


    bool AStarAckermannPlanner::makePlan(const Pose &start, const Pose &goal, vector<Pose> &path) {
        if (!validateParameters()) {
            return false;
        }
        holonomic_distmap_->setOrigin(goal.x, goal.y, goal.th);
        auto cell_start = getCell(start);
        auto cell_goal = getCell(goal);
        // Instantiate data structures
        set<PoseWithDist> candidate_poses = {PoseWithDist(0.0, start)};
        unordered_map<Cell, double> pathLength = {{cell_start, 0.0}};
        unordered_map<Pose, Pose> parents;
        Pose reached_pose;
        bool reached_goal = false;

        // Create variables to log performance
        time_t start_time = time(NULL);
        int num_nodes_visited = 0;

        while (!candidate_poses.empty()) {
            if (max_allowed_time_ > 0 && difftime(time(NULL), start_time) > max_allowed_time_) {
                break;
            }
            num_nodes_visited++;
            PoseWithDist cand = *candidate_poses.begin();
            auto cell_cand = getCell(cand.pose);
            if (hasReachedGoal(cand.pose, goal)) {
                reached_pose = cand.pose;
                reached_goal = true;
                break;
            }
            double l_cand = pathLength[cell_cand];

            candidate_poses.erase(cand);
            vector<PoseWithDist> neighbors = getNeighbors(cand.pose, step_size_);
            for (auto &nbr : neighbors) {
                if (!checkBounds(nbr.pose)) {
                    continue;
                }
                auto cell_nbr = getCell(nbr.pose);
                if (cell_cand == cell_nbr) {
                    ROS_WARN("AStarAckermannPlanner: Oops, ended up in the same cell.");
                    continue;
                }
                if (costmap_->getCost(cell_nbr.x, cell_nbr.y) > 0) {
                    continue;
                }
                if (pathLength.find(cell_nbr) == pathLength.end() || l_cand + nbr.dist < pathLength[cell_nbr]) {
                    pathLength[cell_nbr] = l_cand + nbr.dist;
                    parents[nbr.pose] = cand.pose;
                    candidate_poses.insert(
                        PoseWithDist(l_cand + nbr.dist + distToGoalEstimate(nbr.pose, goal), nbr.pose));
                }
            }
        }
        if (reached_goal) {
            getPath(parents, reached_pose, path);
        }

        ROS_INFO("AStarAckermannPlanner finished in %.2fs, generated %d nodes, reached goal: %s",
                 difftime(time(NULL), start_time), num_nodes_visited, reached_goal ? "true" : "false");
        return reached_goal;

    }

    bool AStarAckermannPlanner::validateParameters() const {
        if (turning_radius_ <= 0) {
            ROS_ERROR("AStarAckermannPlanner: turning radius has invalid value=%.2f. Must be greater than zero.",
                      turning_radius_);
            return false;
        }
        if (step_size_ <= 0) {
            ROS_ERROR("AStarAckermannPlanner: step size has invalid value=%.2f. Must be greater than zero.",
                      step_size_);
            return false;
        }
        if (goal_dist_tolerance_ <= 0) {
            ROS_ERROR("AStarAckermannPlanner: goal tolerance has invalid value=%.2f. Must be greater than zero",
                      goal_dist_tolerance_);
            return false;
        }
        if (goal_dist_tolerance_ < step_size_) {
            ROS_WARN("AStarAckermannPlanner: goal tolerance (=%.2f) is smaller than the step size (=%.2f). "
                     "Planner might fail to find a path.", goal_dist_tolerance_, step_size_);
        }
        if (step_size_ < costmap_->getResolution()) {
            ROS_WARN("AStarPlannner: step size (=%.2f) is smaller than costmap resolution (=%.2f). "
                     "Planner might fail to exlore the map properly.", step_size_, costmap_->getResolution());
        }
        return true;
    }

    uint AStarAckermannPlanner::computeAngleDiscretizationLevel(double step_size) const {
        double dth = step_size / turning_radius_;
        return uint(ceil(2 * 2 * M_PI / dth));
    }

    void AStarAckermannPlanner::initHolonomicDistmap() {
        holonomic_step_size_ = holonomic_distmap_resolution_ * 1.2;
        holonomic_distmap_ = unique_ptr<PositionCostmap>(new PositionCostmap(
            holonomic_distmap_grid_size_, holonomic_distmap_grid_size_,
            computeAngleDiscretizationLevel(holonomic_step_size_),
            holonomic_distmap_resolution_, holonomic_distmap_resolution_,
            0, 0, 0, // origin_x, origin_y, origin_th
            2.0 * holonomic_distmap_grid_size_
        ));
        holonomic_distmap_->setOrigin(-holonomic_distmap_->getSizeInMetersX() / 2,
                                      -holonomic_distmap_->getSizeInMetersY() / 2,
                                      0);
        ROS_INFO("Holonomic distmap initialized with world size = (%.2f, %.2f, %.2f), "
                 "grid size = (%d, %d, %d). ",
                 holonomic_distmap_->getSizeInMetersX(),
                 holonomic_distmap_->getSizeInMetersY(),
                 holonomic_distmap_->getSizeInRadiansTh(),
                 holonomic_distmap_->getSizeInCellsX(),
                 holonomic_distmap_->getSizeInCellsY(),
                 holonomic_distmap_->getSizeInCellsTh());
    }

    void AStarAckermannPlanner::computeHolonomicDistmap() {
        double step_size = holonomic_step_size_;
        auto start_pose = Pose(0, 0, 0);
        set<PoseWithDist> candidate_poses = {PoseWithDist(0, start_pose)};
        Cell origin_cell;
        holonomic_distmap_->worldToMap(
            start_pose.x, start_pose.y, start_pose.th,
            origin_cell.x, origin_cell.y, origin_cell.th
        );
        holonomic_distmap_->setCost(origin_cell.x, origin_cell.y, origin_cell.th, 0);
        time_t start_time = time(NULL);
        uint num_visited_cells = 0;
        while (!candidate_poses.empty()) {
            num_visited_cells++;
            auto curr = *candidate_poses.begin();
            Cell curr_cell;
            holonomic_distmap_->worldToMap(
                curr.pose.x, curr.pose.y, curr.pose.th,
                curr_cell.x, curr_cell.y, curr_cell.th
            );
            double curr_cost = holonomic_distmap_->getCost(curr_cell.x, curr_cell.y, curr_cell.th);
            candidate_poses.erase(curr);
            auto neighbors = getNeighbors(curr.pose, step_size);
            for (auto &nbr : neighbors) {
                Cell nbr_cell;
                if (!holonomic_distmap_->worldToMap(nbr.pose.x, nbr.pose.y, nbr.pose.th,
                                                    nbr_cell.x, nbr_cell.y, nbr_cell.th)) {
                    continue;
                }
                if (curr_cell == nbr_cell) {
                    ROS_WARN("AStarAckermannPlanner.computeHolonomicDistmap"
                             ": Oops, ended up in the same cell.");
                    continue;
                }
                double nbr_cost = holonomic_distmap_->getCost(nbr_cell.x, nbr_cell.y, nbr_cell.th);
                double new_cost = curr_cost + step_size;
                if (new_cost < nbr_cost) {
                    holonomic_distmap_->setCost(nbr_cell.x, nbr_cell.y, nbr_cell.th, new_cost);
                    candidate_poses.insert(PoseWithDist(new_cost, nbr.pose));
                }
            }
        }
        uint num_total_cells = holonomic_distmap_->getSizeInCellsX()
                               * holonomic_distmap_->getSizeInCellsY()
                               * holonomic_distmap_->getSizeInCellsTh();
        ROS_INFO("Holonomic dist map created in %.2fs, visited %d out of %d cells",
                 difftime(time(NULL), start_time), num_visited_cells, num_total_cells);
    }

    void AStarAckermannPlanner::getPath(const unordered_map<Pose, Pose> &parents,
                                        const Pose &goal_pose,
                                        vector<Pose> &path) const {
        auto curr_pose = goal_pose;
        while (true) {
            path.push_back(curr_pose);
            auto search = parents.find(curr_pose);
            if (search == parents.end()) {
                break;
            }
            curr_pose = search->second;
        }
        reverse(path.begin(), path.end());
    }

    bool AStarAckermannPlanner::hasReachedGoal(const Pose &pos, const Pose &goal) {
        return euclid_dist(pos, goal) <= goal_dist_tolerance_
               && angle_diff(pos.th, goal.th) < goal_orientation_tolerance_;
    }

    Pose AStarAckermannPlanner::turnLeft(const Pose &pos, double dth) const {
        double pos_dx = -sin(pos.th) * turning_radius_;
        double pos_dy = cos(pos.th) * turning_radius_;
        double c_x = pos.x - pos_dx;
        double c_y = pos.y - pos_dy;
        double new_dx = -sin(pos.th - dth) * turning_radius_;
        double new_dy = cos(pos.th - dth) * turning_radius_;
        return Pose(c_x + new_dx, c_y + new_dy, pos.th - dth);
    }

    Pose AStarAckermannPlanner::goStraight(const Pose &pos, double dist) const {
        return Pose(
            pos.x + dist * cos(pos.th),
            pos.y + dist * sin(pos.th),
            pos.th
        );
    }

    Pose AStarAckermannPlanner::turnRight(const Pose &pos, double dth) const {
        double pos_dx = sin(pos.th) * turning_radius_;
        double pos_dy = -cos(pos.th) * turning_radius_;
        double c_x = pos.x - pos_dx;
        double c_y = pos.y - pos_dy;
        double new_dx = sin(pos.th + dth) * turning_radius_;
        double new_dy = -cos(pos.th + dth) * turning_radius_;
        return Pose(c_x + new_dx, c_y + new_dy, pos.th + dth); // there's a bug somewhere.
    }

    bool AStarAckermannPlanner::checkBounds(const Pose &pos) const {
        uint x, y;
        return costmap_->worldToMap(pos.x, pos.y, x, y);
    }

    vector<PoseWithDist> AStarAckermannPlanner::getNeighbors(const Pose &pos, double step_size) const {
        double dth = step_size / turning_radius_;
        return {
            PoseWithDist(step_size, turnLeft(pos, dth)),
            PoseWithDist(step_size, goStraight(pos, step_size)),
            PoseWithDist(step_size, turnRight(pos, dth))
        };
    }

    Cell AStarAckermannPlanner::getCell(const Pose &pos) const {
        uint c_x, c_y;
        costmap_->worldToMap(pos.x, pos.y, c_x, c_y);
        // 0 <= cell.th <= angle_discretization_level
        uint c_th = uint(normalize_angle(pos.th) * angle_discretization_level_ / 2 / M_PI);
        return Cell(c_x, c_y, c_th);
    }

    double AStarAckermannPlanner::distToGoalEstimate(const Pose &pose1, const Pose &goal) const {
        double holonomic_cost = 0;
        uint c_x, c_y, c_th;
        if (holonomic_distmap_->worldToMap(pose1.x, pose1.y, pose1.th, c_x, c_y, c_th)) {
            holonomic_cost = holonomic_distmap_->getCost(c_x, c_y, c_th);
        }
        return max(holonomic_cost, euclid_dist(pose1, goal));
    }

    AStarAckermannPlanner::~AStarAckermannPlanner() {
        delete costmap_;
    }
}
