#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "astar_planner/utils.h"

namespace astar_planner {

    Pose::Pose(): Pose(0.0, 0.0, 0.0) {}

    Pose::Pose(double x, double y, double th) : x(x), y(y), th(th) {}

    Pose::Pose(const geometry_msgs::PoseStamped &pose) {
        tf2::Quaternion quat;
        tf2::fromMsg(pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        x = pose.pose.position.x;
        y = pose.pose.position.y;
        th = yaw;
    }

    geometry_msgs::PoseStamped Pose::toPoseStamped() {
        tf2::Quaternion quat;
        quat.setRPY(0, 0, th);
        auto pose = geometry_msgs::PoseStamped();
        pose.pose.orientation = tf2::toMsg(quat);
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        return pose;
    }

    bool Pose::operator==(const astar_planner::Pose &other) const {
        return x == other.x && y == other.y && th == other.th;

    }

    std::ostream& operator<<(std::ostream &out, const Pose &pos) {
        out << std::setprecision(3) << "Pos(" << pos.x << ", " << pos.y << ", " << pos.th / M_PI << "pi)";
    }

    bool PoseWithDist::operator==(const PoseWithDist &other) const {
        return std::equal_to<Pose>()(pose, other.pose);
    }

    bool PoseWithDist::operator<(const PoseWithDist &other) const {
        return dist < other.dist;
}

    double euclid_dist(const Pose &pose1, const Pose &pose2) {
        return sqrt(pow(pose1.x - pose2.x, 2)+ pow(pose1.y - pose2.y, 2));
    }

    /**
     * Normalize angle to [0, 2pi]
     */
    double normalize_angle(double angle) {
        return angle >= 0 ? fmod(angle, 2*M_PI) : 2 * M_PI - fmod(fabs(angle), 2*M_PI);
    }
}
