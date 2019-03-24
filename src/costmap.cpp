#include <ros/console.h>

#include "astar_ackermann_planner/costmap.h"
#include "astar_ackermann_planner/utils.h"

namespace astar_ackermann_planner {

    EmptyCostmap::EmptyCostmap(unsigned int sizeX, unsigned int sizeY, double resolution)
            : sizeX(sizeX), sizeY(sizeY), resolution(resolution) {}

    unsigned int EmptyCostmap::getCost(unsigned int x, unsigned int y) const {
        return 0;
    }

    double EmptyCostmap::getSizeInMetersX() const {
        return sizeX * resolution;
    }

    double EmptyCostmap::getSizeInMetersY() const {
        return sizeY * resolution;
    }

    unsigned int EmptyCostmap::getSizeInCellsX() const {
        return sizeX;
    }

    unsigned int EmptyCostmap::getSizeInCellsY() const {
        return sizeY;
    }

    double EmptyCostmap::getResolution() const {
        return resolution;
    }

    bool EmptyCostmap::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
        if (wx < 0 || wy < 0) {
            return false;
        }
        uint x = uint(wx / resolution);
        uint y = uint(wy / resolution);
        if (x >= getSizeInCellsX() || y >= getSizeInCellsY()) {
            return false;
        }
        mx = x;
        my = y;
        return true;
    }

    EmptyCostmap::~EmptyCostmap() = default;

    CostmapAdapter::CostmapAdapter(costmap_2d::Costmap2D *costmap): costmap_(costmap) {}

    unsigned int CostmapAdapter::getCost(unsigned int x, unsigned int y) const {
        return costmap_->getCost(x, y);
    }

    double CostmapAdapter::getSizeInMetersX() const {
        return costmap_->getSizeInMetersX();
    }
    double CostmapAdapter::getSizeInMetersY() const {
        return costmap_->getSizeInMetersY();
    }

    uint CostmapAdapter::getSizeInCellsX() const {
        return costmap_->getSizeInCellsX();
    }

    uint CostmapAdapter::getSizeInCellsY() const {
        return costmap_->getSizeInCellsY();
    }

    double CostmapAdapter::getResolution() const {
        return costmap_->getResolution();
    }

    bool CostmapAdapter::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const {
        return costmap_->worldToMap(wx, wy, mx, my);
    }

    CostmapAdapter::~CostmapAdapter() = default;

    PositionCostmap::PositionCostmap(
        uint cells_size_x, uint cells_size_y, uint cells_size_th,
        double res_x, double res_y,
        double origin_x, double origin_y, double origin_th,
        double default_value)
        : cells_size_x_(cells_size_x), cells_size_y_(cells_size_y), cells_size_th_(cells_size_th),
          res_x_(res_x), res_y_(res_y), res_th_(2.0 * M_PI / cells_size_th_),
          origin_x_(origin_x), origin_y_(origin_y), origin_th_(origin_th) {
        uint costmap_size = cells_size_x_ * cells_size_y_ * cells_size_th_;
        costmap_ = std::vector<double>(costmap_size, default_value);
      }

    bool PositionCostmap::worldToMap(double wx, double wy, double wth, uint &mx, uint &my, uint &mth) const {
        if (wx < origin_x_ || wy < origin_y_) {
            return false;
        }
        mx = uint((wx - origin_x_) / res_x_);
        my = uint((wy - origin_y_) / res_y_);
        mth = uint(normalize_angle(wth - origin_th_) / res_th_);
        if (mx >= cells_size_x_ || my >= cells_size_y_) {
            return false;
        }
        return true;
    }

    double PositionCostmap::getCost(uint mx, uint my, uint mth) const {
        return costmap_[mx * cells_size_y_ * cells_size_th_ + my * cells_size_th_ + mth];
    }

    void PositionCostmap::setCost(uint mx, uint my, uint mth, double cost) {
        costmap_[mx * cells_size_y_ * cells_size_th_ + my * cells_size_th_ + mth] = cost;
    }

    void PositionCostmap::setOrigin(double wx, double wy, double wth) {
        origin_x_ = wx;
        origin_y_ = wy;
        origin_th_ = wth;
    }

    double PositionCostmap::getOriginX() const {
        return origin_x_;
    }

    double PositionCostmap::getOriginY() const {
        return origin_y_;
    }

    double PositionCostmap::getOriginTh() const {
        return origin_th_;
    }

    uint PositionCostmap::getSizeInCellsX() const {
        return cells_size_x_;
    }

    uint PositionCostmap::getSizeInCellsY() const {
        return cells_size_y_;
    }

    uint PositionCostmap::getSizeInCellsTh() const {
        return cells_size_th_;
    }

    double PositionCostmap::getSizeInMetersX() const {
        return cells_size_x_ * res_x_;
    }

    double PositionCostmap::getSizeInMetersY() const {
        return cells_size_y_ * res_y_;
    }

    double PositionCostmap::getSizeInRadiansTh() const {
        return cells_size_th_ * res_th_;
    }

    PositionCostmap::~PositionCostmap() = default;
}
