//
// Created by alex on 16.02.19.
//

#ifndef ASTAR_ACKERMANN_PLANNER_COSTMAP_H
#define ASTAR_ACKERMANN_PLANNER_COSTMAP_H

#include <costmap_2d/costmap_2d.h>

namespace astar_ackermann_planner {

    /**
     * Associates costs with the points in the world.
     * Largely corresponds to costmap_2d/Costmap2D of the ROS navigation stack.
     */
    class Costmap {
    public:
        /**
         * @brief  Get the cost of a cell in the costmap
         * @param mx The x coordinate of the cell
         * @param my The y coordinate of the cell
         * @return The cost of the cell
         */
        virtual unsigned getCost(unsigned int x, unsigned int y) const = 0;

        virtual double getSizeInMetersX() const = 0;

        virtual double getSizeInMetersY() const = 0;

        virtual unsigned int getSizeInCellsX() const = 0;

        virtual unsigned int getSizeInCellsY() const = 0;

        /**
         * @brief  Get resolution of the costmap, in meters per cell.
         */
        virtual double getResolution() const = 0;

        /**
          * @brief  Convert from world coordinates to map coordinates
          * @param  wx The x world coordinate
          * @param  wy The y world coordinate
          * @param  mx Will be set to the associated map x coordinate
          * @param  my Will be set to the associated map y coordinate
          * @return True if the conversion was successful (legal bounds) false otherwise
          */
        virtual bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const = 0;

        virtual ~Costmap() = default;
    };

    /**
     * A stub costmap used for testing purposes.
     */
    class EmptyCostmap : public Costmap {
    private:
        unsigned int sizeX;
        unsigned int sizeY;
        double resolution;
    public:
        EmptyCostmap(unsigned int sizeX, unsigned int sizeY, double resolution);

        unsigned int getCost(unsigned int x, unsigned int y) const override;

        double getSizeInMetersX() const override;

        double getSizeInMetersY() const override;

        unsigned int getSizeInCellsX() const override;

        unsigned int getSizeInCellsY() const override;

        double getResolution() const override;

        bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const override;

        ~EmptyCostmap() override;
    };

    /**
     * An adapter that fullfils the Costmap interfaces and delegates to a costmap_2d/Costmap2D impolemenation.
     */
    class CostmapAdapter : public Costmap {
    private:
        costmap_2d::Costmap2D *costmap_;
    public:
        CostmapAdapter(costmap_2d::Costmap2D *costmap);

        unsigned int getCost(unsigned int x, unsigned int y) const override;

        double getSizeInMetersX() const override;

        double getSizeInMetersY() const override;

        unsigned int getSizeInCellsX() const override;

        unsigned int getSizeInCellsY() const override;

        double getResolution() const override;

        bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const override;

        ~CostmapAdapter() override;
    };

    /**
     * Associates cost with a position (x, y, th). Is the same as a 3D costmap,
     * but takes into account modular arithmetic for the angles.
     */

    class PositionCostmap {
    private:
        const uint cells_size_x_, cells_size_y_, cells_size_th_;
        double origin_x_, origin_y_, origin_th_;
        const double res_x_, res_y_, res_th_;
        std::vector<double> costmap_;
    public:
        /**
         * @brief  Constructor for a costmap.
         * Size of the map in meters can be specified,
         * however size of the angle dimension is always 2*PI
         * @param  cells_size_x The x size of the map in cells
         * @param  cells_size_y The y size of the map in cells
         * @param  cells_size_th Angle discretization level
         * @param  res_x The resolution of the map in meters/cell
         * @param  res_y The resolution of the map in meters/cell
         * @param  origin_x The x origin of the map
         * @param  origin_y The y origin of the map
         * @param  origin_th The orientation
         * @param  default_value Default Value
         */
        PositionCostmap(
            uint cells_size_x, uint cells_size_y, uint cell_size_th,
            double res_x, double res_y,
            double origin_x, double origin_y, double origin_th,
            double default_value = 0);

        /**
         * Get cost associated with the given cell.
         * The cell should be specified in the map coordinates.
         */
        double getCost(uint mx, uint my, uint mth) const;

        /**
         * Set the cost associated with the given cell.
         * The cell should be specified in the map coordinates.
         */
        void setCost(uint mx, uint my, uint mth, double cost);

        /**
         * Convert world coornidates (in meters) to map coordinates (cell indices)
         * @return True if world coordinates are within the boundaries of the map.
         */
        bool worldToMap(double wx, double wy, double wth, uint &mx, uint &my, uint &mth) const;

        void setOrigin(double origin_x, double origin_y, double origin_th);

        double getOriginX() const;

        double getOriginY() const;

        double getOriginTh() const;

        uint getSizeInCellsX() const;

        uint getSizeInCellsY() const;

        uint getSizeInCellsTh() const;

        double getSizeInMetersX() const;

        double getSizeInMetersY() const;

        double getSizeInRadiansTh() const;

        ~PositionCostmap();
    };
}
#endif //ASTAR_ACKERMANN_PLANNER_COSTMAP_H
