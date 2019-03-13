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
}
#endif //ASTAR_ACKERMANN_PLANNER_COSTMAP_H
