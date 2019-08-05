//
//  LidarSpec.hpp
//  Lisco
//
//  Created by Hannah Ataei on 2017-05-22.
//  Copyright © 2017 Hannah Ataei. All rights reserved.
//

#ifndef LidarSpec_hpp
#define LidarSpec_hpp

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <tgmath.h>
#include <vector>

class LidarClass{
private:
    double min_yaw_difference_;         //minimum yaw angular difference between two steps
    double min_elevation_difference_;   // minimum elevation angular difference between two lasers
    int number_of_lasers_;
    int number_of_steps_;
    double lidar_vertical_offset_;
    std::vector<double> initial_yaw_offset_;
    
public:
    LidarClass():
    min_yaw_difference_ (0.0015),       //settings regard to the velodyne64
    min_elevation_difference_ (0.0059), //settings regard to the velodyne64
    number_of_lasers_ (0),
    number_of_steps_ (0),
    lidar_vertical_offset_(2.3)         //ford campus setting, the position of lidar on the car regard to the ground
    {}
    
    inline void setMinYawDifference (double min_yaw_difference)
    {
        min_yaw_difference_ = min_yaw_difference;
    }
    inline double getMinYawDifference ()
    {
        return (min_yaw_difference_);
    }
    inline void setMinElevationDifference (double min_elevation_difference)
    {
        min_elevation_difference_ = min_elevation_difference;
    }
    inline double getMinElevationDifference ()
    {
        return (min_elevation_difference_);
    }
    inline void setNumberOfLasers (int number_of_lasers)
    {
        number_of_lasers_ = number_of_lasers;
    }
    inline int getNumberOfLasers ()
    {
        return (number_of_lasers_);
    }
    
    inline void setNumberOfSteps (int number_of_steps)
    {
        number_of_steps_ = number_of_steps;
    }
    inline int getNumberOfSteps ()
    {
        return (number_of_steps_);
    }
    
    inline void setLidarVerticalOffset (double lidar_vertical_offset)
    {
        lidar_vertical_offset_ = lidar_vertical_offset;
    }
    inline double getLidarVerticalOffset ()
    {
        return (lidar_vertical_offset_);
    }
    
    void initialize(std::string fileName);
    std::vector< std::vector<int> > alpha_matrix;
    std::vector<int> processLayers;
    std::vector<int> processSteps;
    void setAlphaMatrix();
    
};


#endif /* LidarSpec_hpp */
