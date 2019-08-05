//
//  LinearClustering.hpp
//  Lisco
//
//  Created by Hannah Ataei on 2017-06-09.
//  Copyright © 2017 Hannah Ataei. All rights reserved.
//

#ifndef LinearClustering_hpp
#define LinearClustering_hpp

#include "LidarSpec.hpp"
#include <chrono>
#include <unordered_map>

namespace lisco {
    struct alignas(64) Point{
        double dist, theta, alpha;
        double x,y,z;
        int key;
        Point *parent_pointer;
        
        Point(): dist(0.0), theta(0.0), alpha(0.0), parent_pointer(NULL) {}
    };
    
    struct node{
        Point *array[16];
        node *next;
        int lastIndex;
        int numOfClusterPoints;
        
        node(): lastIndex(0), next(0){}
    };
    
    
    class LinearClustering{
    private:
        int min_cluster_size_;
        double max_cluster_within_distance_;
        double max_cluster_within_distance_pow_;
        int hashSize;
        
    public:
        std::unordered_map<int, node*> hashList;
        LidarClass lidar;
        Point **points;
        
        LinearClustering():
        hashSize(14),
        min_cluster_size_ (1),
        max_cluster_within_distance_ (0.0),
        max_cluster_within_distance_pow_ (0.0)
        {
            //2350 maximum number of steps in one rotation, 63 number of lasers using velodyne64
            points = new Point*[2350];
            for(int i = 0; i <2350; i++)
                points[i] = new Point[64];
        }
        
        ~LinearClustering(){
            for (int i = 2349; i >= 0; i--)
                delete[] points[i];
            delete[] points;
        }
        
        inline void setMinClusterSize (int min_cluster_size)
        {
            min_cluster_size_ = min_cluster_size;
        }
        inline int getMinClusterSize ()
        {
            return (min_cluster_size_);
        }
        
        inline void setMaxClusterWithinDistance (double max_cluster_within_distance)
        {
            max_cluster_within_distance_ = max_cluster_within_distance;
        }
        inline double getMaxClusterWithinDistance ()
        {
            return (max_cluster_within_distance_);
        }
        
        inline void setMaxClusterWithinDistancePow (double max_cluster_within_distance_pow)
        {
            max_cluster_within_distance_pow_ = max_cluster_within_distance_pow;
        }
        inline double getMaxClusterWithinDistancePow ()
        {
            return (max_cluster_within_distance_pow_);
        }
        
        bool SphericalCoordinatesEuclideanDistance(Point &x, Point &y);
        void linearStreamClustering(int step);
        void searchInNeighbors(Point &p, int laserNum, int stepNum);
        void expandThis(Point *p, Point* neighbor);
        void unionTwoClusters(node *p, Point *x);
    };
}

#endif /* LinearClustering_hpp */
