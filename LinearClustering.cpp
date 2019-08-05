//
//  LinearClustering.cpp
//  Lisco
//
//  Created by Hannah Ataei on 2017-06-09.
//  Copyright © 2017 Hannah Ataei. All rights reserved.
//

#include "LinearClustering.hpp"


bool lisco::LinearClustering::SphericalCoordinatesEuclideanDistance(Point &x, Point &y)
{
    if (pow((x.x-y.x),2)+pow((x.y-y.y),2)+pow((x.z-y.z),2) <= max_cluster_within_distance_pow_)
        return true;
    return false;
}

void lisco::LinearClustering::linearStreamClustering(int step){
    for(int i=0; i<lidar.getNumberOfLasers(); i++){
        if(points[step][i].dist>0){
            searchInNeighbors(points[step][i], i, step);
        }
    }
}

void lisco::LinearClustering::searchInNeighbors(Point &p, int laserNum, int stepNum){
    int numOfLayersForProcessing = lidar.processLayers[floor(p.dist)];
    int numOfStepsForProcessing = lidar.processSteps[floor(p.dist)];

    
    double startAngle = points[std::max(0,stepNum-numOfStepsForProcessing)][laserNum].alpha;
    for(int i=std::max(laserNum-numOfLayersForProcessing,0); i<std::min(lidar.getNumberOfLasers(),laserNum+numOfLayersForProcessing); i++){
        for(int j=std::max(0,stepNum-numOfStepsForProcessing-lidar.alpha_matrix[i][laserNum]); j<=stepNum; j++){
            if(j==stepNum && i>=laserNum){
                break;
            }
            if(pow((points[j][i].dist-p.dist), 2)>max_cluster_within_distance_pow_){
                continue;
            }
            if(points[j][i].dist!=0 && points[j][i].alpha>=startAngle && (p.parent_pointer==NULL || points[j][i].parent_pointer==NULL || p.parent_pointer!=points[j][i].parent_pointer)){
                if(SphericalCoordinatesEuclideanDistance(p, points[j][i])){
                    expandThis(&p, &points[j][i]);
                }
            }
        }
        
        if(stepNum+numOfStepsForProcessing+lidar.alpha_matrix[i][laserNum] >= lidar.getNumberOfSteps()){
            int columnNum = lidar.getNumberOfSteps() - (stepNum+numOfStepsForProcessing+lidar.alpha_matrix[i][laserNum]);
            for(int j=0; j<=columnNum; j++){
                if(pow((points[j][i].dist-p.dist), 2)>max_cluster_within_distance_pow_){
                    continue;
                }
                if(points[j][i].dist!=0 && (p.parent_pointer==NULL || points[j][i].parent_pointer==NULL || p.parent_pointer!=points[j][i].parent_pointer)){
                    if(SphericalCoordinatesEuclideanDistance(p, points[j][i])){
                        expandThis(&p, &points[j][i]);
                    }
                }
            }
        }
    }
}

void lisco::LinearClustering::expandThis(Point *p, Point* neighbor){
    if(p->parent_pointer == NULL){
        if(neighbor->parent_pointer==NULL){ //create new cluster
            neighbor->parent_pointer = neighbor;
            p->parent_pointer = neighbor->parent_pointer;
            
            lisco::node *cluster = new lisco::node();
            cluster->array[cluster->lastIndex++] = neighbor;
            cluster->array[cluster->lastIndex++] = p;
            cluster->numOfClusterPoints = 2;
            hashList[neighbor->key] = cluster;
        }
        else{ //add p to neighbour's cluster
            p->parent_pointer = neighbor->parent_pointer;
            
            node *temp = hashList[neighbor->parent_pointer->key];
            temp->numOfClusterPoints++;
            while(temp->next!=0)
                temp = temp->next;
            if(temp->lastIndex >= hashSize){
                node *cluster = new node();
                cluster->array[cluster->lastIndex++] = p;
                temp->next = cluster;
            }
            else{
                temp->array[temp->lastIndex++] = p;
            }
        }
    }
    else if(neighbor->parent_pointer==NULL){ //add neighbour to p's cluster
        neighbor->parent_pointer = p->parent_pointer;
        
        node *temp = hashList[p->parent_pointer->key];
        temp->numOfClusterPoints++;
        while(temp->next!=0)
            temp = temp->next;
        if(temp->lastIndex >= hashSize){
            node *cluster = new node();
            cluster->array[cluster->lastIndex++] = neighbor;
            temp->next = cluster;
        }
        else{
            temp->array[temp->lastIndex++] = neighbor;
        }
    }
    else if(p->parent_pointer != neighbor->parent_pointer){ //merge two clusters
        node *Ptemp = hashList[p->parent_pointer->key];
        node *Ntemp = hashList[neighbor->parent_pointer->key];
        
        if(Ptemp->numOfClusterPoints < Ntemp->numOfClusterPoints){
            //add p's cluster to neighbour's
            int keyForDelete = p->parent_pointer->key;
            unionTwoClusters(Ptemp, neighbor->parent_pointer);
            
            Ntemp->numOfClusterPoints+=Ptemp->numOfClusterPoints;
            while(Ntemp->next!=0)
                Ntemp = Ntemp->next;
            Ntemp->next = hashList[keyForDelete];
            hashList.erase(keyForDelete);
        }
        else{
            
            //add neighbour's cluster to p's
            int keyForDelete = neighbor->parent_pointer->key;
            unionTwoClusters(Ntemp, p->parent_pointer);
            
            Ptemp->numOfClusterPoints+=Ntemp->numOfClusterPoints;
            while(Ptemp->next!=0)
                Ptemp = Ptemp->next;
            Ptemp->next = hashList[keyForDelete];
            hashList.erase(keyForDelete);
        }
    }
}

void lisco::LinearClustering::unionTwoClusters(node *p, Point *header){
    if(p->next != 0){
        unionTwoClusters(p->next, header);
    }
    for(int i=0; i<p->lastIndex; i++){
        p->array[i]->parent_pointer = header;
    }
}
