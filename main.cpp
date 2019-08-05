//
//  LinearClustering.cpp
//  Lisco
//
//  Created by Hannah Ataei on 2017-06-09.
//  Copyright © 2017 Hannah Ataei. All rights reserved.
//
// The main sequential one thread version. All the result for bigdata2017 were extracted from this version.
// the thread traverse all the points one by one and in order, process them and modifies the header links

#include "LinearClustering.hpp"
#include <unistd.h>         //getopt
#include <stdlib.h>         // exit, EXIT_FAILURE

#ifdef __linux__    // Linux only
#include <sched.h>  // sched_setaffinity
#endif

static const char *optString = "t:f:r:e:h";

//filename shows the path to read the data
//groundThreshold is used to identify the ground points
//the point cloud will be returned to the main function in the 2d vecotor points
void readData(std::string filename, double groundThreshold, lisco::Point **points){
    std::ifstream infile(filename);
    int numOfLasers = 0, numOfSteps = 0;
    if (infile.is_open())
    {
        std::string l,s,useless;
        if(std::getline(infile, l, ',') &&
           std::getline(infile, s, ',') &&
           std::getline(infile, useless)){
            numOfLasers = std::stoi(l);
            numOfSteps = std::stoi(s);
        }
        
        for(int j=0; j<numOfSteps; j++)
        {
            for(int i=0; i<numOfLasers; i++){
                std::string ds,ts,as;
                if(std::getline(infile, ds, ',') &&
                   std::getline(infile, ts, ',') &&
                   std::getline(infile, as)){
                    points[j][i].dist = std::stod(ds);
                    points[j][i].theta = std::stod(ts);
                    points[j][i].alpha = std::stod(as);
                    if(points[j][i].dist>0 && -1*points[j][i].dist*sin(points[j][i].theta)<groundThreshold){
                        points[j][i].x =points[j][i].dist*cos(points[j][i].theta)*sin(points[j][i].alpha);
                        points[j][i].y =points[j][i].dist*cos(points[j][i].theta)*cos(points[j][i].alpha);
                        points[j][i].z =points[j][i].dist*sin(points[j][i].theta);
                    }
                    else{
                        points[j][i].dist = 0;
                    }
                }
                points[j][i].key = j*64+i;
            }
        }
        infile.close();
    }
    else std::cout << "Unable to open file to read\n";
}

//print number of clusters and average execution time in the terminal
void printResults(double results[], int numOfRun, std::unordered_map<int, lisco::node*> &hashList, int minPts){
    double overalTime = 0;
    for(int i = 0; i < numOfRun; ++i)
        overalTime += results[i];
    
    double mean = overalTime/numOfRun;
    double standardDeviation = 0;
    for(int i = 0; i < numOfRun; ++i)
        standardDeviation += pow(results[i] - mean, 2);
    standardDeviation = sqrt(standardDeviation / numOfRun);
    
    //confidence level 99%
    double ci = 2.58*(standardDeviation/sqrt(numOfRun));
    
    std::cout << "average elapsed time over " << numOfRun << " runs: "<<  mean << std::endl;
    std::cout << "Confident Interval: "<< ci << std::endl;
    
    int id = 0;
    
    for(auto iter : hashList)
    {
        int count = 0;
        lisco::node *temp = iter.second;
        count+=temp->lastIndex;
        while(temp->next!=0){
            temp = temp->next;
            count+=temp->lastIndex;
        }
        if(count >= minPts){
            id++;
        }
    }
    std::cout << "Number of clusters (with noise cluster) :" << id << std::endl;
}

//this is only for synthetic data to evaluate the results of Lisco with other approaches
void writeSyntheticResults(int fileNum, double epsilon, int numberOfSteps, int numberOfLasers, std::unordered_map<int, lisco::node*> &hashList, lisco::Point **points, int minPts){
    int dataSize = numberOfSteps*numberOfLasers;
    
    int* labels = new int[dataSize];
    for (int i = 0; i < dataSize; i++){
        labels[i] = -1;
    }
    
    int id = 0;
    for(auto iter : hashList)
    {
        int count = 0;
        lisco::node *tmp = iter.second;
        count+=tmp->lastIndex;
        while(tmp->next!=0){
            tmp = tmp->next;
            count+=tmp->lastIndex;
        }
        
        if(count >= minPts){
            lisco::node *temp = iter.second;
            do{
                for(int i=0; i<temp->lastIndex; i++)
                    labels[ temp->array[i]->key ] = id;
                temp = temp->next;
            }while (temp !=0);
            id++;
        }
    }
    
    std::string filename;
#ifdef __APPLE__
    filename = "/Users/hannah/repositories/synthetic_data/sphResult"+std::to_string(fileNum)+"epsilon"+std::to_string(epsilon)+".txt";
#else
    filename = "../../../synthetic_data/sphResult"+std::to_string(fileNum)+"epsilon"+std::to_string(epsilon)+".txt";
#endif
    std::ofstream outfile(filename);
    
    if (outfile.is_open())
    {
        int ind = 0;
        for(int j=0; j<numberOfSteps; j++){
            for(int i=0; i<numberOfLasers; i++){
                if(points[j][i].dist>0){
                    ind = j*numberOfLasers+i;
                    outfile << labels[ind] << ",";
                }
            }
        }
        outfile.close();
    }
    else std::cout << "Unable to open file\n";
    
}

void display_usage()
{
    std::cout<<"Usage..."<<std::endl;
    std::cout<<
    "\n\t-t dataset type (real or synthetic),"
    "\n\t-f file number (for real datatset, it is the index of starting rotation and for synthetic dataset, it is the number of scenario),"
    "\n\t-r #rotations to run,"
    "\n\t-e epsilon (meter),"
    "\n\t-h the help message"
    <<std::endl;
    exit(EXIT_FAILURE);
}

struct input_args {
    const int type;
    const int fileNumber;
    const int rotation;
    const double epsilon;
    
    input_args(int t, int f, int r, double e): type(t), fileNumber(f), rotation(r), epsilon(e){}
};

const input_args parse_arguments(int argc, char** argv)
{
    
    int opt = 0;
    int type=0 ,fileNumber=0, rotation=0;
    double epsilon=0.0;
    
    while ((opt = getopt(argc, argv, optString)) != -1) {
        switch (opt) {
            case 't':
                type = atoi(optarg);
                break;
                
            case 'f':
                fileNumber = atoi(optarg);
                break;
                
            case 'r':
                rotation = atoi(optarg);
                break;
                
            case 'e':
                epsilon = atof(optarg);
                break;
                
            case 'h':
                
            default:
                display_usage();
                break;
        }
    }
    
    if(type==0||fileNumber==0||rotation==0||epsilon==0.0){
        display_usage();
    }
    
    const input_args globalArgs(type ,fileNumber, rotation, epsilon);
    return globalArgs;
}

//inputs: if argv[1]==1 read sythetic files, so argv[2] is filenum and argv[3] is number of experiments, , argv[4] is epsilon
//but if argv[1]!=1, read real dataset, so argv[2] is the start rotation and argv[3] is the number of rotations to process, argv[4] is epsilon
int main (int argc, char *argv[]) {
    
#ifdef __linux__
    cpu_set_t mask;
    int status;
    
    CPU_ZERO(&mask);
    CPU_SET(4, &mask); //run on CPU 4
    status = sched_setaffinity(0, sizeof(mask), &mask);
    if (status != 0)
    {
        perror("sched_setaffinity");
    }
#endif
    const input_args globalArgs = parse_arguments(argc,argv);
    
    
    //set the parameters
    double results[globalArgs.rotation];
    
    std::string filename;
    
    if(globalArgs.type==1) {//synthetic dataset
        for (int k=0; k<globalArgs.rotation; k++){
            //create an object and set its parameters
            lisco::LinearClustering lisco;
            lisco.setMinClusterSize(10);
            lisco.setMaxClusterWithinDistance(globalArgs.epsilon);
            lisco.setMaxClusterWithinDistancePow(pow(globalArgs.epsilon,2));
            for(int i=1; i<120; i++){
                lisco.lidar.processLayers.push_back(ceil(asin(lisco.getMaxClusterWithinDistance()/i)/lisco.lidar.getMinElevationDifference()));
                lisco.lidar.processSteps.push_back(ceil(asin(lisco.getMaxClusterWithinDistance()/i)/lisco.lidar.getMinYawDifference()));
            }
            
            //read synthetic dataset once
#ifdef __APPLE__
            filename = "/Users/hannah/repositories/synthetic_data/SphericalData"+std::to_string(globalArgs.fileNumber)+".txt";
#else
            filename = "../../../synthetic_data/SphericalData"+std::to_string(globalArgs.fileNumber)+".txt";
#endif
            lisco.lidar.initialize(filename);
            readData(filename, lisco.lidar.getLidarVerticalOffset(), lisco.points);
            
            auto sEnd = std::chrono::high_resolution_clock::now();
            auto begin = std::chrono::high_resolution_clock::now();
            for(int j=0; j<lisco.lidar.getNumberOfSteps(); j++){
                lisco.linearStreamClustering(j);
                if(j!=0 && j%6==0){
                    sEnd = std::chrono::high_resolution_clock::now();
                    while( (std::chrono::duration_cast<std::chrono::microseconds>(sEnd-begin)).count() < (j/6)*288 ){
                        sEnd = std::chrono::high_resolution_clock::now();
                    }
                }
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto diff_sec = (std::chrono::duration_cast<std::chrono::milliseconds>(end-begin)).count();
            results[k] = diff_sec;
            if(k==globalArgs.rotation-1){
                std::cout << "epsilon: " << globalArgs.epsilon << std::endl;
                printResults(results, globalArgs.rotation, lisco.hashList, lisco.getMinClusterSize());
                //writeSyntheticResults(globalArgs.fileNumber, globalArgs.epsilon, lisco.lidar.getNumberOfSteps(), lisco.lidar.getNumberOfLasers(), lisco.hashList, lisco.points, lisco.getMinClusterSize());
                std::cout << std::endl;
            }
        }
        
        std::ofstream outfile("../../results/synthetic_sequential_results_scen"+std::to_string(globalArgs.fileNumber)+"_eps"+std::to_string(globalArgs.epsilon)+".txt");
        if (outfile.is_open())
        {
            for(int i=0; i<globalArgs.rotation; i++){
                outfile << results[i] << "\n";
            }
            outfile.close();
        }
        else std::cout << "Unable to open output file\n";
    }
    else{
        int maxNumberOfFiles = 2280;
        
        for (int fileNum=globalArgs.fileNumber, k=0; fileNum<globalArgs.fileNumber+globalArgs.rotation && fileNum<=maxNumberOfFiles; fileNum++, k++){
            //create and object and set its parameters
            lisco::LinearClustering lisco;
            lisco.setMinClusterSize(10);
            lisco.setMaxClusterWithinDistance(globalArgs.epsilon);
            lisco.setMaxClusterWithinDistancePow(pow(globalArgs.epsilon,2));
            for(int i=1; i<120; i++){
                lisco.lidar.processLayers.push_back(ceil(asin(lisco.getMaxClusterWithinDistance()/i)/lisco.lidar.getMinElevationDifference()));
                lisco.lidar.processSteps.push_back(ceil(asin(lisco.getMaxClusterWithinDistance()/i)/lisco.lidar.getMinYawDifference()));
            }
            
            //read data for every iteration since since this is a loop over different rotations
#ifdef __APPLE__
            filename = "/Users/hannah/repositories/real_data/SphericalData"+std::to_string(fileNum)+".txt";
#else
            filename = "../../../real_data/SphericalData"+std::to_string(fileNum)+".txt";
#endif
            
            lisco.lidar.initialize(filename);
            readData(filename, lisco.lidar.getLidarVerticalOffset(), lisco.points);
            
            auto sEnd = std::chrono::high_resolution_clock::now();
            auto begin = std::chrono::high_resolution_clock::now();
            for(int j=0; j<lisco.lidar.getNumberOfSteps(); j++){
                lisco.linearStreamClustering(j);
                if(j!=0 && j%6==0){
                    sEnd = std::chrono::high_resolution_clock::now();
                    while( (std::chrono::duration_cast<std::chrono::microseconds>(sEnd-begin)).count() < (j/6)*288 ){
                        sEnd = std::chrono::high_resolution_clock::now();
                    }
                }
            }
            auto end = std::chrono::high_resolution_clock::now();
            
            auto diff_sec = (std::chrono::duration_cast<std::chrono::milliseconds>(end-begin)).count();
            results[k] = diff_sec;
            if(fileNum==globalArgs.fileNumber+globalArgs.rotation-1 || fileNum==maxNumberOfFiles){
                std::cout << "epsilon: " << globalArgs.epsilon << std::endl;
                printResults(results, globalArgs.rotation, lisco.hashList, lisco.getMinClusterSize());
                std::cout << std::endl;
            }
        }
        
        std::ofstream outfile("../../results/real_sequential_results_eps"+std::to_string(globalArgs.epsilon)+".txt");
        if (outfile.is_open())
        {
            for(int i=0; i<globalArgs.rotation; i++){
                outfile << results[i] << "\n";
            }
            outfile.close();
        }
        else std::cout << "Unable to open output file\n";
    }
    return 0;
}
