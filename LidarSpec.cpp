//
//  LidarSpec.cpp
//  Lisco
//
//  Created by Hannah Ataei on 2017-05-22.
//  Copyright © 2017 Hannah Ataei. All rights reserved.
//

#include "LidarSpec.hpp"

void LidarClass::initialize(std::string fileName){
    
    std::ifstream infile(fileName);
    if (infile.is_open())
    {
        std::string l,s,u;
        if(std::getline(infile, l, ',') &&
           std::getline(infile, s, ',') &&
           std::getline(infile, u)){
            this->setNumberOfLasers(std::stoi(l));
            this->setNumberOfSteps(std::stoi(s));
        }
        
        for(int i=0; i<getNumberOfLasers(); i++){
            std::string d,t,a;
            if(std::getline(infile, d, ',') &&
               std::getline(infile, t, ',') &&
               std::getline(infile, a)){
                initial_yaw_offset_.push_back(std::stod(a));
            }
        }
        infile.close();
    }
    else std::cout << "Unable to open file\n";
    
    setAlphaMatrix();
}

void LidarClass::setAlphaMatrix(){
    std::vector<int> temp;
    for(int i=0; i<getNumberOfLasers(); i++){
        temp.clear();
        for(int j=0; j<getNumberOfLasers(); j++){
            if(i==j){
                temp.push_back(0);
            }
            else{
                temp.push_back(ceil((initial_yaw_offset_[i]-initial_yaw_offset_[j])/getMinYawDifference()));
            }
        }
        alpha_matrix.push_back(temp);
    }
}
