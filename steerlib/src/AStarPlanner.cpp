
//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm>
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define EUCLIDEAN 1
#define DISTANCE 1

namespace SteerLib
{
    AStarPlanner::AStarPlanner(){}
    
    AStarPlanner::~AStarPlanner(){}
    
    bool AStarPlanner::canBeTraversed ( int id )
    {
        double traversal_cost = 0;
        int current_id = id;
        unsigned int x,z;
        gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
        int x_range_min, x_range_max, z_range_min, z_range_max;
        
        x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
        x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());
        
        z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
        z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());
        
        
        for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
        {
            for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
            {
                int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
                traversal_cost += gSpatialDatabase->getTraversalCost ( index );
                
            }
        }
        
        if ( traversal_cost > COLLISION_COST )
            return false;
        return true;
    }
    
    
    
    Util::Point AStarPlanner::getPointFromGridIndex(int id)
    {
        Util::Point p;
        gSpatialDatabase->getLocationFromIndex(id, p);
        return p;
    }
    
    double manhattan(Util::Point s, Util::Point g){
        double distance;
        distance = abs(s.x - g.x) + abs(s.z - g.z);
        return distance;
    }
    
    double euclidean(Util::Point s, Util::Point g){
        double distance;
        distance = sqrt(pow((s.x - g.x), 2) + pow((s.z - g.z), 2));
        //std::cout << distance << std::endl;
        return distance;
    }
    
    std::vector<AStarPlannerNode> AStarPlanner::findNeighbor(Util::Point p1, SteerLib::GridDatabase2D* gSpatialDatabase){
        int index;
        std::vector<AStarPlannerNode> result;
        Util::Point p;
        unsigned x, z;
        
        index = gSpatialDatabase->getCellIndexFromLocation(p1);
        gSpatialDatabase->getGridCoordinatesFromIndex(index, x, z);
        
        index = gSpatialDatabase->getCellIndexFromGridCoords(x, z - 1);
        if(canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            AStarPlannerNode temp = AStarPlannerNode(p, DBL_MAX, DBL_MAX, NULL);
            result.push_back(temp);
        }
        
        index = gSpatialDatabase->getCellIndexFromGridCoords(x+1, z);
        if(canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            AStarPlannerNode temp = AStarPlannerNode(p, DBL_MAX, DBL_MAX, NULL);
            result.push_back(temp);
        }
        
        index = gSpatialDatabase->getCellIndexFromGridCoords(x, z+1);
        if(canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            AStarPlannerNode temp = AStarPlannerNode(p, DBL_MAX, DBL_MAX, NULL);
            result.push_back(temp);
        }
        
        index = gSpatialDatabase->getCellIndexFromGridCoords(x-1, z);
        if(canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            AStarPlannerNode temp = AStarPlannerNode(p, DBL_MAX, DBL_MAX, NULL);
            result.push_back(temp);
        }
        
        index = gSpatialDatabase->getCellIndexFromGridCoords(x-1, z-1);
        if(canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            AStarPlannerNode temp = AStarPlannerNode(p, DBL_MAX, DBL_MAX, NULL);
            result.push_back(temp);
        }
        
        index = gSpatialDatabase->getCellIndexFromGridCoords(x+1, z-1);
        if(canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            AStarPlannerNode temp = AStarPlannerNode(p, DBL_MAX, DBL_MAX, NULL);
            result.push_back(temp);
        }
        
        index = gSpatialDatabase->getCellIndexFromGridCoords(x+1, z+1);
        if(canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            AStarPlannerNode temp = AStarPlannerNode(p, DBL_MAX, DBL_MAX, NULL);
            result.push_back(temp);
        }
        
        
        index = gSpatialDatabase->getCellIndexFromGridCoords(x-1, z+1);
        if(canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            AStarPlannerNode temp = AStarPlannerNode(p, DBL_MAX, DBL_MAX, NULL);
            result.push_back(temp);
        }
        /*
         std::cout << "new neighbors" << std::endl;
         for(int i = 0; i < result.size(); i++){
         std::cout << result[i].point << std::endl;
         }
         */
        
        
        return result;
    }
    
    double heuristic(Util::Point s, Util::Point g){
        if(EUCLIDEAN)
            return euclidean(s, g);
        else
            return manhattan(s, g);
    }
    
    bool closed(AStarPlannerNode node, std::vector<AStarPlannerNode> closedset){
        for(int i = 0; i < closedset.size(); i++){
            if(node == closedset[i])
                return true;
        }
        return false;
    }
    
    bool opened(AStarPlannerNode node, std::vector<AStarPlannerNode> openset){
        for(int i = 0; i < openset.size(); i++){
            if(node == openset[i])
                return true;
        }
        return false;
    }
    
    bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
    {
        gSpatialDatabase = _gSpatialDatabase;
        
        //TODO
        std::cout<<"\nIn A*\n";
        //coordinate for start & goal in the cell
        int startIndex = gSpatialDatabase->getCellIndexFromLocation(start);
        gSpatialDatabase->getLocationFromIndex(startIndex, start);
        int goalIndex = gSpatialDatabase->getCellIndexFromLocation(goal);
        gSpatialDatabase->getLocationFromIndex(goalIndex, goal);
        
        std::vector<AStarPlannerNode> closedset;
        std::vector<AStarPlannerNode> openset;
        std::vector<AStarPlannerNode> neighbors;
        std::map<AStarPlannerNode, AStarPlannerNode> came;
        
        AStarPlannerNode startNode = AStarPlannerNode(start, 0, 0, NULL);
        startNode.startNode = 1;
        startNode.g = 0;
        startNode.f = startNode.g + heuristic(start, goal);
        
        openset.push_back(startNode);
        
        AStarPlannerNode curr;
        int count = 0;
        
        while(!openset.empty()){
            count++;
            //get Lowest F
            int index = 0;
            curr = openset[0];
            for(int i = 0; i < openset.size(); i++){
                if( openset[i] < curr){
                    curr = openset[i];
                    index = i;
                }
            }
            if(curr.point == goal){
                agent_path.push_back(curr.point);
                while(curr.startNode != 1){
                    for(AStarPlannerNode node: closedset){
                        if(node.point.x == curr.parentx && node.point.z == curr.parenty){
                            curr = node;
                            agent_path.push_back(curr.point);
                            break;
                        }
                    }
                }
                std::reverse(agent_path.begin(), agent_path.end());
                return true;
            }
            
            openset.erase(openset.begin() + index);
            closedset.push_back(curr);
            
            neighbors = findNeighbor(curr.point, gSpatialDatabase);
            for(AStarPlannerNode neighbor: neighbors){
                if(closed(neighbor, closedset)){
                    continue;
                }
                
                double temp_g = curr.g + DISTANCE;
                
                if(temp_g < neighbor.g){
                    neighbor.g = temp_g;
                    neighbor.f = neighbor.g + heuristic(neighbor.point, goal);
                    neighbor.parentx = curr.point.x;
                    neighbor.parenty = curr.point.z;
                    if(!opened(neighbor, openset)){
                        openset.push_back(neighbor);
                    }
                }
            }
        }
        return false;
    }
}