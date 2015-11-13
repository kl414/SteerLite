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
    
    std::vector<Util::Point> reconstruct_path(std::vector<AStarPlannerNode*> came_from, AStarPlannerNode* curr){
        std::vector<Util::Point> path;
        return path;
    }

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*\n";

        std::vector<AStarPlannerNode*> closedsed;
        std::vector<AStarPlannerNode*> openset;
        std::vector<AStarPlannerNode*> came_from;
        
        double g = 0;
        double f = g + euclidean(start, goal);
        
        AStarPlannerNode* startNode = new AStarPlannerNode(start, g, f, NULL);
        openset.push_back(startNode);
        
        while(!openset.empty()){
            
            AStarPlannerNode* curr = openset[0];
            
            int index = 0;
            for(int i = 1; i < openset.size(); i++){
                if(openset[i] < curr){
                    curr = openset[i];
                    index = i;
                }
            }
            
            if(curr->point == goal){
                agent_path = reconstruct_path(came_from, curr);
                return true;
            }
            
            openset.erase(openset.begin() + index);
            closedsed.push_back(curr);
            
        }
        
		return false;
	}
}
