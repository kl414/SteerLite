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
    
    std::vector<Util::Point> reconstruct_path(std::map<AStarPlannerNode, AStarPlannerNode> came_from, AStarPlannerNode curr){
        std::vector<Util::Point> path;
        path.push_back(curr.point);
        return path;
    }
    
    std::vector<AStarPlannerNode> findNeighbor(Util::Point p1, SteerLib::GridDatabase2D* gSpatialDatabase){
        int index;
        float x = p1.x;
        float z = p1.z;
        std::vector<AStarPlannerNode> result;
        Util::Point p;
        AStarPlanner* planner = new AStarPlanner();
       
        index = gSpatialDatabase->getCellIndexFromLocation(x, z - 1);
        if(planner->canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            result.push_back(AStarPlannerNode(p, 0, 0, NULL));
        }
        
        index = gSpatialDatabase->getCellIndexFromLocation(x+1, z);
        if(planner->canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            result.push_back(AStarPlannerNode(p, 0, 0, NULL));
        }
        
        index = gSpatialDatabase->getCellIndexFromLocation(x, z+1);
        if(planner->canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            result.push_back(AStarPlannerNode(p, 0, 0, NULL));
        }
        
        index = gSpatialDatabase->getCellIndexFromLocation(x-1, z);
        if(planner->canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            result.push_back(AStarPlannerNode(p, 0, 0, NULL));
        }
        
        index = gSpatialDatabase->getCellIndexFromLocation(x-1, z-1);
        if(planner->canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            result.push_back(AStarPlannerNode(p, 0, 0, NULL));
        }
        
        index = gSpatialDatabase->getCellIndexFromLocation(x+1, z-1);
        if(planner->canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            result.push_back(AStarPlannerNode(p, 0, 0, NULL));
        }
        
        index = gSpatialDatabase->getCellIndexFromLocation(x+1, z+1);
        if(planner->canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            result.push_back(AStarPlannerNode(p, 0, 0, NULL));
        }
        
        
        index = gSpatialDatabase->getCellIndexFromLocation(x-1, z+1);
        if(planner->canBeTraversed(index)){
            gSpatialDatabase->getLocationFromIndex(index, p);
            result.push_back(AStarPlannerNode(p, 0, 0, NULL));
        }
        
        return result;
    }

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*\n";

        std::vector<AStarPlannerNode> closedset;
        std::vector<AStarPlannerNode> openset;
        std::map<AStarPlannerNode, AStarPlannerNode> came_from;
        std::map<Util::Point, int> visited;
       
        std::map<Util::Point, double> g_score;
        g_score[start] = 0;
        std::map<Util::Point, double> f_score;
        f_score[start] = g_score[start] + euclidean(start, goal);
        
        AStarPlannerNode startNode = AStarPlannerNode(start, g_score[start], f_score[start], NULL);
        visited[start] = 1;
        openset.push_back(startNode);
        
        while(!openset.empty()){
            
            AStarPlannerNode curr = openset[0];
            
            int index = 0;
            for(int i = 1; i < openset.size(); i++){
                if(openset[i] < curr){
                    curr = openset[i];
                    index = i;
                }
            }
            
            if(curr.point == goal){
                agent_path = reconstruct_path(came_from, curr);
                return true;
            }
            
            openset.erase(openset.begin() + index);
            closedset.push_back(curr);

            int currIndex = gSpatialDatabase->getCellIndexFromLocation(curr.point);
            Util::Point p1;
            gSpatialDatabase->getLocationFromIndex(currIndex, p1);
            std::vector<AStarPlannerNode> neighbors = findNeighbor(p1, gSpatialDatabase);

            for(AStarPlannerNode neighbor: neighbors){
                //std::cout << node.point << std::endl;
                int flag = 0;
                for(AStarPlannerNode temp: closedset){
                    if(neighbor == temp){
                        flag = 1;
                        break;
                    }
                }

                if(flag == 1)
                    continue;
                
                //this is calculating the differnece, euclidean here is not the heuristic function
                double temp_g_score = g_score[curr.point] + euclidean(curr.point, neighbor.point);

                if(visited[neighbor.point] == 0 || temp_g_score < g_score[neighbor.point]){
                    g_score[neighbor.point] = temp_g_score;
                    f_score[neighbor.point] = g_score[neighbor.point] + euclidean(neighbor.point, goal);
                    neighbor.g = g_score[neighbor.point];
                    neighbor.f = f_score[neighbor.point];
                    came_from[neighbor] = curr;
                    if(visited[neighbor.point] == 0){
                        visited[neighbor.point] = 1;
                        openset.push_back(neighbor);
                    }
                }
            }
        }
        
		return false;
	}
}
