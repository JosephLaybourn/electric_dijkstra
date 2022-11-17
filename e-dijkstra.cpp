/*!
 * \author Joey Laybourn (j.laybourn@digipen.edu)
 * \file e-dijkstra.cpp
 * \date 6-21-20
 * \copyright Copyright (C) "2019" DigiPen Institute of Technology.
 * \brief This file contains the solution for Electric Dijkstra
 */

#include "e-dijkstra.h"
#include <vector>
#include <deque>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>

typedef std::vector<std::vector<int>> Graph;

struct EV
{
  // EV(int numRecharges_, int range_) : 
  //   numRecharges(numRecharges_),
  //   range(range_) { }
  // EV(const EV& cp) :
  //   numRecharges(cp.numRecharges),
  //   range(cp.range) { }
  // EV(EV&& rhs) :
  //   numRecharges(rhs.numRecharges),
  //   range(rhs.range) { }
  // EV& operator=(const EV& rhs)
  // {
  //   numRecharges = rhs.numRecharges;
  //   range = rhs.range;

  //   return *this;
  // }

  int numRecharges;
  int range;
};

Graph initGraph(std::fstream &input)
{
  unsigned size;
  int recharges;
  unsigned lineCount;

  input >> size;
  input >> recharges;
  input >> lineCount;

  Graph town(size);

  for(std::vector<int>& columns : town)
  {
    columns.resize(size);
  }

  for(unsigned i = 0; i < lineCount; ++i)
  {
    unsigned from;
    unsigned to;
    int weight;

    input >> from;
    input >> to;
    input >> weight;

    town[from][to] = weight;
    town[to][from] = weight;
  }

  return town;
}

void printGraph(Graph input)
{
  std::cout << "Input Graph: " << std::endl;
  std::cout << '{' << std::endl;

  for(std::vector<int>& columns : input)
  {
    for(int &distances : columns)
    {
      std::cout << std::setw(3) << distances << ", ";
    }
    std::cout << std::endl;
  }

  std::cout << '}' << std::endl;
}

// A utility function to find the vertex with minimum distance value, from 
// the set of vertices not yet included in shortest path tree 
int minDistance(std::vector<EV> dist, std::vector<bool> sptSet) 
{ 
    // Initialize min value 
    EV min = { 0, -1 };
    int min_index = 0; 
  
    for (unsigned v = 0; v < dist.size(); v++) 
    {
      //std::cout << "testing index: " << v << std::endl;

      if (!sptSet[v] && (dist[v].numRecharges >= min.numRecharges) && (dist[v].range > min.range)) 
      {
        min = dist[v];
        min_index = v; 
        //std::cout << "Found a new min: " << min << std::endl;
      }
    }
    return min_index; 
} 

EV travel(EV currentRange, int distance, int maxRange)
{
  if((currentRange.range - distance) < 0)
  {
    currentRange.range = maxRange;
    --currentRange.numRecharges;
  }
  currentRange.range -= distance;

  return currentRange;
}

bool e_dijkstra(char const *filename, int range)
{
  std::fstream input(filename);
  Graph town = initGraph(input);
  int maxRecharges;
  //EV vehicle;

  input.seekg(0);
  input.ignore(std::numeric_limits<int>::max(), ' ');
  input >> maxRecharges;

  //std::cout << "Max Recharges: " << maxRecharges << std::endl;
  //std::cout << "Max Range: " << range << std::endl;
  
  //printGraph(town);
  ////std::cout << "Range: " << range << std::endl;

  std::vector<EV> remainingCharge(town.size()); // The output array.  dist[i] will hold the shortest 
                                                // distance from src to i 
  std::vector<int> prev(town.size());
  
  std::vector<bool> sptSet(town.size()); // sptSet[i] will be true if vertex i is included in shortest 
                                         // path tree or shortest distance from src to i is finalized 
  
  
    
  for(unsigned source = 0; source < town.size(); ++source)
  {
    // Initialize all distances as INFINITE and stpSet[] as false 
    for (unsigned i = 0; i < town.size(); i++)
    {
      remainingCharge[i].numRecharges = 0;
      remainingCharge[i].range = -1;
      sptSet[i] = false; 
    } 
    //vehicle.numRecharges = maxRecharges;
    //vehicle.range = range;
    // Distance of source vertex from itself is always 0 
    remainingCharge[source].numRecharges = maxRecharges - 1;
    remainingCharge[source].range = range;
  
    // Find shortest path for all vertices 
    for (unsigned count = 0; count < town.size() - 1; count++) 
    { 
      // Pick the minimum distance vertex from the set of vertices not 
      // yet processed. u is always equal to src in the first iteration. 
      int u = minDistance(remainingCharge, sptSet); 

      //std::cout << "u is: " << u << std::endl;
  
      // Mark the picked vertex as processed 
      sptSet[u] = true; 
  
      // Update dist value of the adjacent vertices of the picked vertex. 
      for (unsigned v = 0; v < town.size(); v++) 
      {
        // Update dist[v] only if is not in sptSet, there is an edge from 
        // u to v, and total weight of path from src to  v through u is 
        // smaller than current value of dist[v] 
        // if (!sptSet[v] && town[u][v] && dist[u] != std::numeric_limits<int>::max() && dist[u] + town[u][v] < dist[v] && town[u][v] <= range) 
        // {
        //   dist[v] = dist[u] + town[u][v]; 
        //   prev[v] = u;
        //   std::cout << "Found Distance: " << dist[v] << std::endl;
        // }

        // if the vertex is not yet processed, and there is a legitimate path from u to v
        if(!sptSet[v] && town[u][v])
        {
          EV newDistance = travel(remainingCharge[u], town[u][v], range);

          // if there's a path that consumes less battery while keeping numRecharges >= the current distance
          if((newDistance.numRecharges >= remainingCharge[v].numRecharges) && (newDistance.range > remainingCharge[v].range))
          {
            remainingCharge[v] = newDistance;
            prev[v] = u;
          }
        }
      }
    } 

    // std::cout << "Shortest Distance" << std::endl;;
    // for(unsigned i = 0; i < remainingCharge.size(); ++i)
    // {
    //   std::cout << "Distance at index: " << i << " is: " << remainingCharge[i].numRecharges << ", " << remainingCharge[i].range << std::endl;
    // }
    // std::cout << std::endl;

    for(const EV& index : remainingCharge)
    {
      if(index.range == -1)
      {
        return false;
      }
    }
  }
  return true;
}

// Dijkstra( G, source)
// {
//     for each vertex v in G                  // Initialization
//     {
//         dist[v] = INFINITY                  // Unknown distance from source to v (you may use max instead of infinity)
//         prev[v] = UNDEFINED                 // Previous node in optimal path from source
//     }

//     dist[source] = 0                        // Distance from source to source
     
//     for i = 1 .. number of vertices         // i is not used, basically it says do V iterations
//     {
//          u = argmin( dist )                 // Find not yet evaluated node with the least estimated distance - GREEDY
//                                             // (also terminate if dist[u] is infinity - you are traversing nodes NOT connected to source
//          u.evaluated = true                 // this will be linear in number of nodes
         
//          for each neighbor v of u           // no more then V iterations
//             if ( v.evaluated ) continue     // skip - optimization

//             alt = dist[u] + length(u, v)    // relaxation
//             if alt < dist[v]:               // a shorter path to v has been found
//                 dist[v] = alt 
//                 prev[v] = u 
//     }
//     return dist[], prev[]
// }