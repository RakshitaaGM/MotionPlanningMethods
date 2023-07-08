
#include "Astar.h"
#include <cmath>
#include <algorithm>
namespace pathPlanning
{
    float Astar::calculateHeuristics(Node newNode)
{
    return sqrt(std::pow((goal.x - newNode.x), 2) + pow((goal.y - newNode.y),2));
}

bool Astar::isValid(Node currNode)
{
    if(currNode.x < 0 || currNode.x >= nrows || currNode.y < 0 || currNode.y >= ncols)
    {
        return false;
    }
    return true;
}

bool Astar::isObstacle(Node currNode)
{
    if(m_grid[currNode.x][currNode.y] != 100)  return true;
    return false;
}

bool Astar::isGoal(Node currNode)
{
    if(currNode.x == goal.x && currNode.y == goal.y) return true;
    return false;
}

void Astar::tracePath(std::vector<std::vector<Node>> gridInfo)
{
    if(gridInfo[goal.x][goal.y].parent->x == -1 &&  gridInfo[goal.x][goal.y].parent->y == -1)
    {
        std::cout << "Goal not found " << std::endl;
    }
    int r = gridInfo[goal.x][goal.y].x;
    int c = gridInfo[goal.x][goal.y].y;

    while(!(r == start.x) ||
          !(c == start.y))
    {
        finalPath.emplace_back(std::make_pair(r,c));
        auto t_r = gridInfo[r][c].parent->x;
        auto t_c = gridInfo[r][c].parent->y;
        r = t_r;
        c = t_c;
    }
    finalPath.push_back(std::make_pair(r,c));
    std::reverse(finalPath.begin(), finalPath.end());
    if(finalPath.size() != 0)
    {
      for(uint i = 0; i < finalPath.size() ;i++)
      {
        std::cout << "(" << finalPath[i].first << "," <<finalPath[i].second << ") " << std::endl;
      }
    }
    return;
}

void Astar::findPath()
{
    if(!isValid(start))
    {
        std::cout << "Start is invalid" << std::endl;
    }

    if(!isValid(goal))
    {
        std::cout << "Start is invalid" << std::endl;
    }

    if(isObstacle(start))
    {
        std::cout << "Start on the obstacle" << std::endl;
    }

    if(isObstacle(goal))
    {
        std::cout << "goal on the obstacle" << std::endl;
    }

    if(isGoal(start))
    {
        std::cout << "Start is already at the goal" << std::endl;
    }

    std::vector<std::vector<Node>> gridInfo;
    gridInfo.reserve(nrows);
    // std::cout << "<-------gridInfo is created---------------->"<<  std::endl;
    for(int i = 0; i < nrows; i++)
    {
        std::vector<Node> grid;
        grid.reserve(ncols);
        for(int j = 0; j < ncols; j++)
        {
           Node node;
           node.x = i;
           node.y = j;
           node.f = 0.0;
           node.g = 0.0;
           node.h = 0.0;
           node.parent = new Node;
           int* x = new int;
           *x = -1;
           int* y = new int;
           *y = -1;
           node.parent->x = *x;
           node.parent->y = *y;
           grid.emplace_back(node);
        }
        gridInfo.emplace_back(grid);
    }

    // std::cout << "<-------Done with gridInfo init---------------->"<<  std::endl;
    // Initialising the first node with start
    gridInfo[start.x][start.y].f = 0.0;
    gridInfo[start.x][start.y].g = 0.0;
    gridInfo[start.x][start.y].h = 0.0;
    gridInfo[start.x][start.y].parent->x = start.x;
    gridInfo[start.x][start.y].parent->y = start.y;
    // std::cout << "<-------Done with start assignment ---------------->"<<  std::endl;

    //Adding start node to open list
    openList.emplace_back(std::make_pair(gridInfo[start.x][start.y].f, start));
    // std::cout << "<------ Pushed start into open list ---------------->"<<  std::endl;
    std::vector<std::vector<bool>> closedList;
    closedList.reserve(nrows);
    for(uint i = 0; i< nrows; i++)
    {
        std::vector<bool> rows;
        rows.reserve(ncols);
        for(uint j = 0; j < ncols; j++)
        {
            rows.emplace_back(false);
        }
        closedList.emplace_back(rows);
    }
    // std::cout << "closedList " << closedList.size() << " " << closedList[0].size() << std::endl;
    while(!openList.empty())
    {
        const auto first = openList.begin();
        // std::cout << "first " << first->first << " " << std::endl;
        openList.erase(openList.begin());
        Node currNode = first->second;
        closedList[first->second.x][first->second.y] = true;
        // std::cout << "<-----closedList --> "<< closedList[first->second.x][first->second.y] << std::endl;
        for(auto m : move)
        {
            int move_x = m[0];
            int move_y = m[1];
            if(move_x == 0 && move_y == 0) continue;
            Node newNode;
            newNode.x = currNode.x + move_x;
            newNode.y = currNode.y + move_y;
            // std::cout << "<----newNode----> " << newNode.x << " " << newNode.y << std::endl;
            if(isValid(newNode) && !isObstacle(newNode))
            {
                if(isGoal(newNode))
                {
                    gridInfo[newNode.x][newNode.y].parent->x = currNode.x;
                    gridInfo[newNode.x][newNode.y].parent->y = currNode.y;
                    tracePath(gridInfo);
                    return;
                }
                else if(!closedList[newNode.x][newNode.y])
                {
                   float g = gridInfo[newNode.x][newNode.y].g + 1.0;
                   float h = gridInfo[newNode.x][newNode.y].h + calculateHeuristics(newNode);
                   float f = g + h;
                    // std::cout << "<------ f: " << f <<  " g : "  << g << " h: " << h << std::endl;
                   if(gridInfo[newNode.x][newNode.y].f == 0.0 ||
                      gridInfo[newNode.x][newNode.y].f > f)
                    {
                        gridInfo[newNode.x][newNode.y].f = f;
                        gridInfo[newNode.x][newNode.y].g = g;
                        gridInfo[newNode.x][newNode.y].h = h;
                        gridInfo[newNode.x][newNode.y].parent->x = currNode.x;
                        gridInfo[newNode.x][newNode.y].parent->y = currNode.y;
                        openList.emplace_back(std::make_pair(f, newNode));
                    }
                }
            }
        }
    }
}

void Astar::setStartNode(Node s)
{
    start.x = s.x;
    start.y = s.y;
}

void Astar::setGoalNode(Node g)
{
    goal.x = g.x;
    goal.y = g.y;
}

void Astar::setGrid(std::vector<std::vector<int>>& grid)
{
    m_grid = grid;
}
}


//  std::pair<double, Node> Astar::findMinF()
//  {
//     if(openList.size() == 1 || openList.begin()->first == __FLT_MAX__)
//     {
//         return openList[0];
//     }
//     float minF = __FLT_MAX__;
//     Node minNode;
//     std::cout << "in find Min F " << std::endl;
//     for(auto e : openList)
//     {
//         std::cout << e.second.x << " " << e.second.y << " f " << e.first << std::endl;
//         if(e.first < minF)
//         {
//             std::cout << e.second.x << " " << e.second.y << std::endl;
//             minF = e.first;
//             minNode = e.second;
//         }
//     }
//     return std::make_pair(minF, minNode);
//  }

//  int main()
//  {
//     pathPlanning::Astar astarPlan;
//     pathPlanning::Node start;
//     start.x = 0;
//     start.y = 0;
//     pathPlanning::Node goal;
//     goal.x = 109;
//     goal.y = 177;
//     astarPlan.setStartNode(start);
//     astarPlan.setGoalNode(goal);
//     astarPlan.findPath();
//     return 0;
//  }
