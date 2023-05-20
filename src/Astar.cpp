
#include "Astar.h"
#include <cmath>
#include <algorithm>
float Astar::calculateHeuristics(Node newNode)
{
    return sqrt(std::pow((goal.x - newNode.x), 2) + pow((goal.y - newNode.y),2));
}

bool Astar::isValid(Node currNode)
{
    if(currNode.x < 0 || currNode.x >= TOTAL_NUMBER_ROWS || currNode.y < 0 || currNode.y >= TOTAL_NUMBER_COLS)
    {
        return false;
    }
    return true;
}

bool Astar::isObstacle(Node currNode)
{
    if(map[currNode.x][currNode.y] == 0)  return true;
    return false;
}

bool Astar::isGoal(Node currNode)
{
    if(currNode.x == goal.x && currNode.y == goal.y) return true;
    return false;
}

void Astar::tracePath(Node gridInfo[][TOTAL_NUMBER_COLS])
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
        finalPath.push_back(std::make_pair(r,c));
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

    if(isObstacle(start) || isObstacle(goal))
    {
        std::cout << "Start or goal is on the obstacle" << std::endl;
    }
    if(isGoal(start))
    {
        std::cout << "Start is already at the goal" << std::endl;
    }

    Node gridInfo[TOTAL_NUMBER_ROWS][TOTAL_NUMBER_COLS];
    for(int i = 0; i < TOTAL_NUMBER_ROWS; i++)
    {
        for(int j = 0; j < TOTAL_NUMBER_COLS; j++)
        {
           gridInfo[i][j].x = i;
           gridInfo[i][j].y = j;
           gridInfo[i][j].f = __FLT_MAX__;
           gridInfo[i][j].g = __FLT_MAX__;
           gridInfo[i][j].h = __FLT_MAX__;
           gridInfo[i][j].parent = new Node;
           int* x = new int;
           *x = -1;
           gridInfo[i][j].parent->x = *x;
           gridInfo[i][j].parent->x = *x;
        }
    }

    // Initialising the first node with start
    gridInfo[start.x][start.y].f = 0.0;
    gridInfo[start.x][start.y].g = 0.0;
    gridInfo[start.x][start.y].h = 0.0;
    gridInfo[start.x][start.y].parent->x = start.x;
    gridInfo[start.x][start.y].parent->y = start.y;

    //Adding start node to open list
    openList.push_back(std::make_pair(gridInfo[start.x][start.y].f, start));

    while(!openList.empty())
    {
        const auto first = openList.begin();
        openList.erase(openList.begin());
        Node currNode = first->second;
        closedList[first->second.x][first->second.y] = true;
        for(auto m : move)
        {
            int move_x = m[0];
            int move_y = m[1];
            if(move_x == 0 && move_y == 0) continue;
            Node newNode;
            newNode.x = currNode.x + move_x;
            newNode.y = currNode.y + move_y;
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

                   if(gridInfo[newNode.x][newNode.y].f == __FLT_MAX__ ||
                      gridInfo[newNode.x][newNode.y].f > f)
                    {
                        // auto n = findMinF();
                        // std::cout << "Min Node !!!!!!!!!!! " << n.second.x << " " << n.second.y << std::endl;
                        gridInfo[newNode.x][newNode.y].f = f;
                        gridInfo[newNode.x][newNode.y].g = g;
                        gridInfo[newNode.x][newNode.y].h = h;
                        gridInfo[newNode.x][newNode.y].parent->x = currNode.x;
                        gridInfo[newNode.x][newNode.y].parent->y = currNode.y;
                        openList.push_back(std::make_pair(f, newNode));
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

 int main()
 {
    Astar astarPlan;
    Node start;
    start.x = 0;
    start.y = 0;
    Node goal;
    goal.x = 9;
    goal.y = 7;
    astarPlan.setStartNode(start);
    astarPlan.setGoalNode(goal);
    astarPlan.findPath();
    return 0;
 }
