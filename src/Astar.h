#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#ifndef ASTAR_H
#define ASTAR_H

namespace pathPlanning {
    struct Node
{
    int x; // x coordinate of the node
    int y; // y coordinate of the node
    float theta; // for future use
    float f; // f = g + h
    float g; // cost from starting node to the current node
    float h; // heuristics - the cost to move from curr node to goal node
    Node* parent;
    Node operator() (const int x, const int y) 
    {
        Node n;
        n.x = x;
        n.y = y;
        return n;
    }
};

class Astar 
{
    public:
        Astar(Node m_start, Node m_goal)
        {
            start = m_start;
            goal = m_goal;
        }
        Astar() {}
        Node start;
        Node goal;
        double ncols;
        double nrows;
        std::vector<std::vector<int>> m_grid ;
        // {{1, 0, 1, 1, 0, 0, 0, 1, 1, 1},
        //  {1, 0, 1, 1, 0, 0, 0, 1, 1, 1},
        //  {0, 1, 0, 1, 1, 1, 1, 1, 1, 1},
        //  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        //  {1, 1, 1, 1, 0, 0, 0, 1, 1, 1},
        //  {1, 1, 1, 1, 0, 0, 0, 1, 1, 1},
        //  {0, 0, 0, 1, 0, 0, 0, 1, 1, 1},
        //  {0, 0, 0, 1, 1, 1, 1, 1, 1, 1},
        //  {1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
        //  {1, 1, 1, 1, 1, 1, 1, 1, 0, 0}};
        std::vector<std::pair<double, Node>> openList; // vector<pair(f, Node(x, y))>
        std::vector<std::vector<int>> move = {
        {-1, 1},  {0, 1},   {1, 1},
        {-1, 0},  {0, 0},   {1, 0},
        {-1, -1}, {0, -1}, {1, -1}
        };
        // std::vector<std::vector<bool>> closedList = {{false}};
    
        float calculateHeuristics(Node NewNode);
        bool isValid(Node currNode);
        bool isObstacle(Node currNode);
        bool isGoal(Node currNode);
        
        void tracePath(std::vector<std::vector<Node>> grid);
        std::pair<double, Node> findMinF();
        std::vector<std::pair<int, int>> finalPath;
        // Setters and getters
        void setStartNode(Node s);
        void setGoalNode(Node g);
        void setGrid(std::vector<std::vector<int>>& grid);
        void findPath();
};

}

#endif 