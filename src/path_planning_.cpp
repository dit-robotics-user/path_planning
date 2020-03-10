#include "ros/ros.h"
#include "std_msgs/String.h"
#include "path_planning/path.h"
#include "path_planning/AddTwoInts.h"
#include <cstdlib>

#include <iostream>
#include <queue>
#include <vector>
#include <cmath>


using namespace std;

#define mazeSizeX 62
#define mazeSizeY 42
#define robotSize 9

void build_maze(int maze[mazeSizeX][mazeSizeY])   //build the maze
{
    // maze field
    for(int j = 0; j < mazeSizeY; j++)
    {
        for(int i = 0; i < mazeSizeX; i++)
        {
            maze[i][j] = 1;
        }
    }
    // adding walls
    // top and bottom walls
    for(int i = 0; i < mazeSizeX; i++)
    {
        maze[i][0] = 0;
        maze[i][mazeSizeY-1] = 0;
    }
    //left and right walls
    for(int i = 0; i < mazeSizeY; i++)
    {
        maze[0][i] = 0;
        maze[mazeSizeX-1][i] = 0;
    }
}

void build_obstacles(int middlePoint[2], int wallThick, int maze[mazeSizeX][mazeSizeY]){
    int middlePointX = middlePoint[0];
    int middlePointY = middlePoint[1];
    int halfWallThick = (wallThick-1)/2;
    for(int j = middlePointY-halfWallThick; j < middlePointY+halfWallThick; j++)
    {
        for(int i = middlePointX-halfWallThick; i < middlePointX + halfWallThick; i++)
        {
            if(i>0&&i<mazeSizeX&&j>0&&j<mazeSizeY)
            {
                maze[i][j] = 0;
            }
        }
    }
}

class PosNode   //used to store path;
{
public:
    int pos[2];
};

class Node
{
public:
    Node()
    {
        g = 0;
        h = 0;
        f = 0;
    }
    double g;
    double h;
    double f;
    double move_cost;
    int priority_stamp;
    vector<PosNode> path;

    int PosX()
    {
        return pos[0];
    }
    int PosY()
    {
        return pos[1];
    }
    void PrintPos()
    {
        cout << "Pos: (" << pos[0] << ", " << pos[1] << "\n";
    }
    void GivePos(int new_pos[2])
    {
        pos[0] = new_pos[0];
        pos[1] = new_pos[1];
    }
private:
    int pos[2];
};

void Up(int* new_pos, int pos[2])
{
    new_pos[0] = pos[0];
    new_pos[1] = pos[1];
    new_pos[1] -= 1;
}
void UpRight(int* new_pos, int pos[2])
{
    new_pos[0] = pos[0];
    new_pos[1] = pos[1];
    new_pos[0] += 1;
    new_pos[1] -= 1;
}
void Right(int* new_pos, int pos[2])
{
    new_pos[0] = pos[0];
    new_pos[1] = pos[1];
    new_pos[0] += 1;
}
void DownRight(int* new_pos, int pos[2])
{
    new_pos[0] = pos[0];
    new_pos[1] = pos[1];
    new_pos[0] += 1;
    new_pos[1] += 1;
}
void Down(int* new_pos, int pos[2])
{
    new_pos[0] = pos[0];
    new_pos[1] = pos[1];
    new_pos[1] += 1;
}
void DownLeft(int* new_pos, int pos[2])
{
    new_pos[0] = pos[0];
    new_pos[1] = pos[1];
    new_pos[0] -= 1;
    new_pos[1] += 1;
}
void Left(int*new_pos, int pos[2])
{
    new_pos[0] = pos[0];
    new_pos[1] = pos[1];
    new_pos[0] -= 1;
}
void UpLeft(int* new_pos, int pos[2])
{
    new_pos[0] = pos[0];
    new_pos[1] = pos[1];
    new_pos[0] -= 1;
    new_pos[1] -= 1;
}

bool isValid(int new_pos[2], int maze[mazeSizeX][mazeSizeY])
{
    if(new_pos[0] <=0 || new_pos[0]>= mazeSizeX-1)
    {
        return false;
    }
    if(new_pos[1] <=0 || new_pos[1]>= mazeSizeY-1)
    {
        return false;
    }
    if(maze[new_pos[0]][new_pos[1]] == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

vector<Node> FindSuccessors(int pos[2], int maze[mazeSizeX][mazeSizeY])
{
    double strait = 1;
    double diagnal = sqrt(2);
    vector<Node> output;
    Node successor;
    int new_pos[2];
    Up(new_pos, pos);
    if(isValid(new_pos, maze)==true)
    {
        successor.GivePos(new_pos);
        successor.move_cost = strait;
        output.push_back(successor);
    }
    UpRight(new_pos, pos);
    if(isValid(new_pos, maze)==true)
    {
        successor.GivePos(new_pos);
        successor.move_cost = diagnal;
        output.push_back(successor);
    }
    Right(new_pos, pos);
    if(isValid(new_pos, maze)==true)
    {
        successor.GivePos(new_pos);
        successor.move_cost = strait;
        output.push_back(successor);
    }
    DownRight(new_pos, pos);
    if(isValid(new_pos, maze)==true)
    {
        successor.GivePos(new_pos);
        successor.move_cost = diagnal;
        output.push_back(successor);
    }
    Down(new_pos, pos);
    if(isValid(new_pos, maze)==true)
    {
        successor.GivePos(new_pos);
        successor.move_cost = strait;
        output.push_back(successor);
    }
    DownLeft(new_pos, pos);
    if(isValid(new_pos, maze)==true)
    {
        successor.GivePos(new_pos);
        successor.move_cost = diagnal;
        output.push_back(successor);
    }
    Left(new_pos, pos);
    if(isValid(new_pos, maze)==true)
    {
        successor.GivePos(new_pos);
        successor.move_cost = strait;
        output.push_back(successor);
    }
    UpLeft(new_pos, pos);
    if(isValid(new_pos, maze)==true)
    {
        successor.GivePos(new_pos);
        successor.move_cost = diagnal;
        output.push_back(successor);
    }
    return output;
}



double HeuristicFunctionManhattan(Node start, Node goal)
{
    double cost = abs(start.PosX() - goal.PosX()) + abs(start.PosY() - goal.PosY());
    return cost;
}

double HeuristicFunctionDiagnal(Node start, Node goal)
{
    int strait = 1;
    double diagnal = sqrt(2);
    int dx = abs(start.PosX()-goal.PosX());
    int dy = abs(start.PosY()-goal.PosY());
    double cost = strait*(dx + dy) + (diagnal - 2*strait)*min(dx, dy);
    //double cost = min(dx,dy)*D2+D*(max(dx,dy)-min(dx,dy));
    //cout << "dx: " << dx << ",dy: " << dy << ",cost: " << cost << "\n";
    return cost;
}

void PrintPath(vector<PosNode> p, int maze[mazeSizeX][mazeSizeY])
{
    cout << "path: ";
    for(unsigned int i = 0; i < p.size(); i++)
    {
        cout << "(" << p[i].pos[0] << "," << p[i].pos[1] << ") ";
        maze[p[i].pos[0]][p[i].pos[1]] = 8;  //print out path as 8

    }
    cout << "\n";
}

void PrintMaze(int start[2], int goal[2], int maze[mazeSizeX][mazeSizeY])      // print out maze
{
    cout << "\n";
    maze[start[0]][start[1]] = 5;  //set maze start point as 5
    maze[goal[0]][goal[1]] = 9;  //set maze goal point as 9
    char cmaze[mazeSizeX][mazeSizeY];
    //print out the maze
    for(int j = 0; j < mazeSizeY; j++)
    {
        for(int i = 0; i < mazeSizeX; i++)
        {
            if(maze[i][j] == 0)
            {
                cmaze[i][j] = 'H';
            }
            else if(maze[i][j] == 8)
            {
                cmaze[i][j] = '#';
            }
            else if(maze[i][j] == 4)
            {
                cmaze[i][j] = '~';
            }
            else if(maze[i][j] == 1)
            {
                cmaze[i][j] = '.';
            }
            else if(maze[i][j] == 5)
            {
                cmaze[i][j] = 'S';
            }
            else if(maze[i][j] == 9)
            {
                cmaze[i][j] = 'G';
            }
            cout << cmaze[i][j];
            cout << " ";
        }
        cout << "\n";
    }
}



bool operator<(Node const& n1, Node const& n2)
{
    /*if(n1.priority_stamp!=n2.priority_stamp){
        return n1.priority_stamp > n2.priority_stamp;
    }*/
    if(abs(n1.f-n2.f)<0.000001){
        return n1.priority_stamp > n2.priority_stamp;
    }
    return n1.f > n2.f;
}



vector<PosNode> AStar(int start_pos[2], int goal_pos[2], int maze[mazeSizeX][mazeSizeY])
{
    int priority_count = 0;
    Node start, goal;
    start.GivePos(start_pos);
    goal.GivePos(goal_pos);
    priority_queue<Node> priority_q;
    vector<PosNode> fail;
    bool visited[mazeSizeX][mazeSizeY] = {0};
    if(maze[goal_pos[0]][goal_pos[1]] == 0)
    {
        cout << "goal unreachable\n";
        return fail;
    }
    if(start_pos[0]<0 || start_pos[0]>mazeSizeX-1 || start_pos[1]<0 || start_pos[1]>mazeSizeY-1 || goal_pos[0]<0 || goal_pos[0]>mazeSizeX-1 || goal_pos[1]<0 || goal_pos[1]>mazeSizeY-1)
    {
        cout << "invalid input\n";
        return fail;
    }
    start.move_cost = 0;
    start.g = 0;
    start.h = HeuristicFunctionDiagnal(start, goal);
    start.f = start.g + start.h;
    priority_count ++;
    start.priority_stamp = priority_count;
    priority_q.push(start);
    while(!priority_q.empty())
    {
        Node top_node = priority_q.top();

        PosNode pos;
        pos.pos[0] = top_node.PosX();
        pos.pos[1] = top_node.PosY();
        if(top_node.PosX() == goal.PosX() && top_node.PosY() == goal.PosY())
        {
            top_node.path.push_back(pos);
            cout << "GOAL FOUND!!!!!!!!" << "\n";
            return top_node.path;
        }
        visited[top_node.PosX()][top_node.PosY()] = true;
        priority_q.pop();
        vector<Node> successors = FindSuccessors(pos.pos, maze);
        for(unsigned int i = 0; i < successors.size(); i++)
        {
            Node succ = successors[i];
            succ.path = top_node.path;
            succ.g = succ.move_cost + top_node.g;
            if(!visited[succ.PosX()][succ.PosY()])
            {
                maze[succ.PosX()][succ.PosY()] = 4;
                succ.path.push_back(pos);
                succ.h = HeuristicFunctionDiagnal(succ, goal);
                succ.f = succ.g + succ.h;
                //cout << "pos: (" << succ.PosX() << "," << succ.PosY() << ")" << " f: " << succ.f <<"\n";
                priority_count ++;
                succ.priority_stamp = priority_count;
                priority_q.push(succ);
                visited[succ.PosX()][succ.PosY()] = true;

            }
        }
    }
    cout << "search fail\n";
    return fail;
}



vector<PosNode> bresenhams_line_alg(vector<PosNode> new_path, int maze[mazeSizeX][mazeSizeY])
{
    PosNode start = new_path.front();
    vector<PosNode> line_of_sight;
    vector<PosNode> output;
    //output.push_back(start);
    for(unsigned int i = 1; i < new_path.size(); i++)
    {
        PosNode goal = new_path[i];
        bool steep = false;
        bool walkable = true;
        if(abs(goal.pos[1]-start.pos[1])>abs(goal.pos[0]-start.pos[0]))
        {
            steep = true;
        }
        if(steep)
        {
            swap(start.pos[0], start.pos[1]);
            swap(goal.pos[0], goal.pos[1]);
        }
        if(start.pos[0]>goal.pos[0])
        {
            swap(start.pos[0], goal.pos[0]);
            swap(start.pos[1], goal.pos[1]);
        }
        int delta_x = goal.pos[0] - start.pos[0];
        int delta_y = abs(goal.pos[1] - start.pos[1]);
        int error = delta_x;
        int y_step;
        int y = start.pos[1];
        if(start.pos[1] < goal.pos[1])
        {
            y_step = 1;
        }
        else
        {
            y_step = -1;
        }
        for(int x=start.pos[0]; x<=goal.pos[0]; x++)
        {
            if(steep)
            {
                if(maze[y][x] == 0)
                {
                    walkable = false;
                }
            }
            else
            {
                if(maze[x][y] == 0)
                {
                    walkable = false;
                }
            }
            error -= 2*delta_y;
            if(error <= 0)
            {
                y += y_step;
                error += 2*delta_x;
            }
        }
        if(walkable == false)
        {
            output.push_back(new_path[i-1]);
            start = new_path[i-1];
        }
    }
    output.push_back(new_path.back());
    return output;
}

int get_x(vector<PosNode> path){
    PosNode a = path.front();
    int x = a.pos[0];
    return x;
}

int get_y(vector<PosNode> path){
    PosNode a = path.front();
    int y = a.pos[1];
    return y;
}



bool add(path_planning::path::Request  &req,
         path_planning::path::Response &res)
{
    
    int maze[mazeSizeX][mazeSizeY];
    build_maze(maze); 
    int obstacle_a[2] = {req.enemy1_x,req.enemy1_y};//<---from camera
    int obstacle_b[2] = {req.enemy2_x,req.enemy2_y};//<---from camera
    int obstacle_c[2] = {req.ally_x,req.ally_y};//<---from camera 
    build_obstacles(obstacle_a, robotSize, maze);
    build_obstacles(obstacle_b, robotSize, maze);
    build_obstacles(obstacle_c, robotSize, maze);
   
    int start_pos[2] = {req.my_pos_x,req.my_pos_y};//<---my_pos
    int goal_pos[2] = {req.goal_pos_x,req.goal_pos_y};//<---goap  


    vector<PosNode> a = AStar(start_pos, goal_pos, maze);
    vector<PosNode> b = bresenhams_line_alg(a, maze);//--->output b
   
    res.next_pos_x =  get_x(b) ;
    res.next_pos_y = get_y(b) ;

    return true;

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "path_planning_");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("add_two_ints_1", add);
    ros::spin();

    return 0;
}