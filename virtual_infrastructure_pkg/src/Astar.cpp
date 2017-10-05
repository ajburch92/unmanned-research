#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

const int n=128; // horizontal size of the grid  CAN I CHECK CONFIG FILE FOR THIS VALUE
const int m=96; // vertical size size of the grid
static int grid[n][m];
static int closed_nodes_grid[n][m]; // grid of closed (tried-out) nodes
static int open_nodes_grid[n][m]; // grid of open (not-yet-tried) nodes
static int dir_grid[n][m]; // grid of directions
const int dir=8; // number of possible directions to go at any position
int counter=0;
// if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};
int scale_factor = 2;
const string windowName = "Astar planner";



// start and finish locations
int xA, yA, xB, yB;

Mat gridDown;
Mat occupancyGrid;

class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        node(int xp, int yp, int d, int p) 
            {xPos=xp; yPos=yp; level=d; priority=p;}
    
        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority=level+estimate(xDest, yDest)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i) // i: direction
        {
             level+=(dir==8?(i%2==0?10:14):10);
        }
        
        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;         

            // Euclidian Distance
            d=static_cast<int>(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            //d=abs(xd)+abs(yd);
            
            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart, 
                 const int & xFinish, const int & yFinish )
{
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the node grids
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_grid[x][y]=0;
            open_nodes_grid[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_grid[x][y]=n0->getPriority(); // mark it on the open nodes grid

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_grid[x][y]=0;
        // mark it on the closed nodes grid
        closed_nodes_grid[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_grid[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();           
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || grid[xdx][ydy]==1 
                || closed_nodes_grid[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(), 
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_grid[xdx][ydy]==0)
                {
                    open_nodes_grid[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_grid[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_grid[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_grid[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_grid[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx && 
                           pq[pqi].top().getyPos()==ydy))
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pq[pqi].pop(); // remove the wanted node
                    
                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

void upsampleGrid () {
	pyrUp( gridDown, occupancyGrid, Size( gridDown.cols*scale_factor, gridDown.rows*scale_factor) );
}

void drawPath()
{

}

void vehicleCallback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
{
    xA  = vehicle_pose_msg->x;
    yA = vehicle_pose_msg->y;
    ROS_INFO("vehicleCallback: ( %i , %i )",xA,yA);
    clock_t start = clock();
    string route=pathFind(xA, yA, xB, yB);

    if(route=="") ROS_INFO("An empty route generated!");

    // get cpu time
    clock_t end = clock();
    double time_elapsed = double(end - start);

    ROS_INFO("Time to calculate the route (ms): %f" , time_elapsed);
    //ROS_INFO("Route: %s ", route);

    // follow the route on the grid and display it 
    if(route.length()>0)
    {
        int j; char c;
        int x=xA;
        int y=yA;
        grid[x][y]=2;
        for(int i=0;i<route.length();i++)
        {
            c =route.at(i);
            j=atoi(&c); 
            x=x+dx[j];
            y=y+dy[j];
            grid[x][y]=3;
        }
        grid[x][y]=4;
    
        // display the grid with the route
        //write to output image topic
        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++)
            {
                if(grid[x][y]==0)
                {
                    //cout<<".";
                }
                else if(grid[x][y]==1)
                {
                    //cout<<"O"; //obstacle
                }
                else if(grid[x][y]==2)
                {
                    //cout<<"S"; //start
                }
                else if(grid[x][y]==3)
                {
                    //cout<<"R"; //route
                }
                else if(grid[x][y]==4)
                {
                    //cout<<"F"; //finish
                }
            //cout<<endl;
            }
        }
    }

    // convert route to real world coordinates
    // draw route on output frame
    // publish waypoints to low level controller 

}

void goalCallback (const geometry_msgs::Pose2D::ConstPtr& goal_pose_msg) 
{
    xB  = goal_pose_msg->x;
    yB = goal_pose_msg ->y;

    //translate to downsampled coordinates
    xB = xB / scale_factor;
    yB = yB / scale_factor;

    //draw points on image
    circle(gridDown, Point(xB,yB), 9, Scalar(255,0,0),3);


    ROS_INFO("goalCallback: ( %i , %i )",xB,yB);
}

void occupancyGridCallback (const sensor_msgs::ImageConstPtr& msg) 
{
    // fillout the grid matrix with a '+' pattern
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg;
	sensor_msgs::Image occupancyGrid_msg;
	cv_bridge::CvImagePtr occupancyGrid_ptr;
	
	try
	{
		occupancyGrid_ptr  = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_INFO("cv_bridge exception: %s", e.what());
		return;
	}


    // convert to grid size
    // downsample grid to size
    // fill matrix  with object positions


    std_msgs::Header header; //empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    gridDown = occupancyGrid_ptr -> image;

    for(int x=n/8;x<n*7/8;x++)
    {
        grid[x][m/2]=1;
    }
    for(int y=m/8;y<m*7/8;y++)
    {
        grid[n/2][y]=1;
    }

    imshow(windowName,gridDown);

    counter++;
}


int main(int argc, char **argv)
{
	
    srand(time(NULL));

    // create empty grid
    for(int y=0;y<m;y++)
    {
        for(int x=0;x<n;x++) grid[x][y]=0;
    }

    // init ROS node
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle node;
    // subscibe to vision outputs
    image_transport::ImageTransport it_astar(node);
	image_transport::Subscriber sub_occupancyGrid = it_astar.subscribe("/occupancyGrid",1, &occupancyGridCallback); // use image_rect
    ros::Subscriber sub_vehicle = node.subscribe("/vehicle_pose",20, &vehicleCallback);
    ros::Subscriber sub_goal = node.subscribe("/goal_pose",20, &goalCallback);
    // subscribe to occupancy grid with obstacles

    // subscribe to output frame for path visualization

    // publish topics for the output visualization and for the next waypoint for the low level controller.
    ros::Publisher target_angle_pub = node.advertise<std_msgs::Float64>("/target_angle",2);
    ros::Publisher target_speed_pub = node.advertise<std_msgs::Float64>("/target_speed",2);
    
    // random starting position generation
/*    switch(rand()%8)
    {
        case 0: xA=0;yA=0;xB=n-1;yB=m-1; break;
        case 1: xA=0;yA=m-1;xB=n-1;yB=0; break;
        case 2: xA=n/2-1;yA=m/2-1;xB=n/2+1;yB=m/2+1; break;
        case 3: xA=n/2-1;yA=m/2+1;xB=n/2+1;yB=m/2-1; break;
        case 4: xA=n/2-1;yA=0;xB=n/2+1;yB=m-1; break;
        case 5: xA=n/2+1;yA=m-1;xB=n/2-1;yB=0; break;
        case 6: xA=0;yA=m/2-1;xB=n-1;yB=m/2+1; break;
        case 7: xA=n-1;yA=m/2+1;xB=0;yB=m/2-1; break;
    }*/ 

    ros::spin();

    return(0);
}