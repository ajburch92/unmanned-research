#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#define LOS_RADIUS 100 //currently in pixels

using namespace std;
using namespace cv;

//m and n should start as camera resolution size
int scale_factor = 8; // change this to a launch file paramerter. this is the downsampling applied to the occupancy grid.
//res initially 1288x964 then resize to 1280x960, reduced to 160x120 (8x)
const int n=160; // horizontal size of the grid  CAN I CHECK CONFIG FILE FOR THIS VALUE
const int m=120; // vertical size size of the grid
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
const string astarwindowName = "Astar planner";
image_transport::Publisher path_pub;
ros::Publisher wp_pub;

Point vehicle_pose;

// start and finish locations
int xA = 0;
int yA = 0;
int xB = 110 / scale_factor;
int yB = 0;

int subgoal = 0;

Mat gridDown(n,m,CV_8UC1,Scalar(0));
Mat pathDown(n,m,CV_8UC1,Scalar(0));
Mat occupancyGrid(n,m,CV_8UC1,Scalar(0));

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


void vehicleCallback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
{
	double xtemp, ytemp;
    vehicle_pose.x = vehicle_pose_msg->x;
    vehicle_pose.y = vehicle_pose_msg ->y;
    //translate to downsampled coordinates
    xtemp = vehicle_pose.x / scale_factor;
    ytemp = vehicle_pose.y / scale_factor;
    xA  = (int)xtemp;
    yA = (int)ytemp;

    ROS_INFO("vehicleCallback (xA, yA): ( %i , %i )",xA,yA);
}

/*void goalCallback (const geometry_msgs::Pose2D::ConstPtr& goal_pose_msg) 
{
    double xtemp, ytemp;
    xtemp = goal_pose_msg->x;
    ytemp = goal_pose_msg ->y;
    //translate to downsampled coordinates
    xtemp = xtemp / scale_factor;
    ytemp = ytemp / scale_factor;
    //xB  = (int)xtemp;
    //yB = (int)ytemp;

    ROS_INFO("goalCallback: ( %i , %i )",xB,yB);
}*/

void goal_outCallback (const geometry_msgs::PoseArray::ConstPtr& goal_out_pose_msg) 
{
    double xtemp, ytemp;
    int sz = goal_out_pose_msg->poses.size();
    double euclidean_d=0;

/*    if ( goal_out_pose_msg->poses[0].position.x == 0 && goal_out_pose_msg->poses[0].position.x == 0 ) //no subgoals or cleared subgoals ; reset subgoal indice
    {
    	subgoal = 0;
    } */

    while (1)
    {
      euclidean_d = sqrt(((((vehicle_pose.x)-goal_out_pose_msg->poses[subgoal].position.x)*((vehicle_pose.x)-goal_out_pose_msg->poses[subgoal].position.x)) + (((vehicle_pose.y)-goal_out_pose_msg->poses[subgoal].position.y)*((vehicle_pose.y)-goal_out_pose_msg->poses[subgoal].position.y))));
      if (euclidean_d > LOS_RADIUS) // continue to waypoint
      {
	    break;
      } else { //distance < los radius : either go to next waypoint or stop at last waypoint
      	if (subgoal==(sz-1)) // last waypoint, stop
      	{
      		subgoal = 0;
      		//break;
      	} else { // subgoal, go to next waypoint
      		subgoal++;
      	}
      }
    }
	xtemp = goal_out_pose_msg->poses[subgoal].position.x / scale_factor;
	ytemp = goal_out_pose_msg->poses[subgoal].position.y / scale_factor;
	xB  = (int)xtemp;
	yB = (int)ytemp;

    ROS_INFO("goal_outCallback (xB, yB): ( %i , %i ),  subgoal_distance: %f i = %i",xB,yB,euclidean_d,subgoal);



}

void occupancyGridCallback (const sensor_msgs::ImageConstPtr& msg) 
{
    // fillout the grid matrix with a '+' pattern
	cv_bridge::CvImagePtr occupancyGrid_ptr;

	gridDown.setTo(Scalar(0));
	occupancyGrid.setTo(Scalar(0));
	gridDown.setTo(Scalar(0));
	
	try
	{
		occupancyGrid_ptr  = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
		ROS_INFO("cv_bridge msg read");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_INFO("cv_bridge exception: %s", e.what());
		return;
	}

    gridDown = occupancyGrid_ptr -> image;

    // convert to grid size - ALREADY DOWN SAMPLED, JUST NEED TO INCREASE RESOLUTION UPON PUBLISHING
    //memcpy(gridDown.data, grid,n*m*sizeof(int)
    // fill matrix grid same size with object positions
    for (int x = 1; x <= n; x++)
    {
		for (int y = 1; y<=m; y++)
		{
			int pix = (int)gridDown.at<uchar>(y,x);
			if (pix > 0) // obstacle
			{
				grid[x][y] = 1;
			} else {
				grid[x][y] = 0;
			}
		}
	}


	std_msgs::Header header; //empty header
	header.seq = counter; // user defined counter
	header.stamp = ros::Time::now(); // time
    cv_bridge::CvImage occupancy_bridge;
	sensor_msgs::Image occupancyGrid_msg;

	// create waypoint message
	geometry_msgs::PoseArray poseArray;
    poseArray.poses.clear();
    poseArray.header.stamp=ros::Time::now();
	geometry_msgs::Pose wp_pose_msg;

    clock_t start = clock();
    string route=pathFind(xA, yA, xB, yB);
    if(route=="") ROS_INFO("An empty route generated!");
    // get cpu time
    clock_t end = clock();
    double time_elapsed = double(end - start);
    ROS_INFO("Time to calculate the route (ms): %f" , time_elapsed);

    // follow the route on the grid and display it 
    int j; char c;
	int x=xA;
	int y=yA;
	int x_prev;
	int y_prev;
    if(route.length()>0)
    {

        grid[x][y]=2;
        for(int i=0;i<route.length();i++)
        {
            c =route.at(i);
            j=atoi(&c); 
            x=x+dx[j];
            y=y+dy[j];
            grid[x][y]=3;

            // fill wp vector
            wp_pose_msg.position.x = double(x*scale_factor);
            wp_pose_msg.position.y = double(y*scale_factor); // change to doubles
            poseArray.poses.push_back(wp_pose_msg); 

            // pushback if so many steps have passed? 
            //could hard code coarser path here, otherwise look at resolution factor tree, or crop/resize

            x_prev = x;
            y_prev = y;
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
                    //cout<<"."; // open space, keep black
                }
                else if(grid[x][y]==1)
                {
                    //cout<<"O"; //obstacle
                    occupancyGrid.at<uchar>(Point(x,y)) = 255;
                }
                else if(grid[x][y]==2)
                {
                    //cout<<"S"; //start
                    circle(occupancyGrid, Point(x,y), 2, 125,2);

                }
                else if(grid[x][y]==3)
                {
                    occupancyGrid.at<uchar>(Point(x,y)) = 150;
                    //circle(occupancyGrid, Point(x,y), 1,255,1);
                }
                else if(grid[x][y]==4)
                {
                    //cout<<"F"; //finish
                    circle(occupancyGrid, Point(x,y), 2,175,2);
                }
            //cout<<endl;
            }
        }
    }

    ROS_INFO("poseArray published");
	wp_pub.publish(poseArray); 
    
    occupancy_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, occupancyGrid);
    occupancy_bridge.toImageMsg(occupancyGrid_msg);
    path_pub.publish(occupancyGrid_msg);

    //empty map
    for(int y=0;y<m;y++)
	{
    	for(int x=0;x<n;x++) grid[x][y]=0;
	}
	//empty mats
	
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

	vehicle_pose.x = 0;
	vehicle_pose.y = 0;

    // init ROS node
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle node;
    // subscibe to vision outputs
    image_transport::ImageTransport it_astar(node);
	image_transport::Subscriber sub_occupancyGrid = it_astar.subscribe("/occupancyGrid",1, &occupancyGridCallback); // use image_rect
    ros::Subscriber sub_vehicle = node.subscribe("/vehicle_pose",2, &vehicleCallback);
    //ros::Subscriber sub_goal = node.subscribe("/goal_pose",2, &goalCallback);
    ros::Subscriber sub_goal_out = node.subscribe("/goal_pose_out",2, &goal_outCallback);

    // publish topics for the output visualization and for the next waypoint for the low level controller.
    path_pub = it_astar.advertise("/pathGrid",1);
	wp_pub = node.advertise<geometry_msgs::PoseArray>("wp_pose",2);
    
    ROS_INFO("Start path planning operations");

    ros::spin();
    
    return(0);

}