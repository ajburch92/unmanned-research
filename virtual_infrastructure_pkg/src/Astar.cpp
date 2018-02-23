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
#include <std_msgs/Int8.h>
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
const int n=120*3;//160; // horizontal size of the grid  CAN I CHECK CONFIG FILE FOR THIS VALUE
const int m=160*3;//120; // vertical size size of the grid
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
image_transport::Publisher pub_worldOG;
Point vehicle_pose;

// start and finish locations
int xA = 0;
int yA = 0;
int xB = 110 / scale_factor;
int yB = 0;

int subgoal = 0;

Mat gridDownLocal(n/3,m/3,CV_8UC1,Scalar(0));
Mat occupancyGridLocal(n/3,m/3,CV_8UC1,Scalar(0));
Mat gridDownRemote(n/3,m/3,CV_8UC1,Scalar(0));
Mat occupancyGridRemote(n/3,m/3,CV_8UC1,Scalar(0));
Mat occupancyGridworld(n,m,CV_8UC1,Scalar(0));

double conv_facLocal;
double conv_facRemote;
double confidenceLocal;
double confidenceRemote;
int ID_num;
int other_num;

vector<Point2f> corners2_vec;
vector<Point2f> corners1_vec;
Point2f undistorted_pts[4];
Point2f corners1_pts[4];
Point2f corners2_pts[4];
Point2f corners2_pts_camcam[4];

int corner_tic = 0;
int board_w = 160;
int board_h = 120;
Mat H_camcam;
Mat H_camcam_inv;
Mat H_cambird;
Mat gridRemoteResized;
Mat worldMap, MapLocal, MapRemote, worldMapPrev;
int worldMap_tic = 0;
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


void vehicleCallback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
{
	double xtemp, ytemp;
    vehicle_pose.x = vehicle_pose_msg->x ;
    vehicle_pose.y = vehicle_pose_msg ->y ;
    //translate to downsampled coordinates
    xtemp = vehicle_pose.x / scale_factor + 160;
    ytemp = vehicle_pose.y / scale_factor + 120;

    Point2f vehicle_pose_temp;


    // transform coordinates
    if (ID_num > 1) { // ID = 2, HbirdHcamcamOG
                float x = H_camcam_inv.at<double>(0,0) * xtemp + H_camcam_inv.at<double>(0,1) * ytemp + H_camcam_inv.at<double>(0,2);
                float y = H_camcam_inv.at<double>(1,0) * xtemp + H_camcam_inv.at<double>(1,1) * ytemp + H_camcam_inv.at<double>(1,2);
                float w = H_camcam_inv.at<double>(2,0) * xtemp + H_camcam_inv.at<double>(2,1) * ytemp + H_camcam_inv.at<double>(2,2);

                vehicle_pose_temp=Point(x/w,y/w);


    } else { // ID_num = 1 , HbirdOG

                vehicle_pose_temp=Point(xtemp,ytemp);

    }

    xA  = (int)vehicle_pose_temp.x;
    yA = (int)vehicle_pose_temp.y;
    ROS_INFO("vehicleCallback (xA, yA): ( %i , %i )",xA,yA);
}

void confidenceLocalCallback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
{
    confidenceLocal = conv_fac_msg -> data;

}

void confidenceRemoteCallback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
{
    confidenceRemote = conv_fac_msg -> data;

}

void conv_facLocalCallback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
{
    conv_facLocal = conv_fac_msg -> data;

}

void conv_facRemoteCallback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
{
    conv_facRemote = conv_fac_msg -> data;

}

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
	xtemp = goal_out_pose_msg->poses[subgoal].position.x;
	ytemp = goal_out_pose_msg->poses[subgoal].position.y;
	xB  = (int)xtemp;
	yB = (int)ytemp;

    ROS_INFO("goal_outCallback (xB, yB): ( %i , %i ),  subgoal_distance: %f i = %i",xB,yB,euclidean_d,subgoal);

}

void resizeGrid(Mat grid) {
    gridRemoteResized = grid;
    Size downsizeHigh(gridRemoteResized.cols * scale_factor , gridRemoteResized.rows * scale_factor); // this should be 160x120 
    resize(grid,gridRemoteResized,downsizeHigh);
    //ROS_INFO("gridRemoteResized : %i x %i" , gridRemoteResized.cols, gridRemoteResized.rows);
}


void updateMap(Mat &temp, bool local) {
    Mat Maptemp(temp.rows*3,temp.cols*3,temp.type(),Scalar(0));

    occupancyGridworld.setTo(Scalar(0)); // path pub (world size)

    if (counter < 1) { //zero out
        Maptemp.copyTo(worldMap);
        worldMap.copyTo(worldMapPrev);
        worldMap.copyTo(MapLocal);
        worldMap.copyTo(MapRemote);
    }
    worldMap_tic++;;
    //else, for 2 count, add images from ID 1 and 2
    if (local > 0) { //local
        MapLocal.setTo(Scalar(0));
        temp.copyTo(MapLocal(Rect(temp.cols,temp.rows,temp.cols,temp.rows)));
        warpPerspective(MapLocal , MapLocal, H_camcam, MapLocal.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(255));      
        cout << "tranform performed" << endl; 
    } else { //remote
        MapRemote.setTo(Scalar(0));
        temp.copyTo(MapRemote(Rect(temp.cols,temp.rows,temp.cols,temp.rows)));
    }

    if (worldMap_tic >= 2) {
        worldMap_tic = 0;
        addWeighted(MapLocal,0.5,MapRemote,0.5,0.0,worldMap);
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage worldOG_bridge;
        sensor_msgs::Image worldOG_msg;
        worldOG_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::MONO8, worldMap);
        worldOG_bridge.toImageMsg(worldOG_msg);
        pub_worldOG.publish(worldOG_msg);

        for (int x = 1; x <= n; x++)
        {
            for (int y = 1; y<=m; y++)
            {   
                int pix = (int)worldMap.at<uchar>(x,y);
                if (pix> 0) // obstacle
                {    
                    grid[y][x] = 1;
                } else {
                    grid[y][x] = 0;
                }
            }
        }

        header.seq = counter; // user defined counter
        header.stamp = ros::Time::now(); // time
        cv_bridge::CvImage occupancy_bridge;
        sensor_msgs::Image occupancyGrid_msg;

        // create waypoint message
        geometry_msgs::PoseArray poseArray;
        poseArray.poses.clear();
        poseArray.header.stamp=ros::Time::now();
        geometry_msgs::Pose wp_pose_msg;
        ROS_INFO("start(%i, %i) - end (%i, %i)", xA, yA, xB, yB);
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
                wp_pose_msg.position.x = double(x);
                wp_pose_msg.position.y = double(y); // change to doubles
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
                        occupancyGridworld.at<uchar>(Point(x,y)) = 255;
                    }
                    else if(grid[x][y]==2)
                    {
                        //cout<<"S"; //start
                        circle(occupancyGridworld, Point(x,y), 2, 125,2);

                    }
                    else if(grid[x][y]==3)
                    {
                        occupancyGridworld.at<uchar>(Point(x,y)) = 150;
                        //circle(occupancyGrid, Point(x,y), 1,255,1);
                    }
                    else if(grid[x][y]==4)
                    {
                        //cout<<"F"; //finish
                        circle(occupancyGridworld, Point(x,y), 2,175,2);
                    }
                //cout<<endl;
                }
            }
        } else { // just transfer objects 
            for(int y=0;y<m;y++)
            {
                for(int x=0;x<n;x++)
                {
                    if(grid[x][y]==1)
                    {
                        //cout<<"O"; //obstacle
                        occupancyGridworld.at<uchar>(Point(x,y)) = 255;
                    }
                }
            }
        }

        ROS_INFO("poseArray published");
        wp_pub.publish(poseArray); 
        
        occupancy_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, occupancyGridworld);
        occupancy_bridge.toImageMsg(occupancyGrid_msg);
        path_pub.publish(occupancyGrid_msg);

        //empty map
        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++) grid[x][y]=0;
        }

    }
}

void occupancyGridLocalCallback (const sensor_msgs::ImageConstPtr& msg) 
{
	cv_bridge::CvImagePtr occupancyGridLocal_ptr;

	gridDownLocal.setTo(Scalar(0)); // just obstacles (world size)

	try
	{
		occupancyGridLocal_ptr  = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
		ROS_INFO("cv_bridge local msg read");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_INFO("cv_bridge exception: %s", e.what());
		return;
	}

    gridDownLocal = occupancyGridLocal_ptr -> image;

    updateMap(gridDownLocal, 1);

    counter++;
    ROS_INFO("finished Local");

}

void occupancyGridRemoteCallback (const sensor_msgs::ImageConstPtr& msg) 
{
    // fillout the grid matrix with a '+' pattern
	cv_bridge::CvImagePtr occupancyGridRemote_ptr;

	gridDownRemote.setTo(Scalar(0));
	
	try
	{
		occupancyGridRemote_ptr  = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
		ROS_INFO("cv_bridge remote msg read");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_INFO("cv_bridge exception: %s", e.what());
		return;
	}

    gridDownRemote = occupancyGridRemote_ptr -> image;

    resizeGrid(gridDownRemote);
        
    updateMap(gridRemoteResized, 0);
    
    ROS_INFO("finished Remote");
    
}


void getPerspectives() {
    H_camcam = getPerspectiveTransform(corners1_pts,corners2_pts);
    H_camcam_inv = H_camcam.inv(DECOMP_SVD);    
    ROS_INFO("camcam");
    cout << H_camcam << endl;
}


void corners1Callback (const geometry_msgs::PoseArray::ConstPtr& corners1_msg) 
{
    corners1_vec.clear();
    int size = corners1_msg->poses.size();
    for (int i=0;i<size;i++)
    {
        corners1_vec.push_back(Point2f(corners1_msg->poses[i].position.x,corners1_msg->poses[i].position.y));
        corners1_pts[i] = corners1_vec[i];    
    }
    
    ROS_INFO("corners1");
    cout << corners1_vec << endl;

    getPerspectives();
}

void corners2Callback (const geometry_msgs::PoseArray::ConstPtr& corners2_msg) 
{       
    corners2_vec.clear();
    int size = corners2_msg->poses.size();
    for (int i=0;i<size;i++)
    {
        corners2_vec.push_back(Point((int)corners2_msg->poses[i].position.x,(int)corners2_msg->poses[i].position.y));
        corners2_pts[i] = corners2_vec[i];    
    }

    ROS_INFO("corners2");
    cout << corners2_vec << endl;

    getPerspectives();

    for (int n=0;n<=3; n++) {

        float x = H_camcam_inv.at<double>(0,0) * corners2_vec[n].x + H_camcam_inv.at<double>(0,1) * corners2_vec[n].y + H_camcam_inv.at<double>(0,2);
        float y = H_camcam_inv.at<double>(1,0) * corners2_vec[n].x + H_camcam_inv.at<double>(1,1) * corners2_vec[n].y + H_camcam_inv.at<double>(1,2);
        float w = H_camcam_inv.at<double>(2,0) * corners2_vec[n].x + H_camcam_inv.at<double>(2,1) * corners2_vec[n].y + H_camcam_inv.at<double>(2,2);

        corners2_pts_camcam[n]=Point(x/w,y/w);
    }
}


int main(int argc, char **argv)
{

    // init ROS node
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle node;
    ros::NodeHandle nodep("~");

    nodep.param("ID_num",ID_num,0);
    //ID_num = 1;

	stringstream ss;
	ss << ID_num;
	string s;
	s = ss.str();

	if (ID_num == 1) {
		other_num = 2;
	} else {
		other_num = 1;
	}
	stringstream other_ss;
	other_ss << other_num;
	string other_s;
	other_s = other_ss.str();

	string conv_facLocal = "/conv_fac" + s ;
    string conv_facRemote = "/conv_fac" + other_s;
	string occupancyGridLocal = "/occupancyGridHigh" + s ;
	string occupancyGridRemote = "/occupancyGridLow" + other_s ;
	string vehicle_pose_s = "/vehicle_pose" + s ;
	string confidenceLocal = "/confidence" + s ;
	string confidenceRemote = "/confidence" + other_s;
	string pathGrid = "/pathGrid"  + s;
	string wp_pose = "/wp_pose" + s ;
    string worldOG_string = "/occupancyGridWorld" + s;


    srand(time(NULL));

    // create empty grid
    for(int y=0;y<m;y++)
    {
        for(int x=0;x<n;x++) grid[x][y]=0;
    }

	vehicle_pose.x = 0;
	vehicle_pose.y = 0;

    undistorted_pts[0].x=0;
    undistorted_pts[1].x=board_w-1;
    undistorted_pts[2].x=0;
    undistorted_pts[3].x=board_w-1;
    undistorted_pts[0].y=0;
    undistorted_pts[1].y=0;
    undistorted_pts[2].y=board_h-1;
    undistorted_pts[3].y=board_h-1;

    H_camcam = getPerspectiveTransform(undistorted_pts, undistorted_pts);
    H_cambird = getPerspectiveTransform(undistorted_pts, undistorted_pts);
    getPerspectives();

    // subscibe to vision outputs
    image_transport::ImageTransport it_astar(node);
	image_transport::Subscriber sub_occupancyGridLocal = it_astar.subscribe(occupancyGridLocal,1, &occupancyGridLocalCallback); 
	image_transport::Subscriber sub_occupancyGridRemote = it_astar.subscribe(occupancyGridRemote,1, &occupancyGridRemoteCallback); 
    ros::Subscriber sub_corners1 = node.subscribe("corners1",1,&corners1Callback);
    ros::Subscriber sub_corners2 = node.subscribe("corners2",1,&corners2Callback);
    ros::Subscriber sub_vehicle = node.subscribe(vehicle_pose_s,2, &vehicleCallback);
    ros::Subscriber sub_goal_out = node.subscribe("goal_pose_out",2, &goal_outCallback);
    ros::Subscriber sub_conv_facLocal = node.subscribe(conv_facLocal,2, &conv_facLocalCallback);
    ros::Subscriber sub_conv_facRemote = node.subscribe(conv_facRemote,2, &conv_facRemoteCallback);
    ros::Subscriber sub_confidenceLocal = node.subscribe(confidenceLocal,2, &confidenceLocalCallback);
    ros::Subscriber sub_confidenceRemote = node.subscribe(confidenceLocal,2, &confidenceRemoteCallback);

    // publish topics for the output visualization and for the next waypoint for the low level controller.
    path_pub = it_astar.advertise(pathGrid,1);
	wp_pub = node.advertise<geometry_msgs::PoseArray>(wp_pose,2);
    pub_worldOG = it_astar.advertise(worldOG_string,1);

    
    ROS_INFO("Start path planning operations");

//    ros::spin();
    ros::Rate r(4);
    while(ros::ok()) {
        ros::spinOnce();
        updateMap(gridDownLocal,1);
        updateMap(gridDownRemote,0);
        r.sleep();
    }
    
    return(0);

}