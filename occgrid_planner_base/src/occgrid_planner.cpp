
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <list>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <occgrid_planner_base/Signal.h>

#include <std_msgs/Bool.h>

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800

class OccupancyGridPlanner {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber og_sub_;
        ros::Subscriber target_sub_;
        ros::Publisher path_pub_;
        ros::Subscriber signal_sub_;

        ros::Publisher target_pub;
        ros::Subscriber new_explore_pub_;

        tf::TransformListener listener_;

        cv::Rect roi_;
        cv::Mat_<uint8_t> og_, cropped_og_;
        cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_;
        cv::Point3i og_center_;
        nav_msgs::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        unsigned int neighbourhood_;
        bool ready;
        bool debug;
        float rob_rad = 0.2;

        ros::Timer timer;

        bool timer_flag = true;

        int signal_range = 10;
        std::map<double, std::pair<int,int>> frontier;
        geometry_msgs::PoseStamped goal;
        std::map<double, std::pair<int,int>>::iterator listIndex = frontier.begin();

        typedef std::multimap<float, cv::Point3i> Heap;

        void metal_callback(const occgrid_planner_base::SignalPtr &msg){
            static cv::Mat_<float> moy_ = cv::Mat_<float>(og_.rows, og_.cols,NAN);
            static cv::Mat_<int> iter_ = cv::Mat_<int>(og_.rows, og_.cols,0);
            static cv::Mat_<uint8_t> im_moy_ = cv::Mat_<int>(og_.rows, og_.cols,0x00);
            static cv::Mat_<cv::Vec3b> im_moy_rgb_;
            cv::Point3i curr_pose;
            tf::StampedTransform transform;
            listener_.waitForTransform("map",base_link_,ros::Time::now(),ros::Duration(1.0));
            listener_.lookupTransform("map",base_link_, ros::Time(0), transform);

            curr_pose.x=transform.getOrigin().x() / info_.resolution + og_center_.x;
            curr_pose.y=transform.getOrigin().y() / info_.resolution + og_center_.y;
            curr_pose.z=(unsigned int) round(tf::getYaw(transform.getRotation())/M_PI*4)%8 + og_center_.z;

            for (int i = curr_pose.x-signal_range; i <= curr_pose.x+signal_range ; i++) {
                for (int j = curr_pose.y-signal_range; j <= curr_pose.y+signal_range ; j++) {
                    if (std::pow(i-curr_pose.x,2) + std::pow(j-curr_pose.y,2) <= std::pow(signal_range, 2)){
                        moy_(i,j) = ((isnan(moy_(i,j)) ?  0 : moy_(i,j))*iter_(i,j) + msg->data)/(++iter_(i,j));
                        im_moy_(i,j) = (uint8_t) (moy_(i,j)*255);
                    }
                }
            }

            im_moy_rgb_ = im_moy_.clone();
            cv::resize(im_moy_rgb_, im_moy_rgb_,cv::Size(im_moy_.rows, im_moy_.cols));
            for (int k = 1; k < im_moy_.rows; k++ ){
                for(int l = 1; l < im_moy_.cols; l++){
                    if(!isnan(moy_(k,l)) && (isnan(moy_(k-1,l)) || isnan(moy_(k+1,l)) || isnan(moy_(k,l-1)) || isnan(moy_(k,l+1))) && og_(l,k)) {  //&& (og_(l,k) || og_(l-1,l)|| og_(k+1,l) || og_(k,l-1) || og_(k,l+1))){
                        frontier[hypot(curr_pose.x-k,curr_pose.y-l)] = std::make_pair(k,l); 
                        im_moy_rgb_(k,l) = cv::Vec3b(255,0,255);
                    }
                }
            }


            if (timer_flag) {
                    ROS_INFO("new explore target");
                    while (abs(curr_pose.x - listIndex->second.first) <= (signal_range-2) && abs(curr_pose.y - listIndex->second.second) <= (signal_range-2)) {
                        std::advance(listIndex,1);
                    }
                    goal.header.frame_id = frame_id_; 
                    goal.pose.position.x = (listIndex->second.first - og_center_.x) * info_.resolution; 
                    goal.pose.position.y = (listIndex->second.second - og_center_.y)* info_.resolution; 
                    goal.pose.orientation.w = 1;
                    std::cout << "Position   " << curr_pose.x << "    " << curr_pose.y << std::endl;
                    std::cout << "Target   " << listIndex->second.first << "    " << listIndex->second.second << std::endl;
                    target_pub.publish(goal);
                    timer_flag = !timer_flag;
            }
            
            cv::imshow( "metal_score", im_moy_rgb_);
        }

        void timer_callback(const ros::TimerEvent& event) {
            if (!timer_flag) {
                timer_flag = !timer_flag;
            }
        }

        // Callback for Occupancy Grids
        void og_callback(const nav_msgs::OccupancyGridConstPtr & msg) {
            info_ = msg->info;
            frame_id_ = msg->header.frame_id;
            // Create an image to store the value of the grid.
            og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_center_ = cv::Point3i(-info_.origin.position.x/info_.resolution, -info_.origin.position.y/info_.resolution, 0);

            // Some variables to select the useful bounding box 
            unsigned int maxx=0, minx=msg->info.width, 
                         maxy=0, miny=msg->info.height;
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    int8_t v = msg->data[j*msg->info.width + i];
                    switch (v) {
                        case 0: 
                            og_(j,i) = FREE; 
                            break;
                        case 100: 
                            og_(j,i) = OCCUPIED; 
                            break;
                        case -1: 
                        default:
                            og_(j,i) = UNKNOWN; 
                            break;
                    }
                    // Update the bounding box of free or occupied cells.
                    
                    if (og_(j,i) != UNKNOWN) {
                        minx = std::min(minx,i);
                        miny = std::min(miny,j);
                        maxx = std::max(maxx,i);
                        maxy = std::max(maxy,j);
                    }
                }
            }

            float dilatation_size = ceil(rob_rad/info_.resolution);
            cv::erode(og_, og_, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*dilatation_size+3, 2*dilatation_size+3), cv::Point(dilatation_size, dilatation_size)));

            if (!ready) {
                ready = true;
                ROS_INFO("Received occupancy grid, ready to plan");
            }
            // The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            roi_ = cv::Rect(minx,miny,w,h);
            cv::cvtColor(og_, og_rgb_, cv::COLOR_GRAY2RGB);
            // Compute a sub-image that covers only the useful part of the
            // grid.
            cropped_og_ = cv::Mat_<uint8_t>(og_,roi_);
            if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                // The occupancy grid is too large to display. We need to scale
                // it first.
                double ratio = w / ((double)h);
                cv::Size new_size;
                if (ratio >= 1) {
                    new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                } else {
                    new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                }
                cv::Mat_<uint8_t> resized_og;
                cv::resize(cropped_og_,resized_og,new_size);
                cv::imshow( "OccGrid", resized_og );
            } else {
                // cv::imshow( "OccGrid", cropped_og_ );
                cv::imshow( "OccGrid", og_rgb_ );
            }
        }

        // Generic test if a point is within the occupancy grid
        bool isInGrid(const cv::Point3i & P) {
            if ((P.x < 0) || (P.x >= (signed)info_.width) 
                    || (P.y < 0) || (P.y >= (signed)info_.height)) {
                return false;
            }
            return true;
        }

        // This is called when a new goal is posted by RViz. We don't use a
        // mutex here, because it can only be called in spinOnce.
        void target_callback(const geometry_msgs::PoseStampedConstPtr & msg) {

            if (msg->header.frame_id != "") {

                tf::StampedTransform transform;
                geometry_msgs::PoseStamped pose;
                if (!ready) {
                    ROS_WARN("Ignoring target while the occupancy grid has not been received");
                    return;
                }
                ROS_INFO("Received planning request");
                og_rgb_marked_ = og_rgb_.clone();
                // Convert the destination point in the occupancy grid frame. 
                // The debug case is useful is the map is published without
                // gmapping running (for instance with map_server).
                if (debug) {
                    pose = *msg;
                } else {
                    // This converts target in the grid frame.
                    listener_.waitForTransform(frame_id_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                    listener_.transformPose(frame_id_,*msg, pose);
                    // this gets the current pose in transform
                    listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
                }
                // Now scale the target to the grid resolution and shift it to the
                // grid center.
                int target_yaw = (unsigned int) round(tf::getYaw(pose.pose.orientation)/M_PI*4)%8;
                cv::Point3i target = cv::Point3i(pose.pose.position.x / info_.resolution, pose.pose.position.y / info_.resolution, target_yaw) + og_center_;
                ROS_INFO("Planning target: %.2f %.2f %.2f-> %d %d %d",
                            pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation), target.x, target.y, target.z);
                cv::circle(og_rgb_marked_, cv::Point(target.x, target.y), 10, cv::Scalar(0,0,255));
                cv::imshow( "OccGrid", og_rgb_marked_ );
                if (!isInGrid(target)) {
                    ROS_ERROR("Invalid target point (%.2f %.2f %.2f) -> (%d %d %d)",
                            pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation), target.x, target.y, target.z);
                    std::advance(listIndex, 1);
                    return;
                }
                // Only accept target which are FREE in the grid (HW, Step 5).
                if (og_(cv::Point(target.x, target.y)) == OCCUPIED) {
                    ROS_ERROR("Invalid target point: occupancy = %d",og_(cv::Point(target.x, target.y)));
                    std::advance(listIndex, 1);
                    return;
                }

                listIndex = frontier.begin();
                // Now get the current point in grid coordinates.
                cv::Point3i start;
                if (debug) {
                    start = og_center_;
                } else {
                    int start_yaw = (unsigned int) round(tf::getYaw(transform.getRotation())/M_PI*4)%8;
                    start = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution, start_yaw)
                        + og_center_;
                }
                ROS_INFO("Planning origin %.2f %.2f %.2f -> %d %d %d",
                        transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()), start.x, start.y, start.z);
                cv::circle(og_rgb_marked_, cv::Point(start.x, start.y), 10, cv::Scalar(0,255,0));
                cv::imshow( "OccGrid", og_rgb_marked_ );
                if (!isInGrid(start)) {
                    ROS_ERROR("Invalid starting point (%.2f %.2f %.2f) -> (%d %d %d)",
                            transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()), start.x, start.y, start.z);
                    return;
                }
                // If the starting point is not FREE there is a bug somewhere, but
                // better to check
                if (og_(cv::Point(start.x, start.y)) == OCCUPIED) {
                    og_(cv::Point(start.x, start.y)) = FREE;
                    ROS_ERROR("Invalid start point: occupancy = %d",og_(cv::Point(start.x, start.y)));
                    return;
                }
                ROS_INFO("Starting planning from (%d, %d, %d) to (%d, %d, %d)", start.x, start.y, start.z, target.x, target.y, target.z);
                // Here the Dijskstra algorithm starts 
                // The best distance to the goal computed so far. This is
                // initialised with Not-A-Number. 
                int dim[3] = {og_.size().width, og_.size().height, 8};
                cv::Mat_<float> cell_value(3, dim, NAN);
                // For each cell we need to store a pointer to the coordinates of
                // its best predecessor. 
                cv::Mat_<cv::Vec3s> predecessor(3,dim);
                // The neighbour of a given cell in relative coordinates. The order
                // is important. If we use 4-connexity, then we can use only the
                // first 4 values of the array. If we use 8-connexity we use the
                // full array.
                std::vector<std::vector<cv::Point3i>> neighbours =
                    {{cv::Point3i( 1, 0,0), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                     {cv::Point3i( 1, 1,0), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                     {cv::Point3i( 0, 1,0), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                     {cv::Point3i(-1, 1,0), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                     {cv::Point3i(-1, 0,0), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                     {cv::Point3i(-1,-1,0), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                     {cv::Point3i( 0,-1,0), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                     {cv::Point3i( 1,-1,0), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)}};
                // Cost of displacement corresponding the neighbours. Diagonal
                // moves are 44% longer.*/
                std::vector<std::vector<float>> cost = {{1, 10, 10}, {sqrt(2), 10, 10}};

                // The core of Dijkstra's Algorithm, a sorted heap, where the first
                // element is always the closer to the start.

                Heap heap;
                heap.insert(Heap::value_type(hypot(target.x - start.x, target.y - start.y), start));
                cell_value(start.x,start.y,start.z) = 0;
                while (!heap.empty() && !(heap.begin()->second==target) ) {
                    // Select the cell at the top of the heap
                    Heap::iterator hit = heap.begin();
                    // the cell it contains is this_cell
                    cv::Point3i this_cell = hit->second;
                    // if (this_cell == target)
                    //     break;
                    // and its score is this_cost
                    float this_cost = hit->first; //cell_value(this_cell.x,this_cell.y,this_cell.z);
                    // We can remove it from the heap now.
                    heap.erase(hit);
                    // Now see where we can go from this_cell
                    for (unsigned int i=0;i<neighbourhood_;i++) {
                        cv::Point3i dest = this_cell + neighbours[this_cell.z][i];
                        dest.z = ((dest.z) + 8) % 8;
                        // dest.z = dest.z + ((i > 0) ? pow(-1,i) : i);
                        if (!isInGrid(dest)) {
                            // outside the grid
                            continue;
                        }
                        uint8_t og = og_(cv::Point(dest.x, dest.y));
                        if (og == OCCUPIED) {
                            // occupied or unknown
                            continue;
                        }
                        float cv = cell_value(dest.x, dest.y, dest.z);
                        float new_cost = this_cost + cost[i%2][i];
                        if (std::isnan(cv) || (new_cost < cv)) {
                            // found shortest path (or new path), updating the
                            // predecessor and the value of the cell
                            predecessor.at<cv::Vec3s>(dest.x,dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y,this_cell.z);
                            cell_value(dest.x, dest.y, dest.z) = new_cost;
                            float heuristic = hypot(target.x - dest.x, target.y - dest.y);
                            // And insert the selected cells in the map.
                            heap.insert(Heap::value_type(new_cost + heuristic, dest));
                        }
                    }
                }

                if (std::isnan(cell_value(target.x, target.y, target.z))) {
                    // No path found
                    ROS_ERROR("No path found from (%d, %d, %d) to (%d, %d, %d)",start.x,start.y,start.z,target.x,target.y,target.z);
                    return;
                }
                ROS_INFO("Planning completed");
                // Now extract the path by starting from goal and going through the
                // predecessors until the starting point
                std::list<cv::Point3i> lpath;
                while (target != start) {
                    assert(lpath.size()<1000000);
                    lpath.push_front(target);
                    cv::Vec3s p = predecessor(target.x, target.y, target.z);
                    target.x = p[0]; target.y = p[1]; target.z = p[2];
                }
                lpath.push_front(start);
                // Finally create a ROS path message
                nav_msgs::Path path;
                path.header.stamp = ros::Time::now();
                path.header.frame_id = frame_id_;
                path.poses.resize(lpath.size());
                std::list<cv::Point3i>::const_iterator it = lpath.begin();
                unsigned int ipose = 0;
                while (it != lpath.end()) {
                    // time stamp is not updated because we're not creating a
                    // trajectory at this stage
                    path.poses[ipose].header = path.header;
                    cv::Point3i P = *it - og_center_;
                    path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
                    path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
                    tf::Quaternion Q = tf::createQuaternionFromRPY(0, 0, P.z*M_PI/4);
                    tf::quaternionTFToMsg(Q, path.poses[ipose].pose.orientation);
                    ipose++;
                    it ++;
                }
                path_pub_.publish(path);
                ROS_INFO("Request completed");
            }
        }
        
    public:
        OccupancyGridPlanner() : nh_("~") {
            int nbour = 4;
            bool toggle_explore = true;
            ready = false;
            nh_.param("base_frame",base_link_,std::string("/body"));
            nh_.param("debug",debug,false);
            nh_.param("neighbourhood",nbour,nbour);
            nh_.param("toggle_explore",toggle_explore,toggle_explore);
            switch (nbour) {
                case 4: neighbourhood_ = nbour; break;
                case 8: neighbourhood_ = nbour; break;
                default: 
                    ROS_WARN("Invalid neighbourhood specification (%d instead of 4 or 8)",nbour);
                    neighbourhood_ = 3;
            }
            
            timer = nh_.createTimer(ros::Duration(2), &OccupancyGridPlanner::timer_callback, this);
            timer.start();

            og_sub_ = nh_.subscribe("occ_grid",1,&OccupancyGridPlanner::og_callback,this);
            target_sub_ = nh_.subscribe("goal",1,&OccupancyGridPlanner::target_callback,this);
            path_pub_ = nh_.advertise<nav_msgs::Path>("path",1,true);

            target_pub = nh_.advertise<geometry_msgs::PoseStamped>("goal",1);
            signal_sub_ = nh_.subscribe("/vrep/signal",1,&OccupancyGridPlanner::metal_callback,this);
        }
};

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"occgrid_planner");
    OccupancyGridPlanner ogp;
    cv::namedWindow( "OccGrid", cv::WINDOW_AUTOSIZE );
    while (ros::ok()) {
        ros::spinOnce();
        if (cv::waitKey( 50 )== 'q') {
            ros::shutdown();
        }
    }
}

