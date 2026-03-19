#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <queue>
#include <map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>

enum Direction {
    FRONT = 0, 
    RIGHT = 1, 
    BACK = 2,  
    LEFT = 3   
};

const double PI = 3.14159265358979323846;
geometry_msgs::Pose2D current_pose;
geometry_msgs::Pose2D currentJunction_pose;
geometry_msgs::Pose2D lastJunction_pose;
int front_wall = 0;
int right_wall = 1;
int left_wall = 1;
int finish = 0;
bool atJunction = false;
const float max_dist = 0.8;
int rot_time = 0;
int current_junction = 0;
int current_direction = 0;

std::queue<int> camino;

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_pose.theta = yaw;
}

// Detect front wall
void detect_front_wall(const sensor_msgs::LaserScan::ConstPtr& msg) {
  front_wall= 0;
  for (int i = 350; i < 359; i++){
    if(msg->ranges[i] < 0.25 && msg->ranges[i] > 0.02){
      front_wall = 1;
      return;
    }
  }
  for (int i = 1; i < 9; i++){
    if(msg->ranges[i] < 0.25 && msg->ranges[i] > 0.02){
      front_wall = 1;
      return;
    }
  }
}

// Detect right wall
void detect_right_wall(const sensor_msgs::LaserScan::ConstPtr& msg) {
  right_wall= 0;
  for (int i = 260; i < 280; i++){
    if(msg->ranges[i] < 0.28 && msg->ranges[i] > 0.02){
      right_wall = 1;
      return;
    }
  }
}

// Detect left wall
void detect_left_wall(const sensor_msgs::LaserScan::ConstPtr& msg) {
  left_wall= 0;
  for (int i = 80; i < 100; i++){
    if(msg->ranges[i] < 0.28 && msg->ranges[i] > 0.02){
      left_wall = 1;
      return;
    }
  }
}

// Struct Junction: a junction is caracterized by
// - An identificator (id)
// - A position (pose)
// - Directions to go from here (max. 4) (edges)
struct Junction {
  int id;
  geometry_msgs::Pose2D pose;
  std::map<int,int> edges;
};

// Maze with which the maze is mapped
static std::map<int,Junction> graph;

// Function to print the information from the graph 
void printGraph() {
    ROS_INFO("Graph nodes at the end:");
    
    for (const auto& entry : graph) {
        const Junction& j = entry.second; 
        ROS_INFO("Junction ID: %i, Pose: (%.2f, %.2f)", 
                 j.id, j.pose.x, j.pose.y);

        if (!j.edges.empty()) {
            ROS_INFO("  Edges:");
            for (const auto& edge : j.edges) {
                Direction dir = static_cast<Direction>(edge.first); 
                ROS_INFO("    - Direction: %d, Connected to Junction ID: %d", dir, edge.second);
            }
        } else {
            ROS_INFO("  No edges connected.");
        }
    }
}

// Function to load the graph from the YAML file
bool loadMazeGraph(const std::string& filename) {
  try {
    YAML::Node root = YAML::LoadFile(filename);
    for (auto it = root.begin(); it != root.end(); ++it) {
      const YAML::Node& node = it->second;
      Junction junc;

      junc.id = node["id"].as<int>();

      const YAML::Node& p = node["pose"];
      junc.pose.x     = p["x"].as<double>();
      junc.pose.y     = p["y"].as<double>();
      junc.pose.theta = p["theta"].as<double>();

      const YAML::Node& e = node["edges"];
      for (auto eit = e.begin(); eit != e.end(); ++eit) {
        int edge_id    = eit->first.as<int>();
        int neighbor   = eit->second.as<int>();
        junc.edges[edge_id] = neighbor;
      }

      graph[junc.id] = junc;
    }
    printGraph();
    return true;
  }
  catch (const YAML::Exception& ex) {
    std::cerr << "Failed to parse YAML file: " << ex.what() << std::endl;
    return false;
  }
}

// BFS that finds the shortes path to go from node with id 0 to node with highest id
// and returns a queue with the series of directions that must be taken at each
// junction to reach the exit through the fastest route
std::queue<int> computeDirectionQueue() {
  if (graph.empty()) {
    return {};
  }

  int start_id = 0;
  int end_id = std::max_element(
    graph.begin(), graph.end(),
    [](auto &a, auto &b){ return a.first < b.first; }
  )->first;

  std::queue<int>    q;
  std::map<int,bool> visited;
  std::map<int, std::pair<int,int>> parent;

  q.push(start_id);
  visited[start_id] = true;

  bool found = false;
  while (!q.empty() && !found) {
    int u = q.front(); q.pop();

    for (auto &e : graph[u].edges) {
      int dir = e.first;
      int v   = e.second;
      if (!visited[v]) {
        visited[v] = true;
        parent[v]  = {u, dir};
        if (v == end_id) {
          found = true;
          break;
        }
        q.push(v);
      }
    }
  }

  if (!found) {
    std::cerr << "No path from " << start_id << " to " << end_id << "\n";
    return {};
  }

  std::vector<int> rev_dirs;
  for (int cur = end_id; cur != start_id; ) {
    auto p = parent[cur];
    rev_dirs.push_back(p.second);  
    cur = p.first;                
  }
  std::reverse(rev_dirs.begin(), rev_dirs.end());

  std::queue<int> dir_queue;
  for (int d : rev_dirs) {
    dir_queue.push(d);
  }
  return dir_queue;
}


// Detect a junction
// A junction occurs when the robot must turn or a choice must be made
void junction(const sensor_msgs::LaserScan::ConstPtr& msg) {

  atJunction = !right_wall || !left_wall || front_wall;
  if (atJunction) {
    currentJunction_pose = current_pose;
  }
  return;
}

// The robot turns 90 degrees to the right if right = True, 90 degrees to the left otherwise
void turn(ros::Publisher& pub, bool right) {
  double angle = right ? -M_PI / 2.0 : M_PI / 2.0;
  double target = current_pose.theta + angle;

  // Keep updated global current direction
  if (right) current_direction = (current_direction + 1) % 4;
  else current_direction = (current_direction - 1 + 4) % 4;
  
  if (target > M_PI) target -= 2 * M_PI;
  if (target < -M_PI) target += 2 * M_PI;

  geometry_msgs::Twist cmd;
  cmd.angular.z = right ? -0.8 : 0.8;  

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    double error = target - current_pose.theta;
    if (error > M_PI) error -= 2 * M_PI;
    if (error < -M_PI) error += 2 * M_PI;
    if (fabs(error) < 0.2) break;     // Admit a small error

    pub.publish(cmd);
    rate.sleep();
  }

  cmd.angular.z = 0.0;
  pub.publish(cmd);
}

// Stops the robot for a determinate amount of seconds
void stop_for_seconds(ros::Publisher& pub, double seconds)
{
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;

    ros::Rate rate(100); // 100 Hz
    int ticks = static_cast<int>(seconds * 100);

    for (int i = 0; i < ticks && ros::ok(); ++i) {
        pub.publish(stop_cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

// The robot moves ahead a determined distance, and then stops
void move_forward_distance(ros::Publisher& pub, double distance_meters)
{
    geometry_msgs::Pose2D start_pos = current_pose;
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.1;  // Forward speed (adjust as needed)
    move_cmd.angular.z = 0.0;

    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();

        double dx = current_pose.x - start_pos.x;
        double dy = current_pose.y - start_pos.y;
        double dist = sqrt(dx*dx + dy*dy);

        if (dist >= distance_meters) break;

        pub.publish(move_cmd);
        rate.sleep();
    }

    // Stop after moving
    move_cmd.linear.x = 0.0;
    pub.publish(move_cmd);
}

// Checks if 2 positions are within 25 cm of one another
bool isSamePosition(const geometry_msgs::Pose2D& pose1, const geometry_msgs::Pose2D& pose2) {
    double dx = pose1.x - pose2.x;
    double dy = pose1.y - pose2.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance < 0.25; 
}

int main(int argc, char **argv)
{
  // Initialize the robot
  ros::init(argc, argv,"HolaBot");
  // Subscribe to the LaserScan to have constantly updated information
  ros::NodeHandle n;
  ros::Subscriber sub_odometry = n.subscribe("odom", 100, odomCallback);
  ros::Subscriber front_scan = n.subscribe("scan", 100, detect_front_wall);
  ros::Subscriber right_scan = n.subscribe("scan", 100, detect_right_wall);
  ros::Subscriber left_scan = n.subscribe("scan", 100, detect_left_wall);
  ros::Subscriber junction_scan = n.subscribe("scan", 100, junction);
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  ros::Rate rate(100);
  // Initialize the initial pose and create the first junction (necessary for later reference)
  geometry_msgs::Twist move;
  geometry_msgs::Pose2D init_pos;
  init_pos.x = current_pose.x;
  init_pos.y = current_pose.y;
  double init_theta = current_pose.theta;
  lastJunction_pose = current_pose;
  ROS_INFO("%f %f %f", init_pos.x, init_pos.y, init_theta);
  // Load the maze from the YAML file
  loadMazeGraph("/tmp/maze_graph.yaml");
  // Get the directions to exit the maze
  camino = computeDirectionQueue();

  while(ros::ok() )
  {
    
    // If not at a junction, keep moving same direction
    // otherwise, check the directions in "camino"
    if (atJunction && !isSamePosition(currentJunction_pose, lastJunction_pose)) {
      lastJunction_pose = currentJunction_pose; // Update the new lastJunction_pose

      // Current direction is the same indicated by the queue -> keep moving this direction
      if (camino.front() == current_direction) {
        move.linear.x = 0.1;
      }
      else if (camino.front() == (current_direction + 1) % 4) {           // Indicated direction is 1 to the right of current direction
        move_forward_distance(movement_pub, 0.1); 
        turn(movement_pub, 1);    // Turn right
        stop_for_seconds(movement_pub, 1);
        move.linear.x = 0.1;
        while(right_wall == 0) {
          movement_pub.publish(move);
          ros::spinOnce();
          rate.sleep();
        }
      }
      else if (camino.front() == (current_direction + 3) % 4) {         // // Indicated direction is 1 to the left of current direction
        move_forward_distance(movement_pub, 0.1); 
        turn(movement_pub, 0);    // Turn left
        stop_for_seconds(movement_pub, 1);
        move.linear.x = 0.1;
        while(left_wall == 0) {
          movement_pub.publish(move);
          ros::spinOnce();
          rate.sleep();
        }

      }
      camino.pop();   // Remove used direction
    }
    else {
      move.linear.x = 0.1;
    }

    // No more directionn -> robot is out of the maze
    if (camino.empty()) break;

    movement_pub.publish(move);

    ros::spinOnce();
    rate.sleep();
  };

  return 0;
}
