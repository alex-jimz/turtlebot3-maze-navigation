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

enum Direction {
    FRONT = 0, 
    RIGHT = 1, 
    BACK = 2,  
    LEFT = 3  
};

const double PI = 3.14159265358979323846;
geometry_msgs::Pose2D current_pose;
geometry_msgs::Pose2D last_junction_pose;
int front_wall = 0;
int right_wall = 1;
int left_wall = 1;
int finish = 0;
bool atJunction = false;
const float max_dist = 0.8;
int rot_time = 0;
int current_junction = 0;
int current_direction = 0;

// Constantly updates the current pose of the robot
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

// Detect the program has finished, that is, the robot has reached the exit, i.e. there are
// no walls in any direction (except back) for at least half a meter
void finished(const sensor_msgs::LaserScan::ConstPtr& msg) {
	finish = 1;
	for (int i = 280; i < 359; i++){
		if(msg->ranges[i] < 0.5 && msg->ranges[i] > 0.02){
			finish = 0;
			return;
		}
	}
	for (int i = 1; i < 80; i++){
		if(msg->ranges[i] < 0.5 && msg->ranges[i] > 0.02){
			finish = 0;
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
static int next_junc_id = 0;

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

// Function to save the graph in YAML format (saved to tmp folder)
void saveGraph(const std::string& path) {
  YAML::Node root(YAML::NodeType::Map);

  for (auto& kv : graph) {
    const Junction& J = kv.second;

    YAML::Node node(YAML::NodeType::Map);
    node["id"] = J.id;

    node["pose"] = YAML::Node(YAML::NodeType::Map);
    node["pose"]["x"]     = J.pose.x;
    node["pose"]["y"]     = J.pose.y;
    node["pose"]["theta"] = J.pose.theta;

    node["edges"] = YAML::Node(YAML::NodeType::Map);
    for (auto& e : J.edges) {
      node["edges"][ std::to_string(e.first) ] = e.second;
    }

    root[ std::to_string(J.id) ] = node;
  }

  std::ofstream fout(path);
  fout << root;
  ROS_INFO("Graph saved to %s", path.c_str());
}

// Checks if 2 positions are within 30 cm of one another
bool isSamePosition(const geometry_msgs::Pose2D& pose1, const geometry_msgs::Pose2D& pose2) {
    double dx = pose1.x - pose2.x;
    double dy = pose1.y - pose2.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance < 0.30;  
}

// Creates a new junction, unless a junction already exists in the indicated position
int createJunction(const geometry_msgs::Pose2D& p) {
  Junction j;
  for (auto junc : graph) {
	if (isSamePosition(p, junc.second.pose)) {
		last_junction_pose = p;
		ROS_INFO("Repeated junction id: %d", junc.first);
		return junc.first;
	}
  }

  j.id = next_junc_id++;
  j.pose = p;
  graph[j.id] = j;
  last_junction_pose = p;
  ROS_INFO("New junction id: %d", j.id);
  return j.id;
}

// Detect a junction
// A junction occurs when the robot must turn or a choice must be made
void junction(const sensor_msgs::LaserScan::ConstPtr& msg) {
	atJunction = !right_wall || !left_wall || front_wall;

	// If the junction is not essentialy the same as the one previously found (similar position)
	// A new junction is created and the corresponding node is linked to the last junction node
	if (atJunction && !isSamePosition(current_pose, last_junction_pose)) {
		int new_junction = createJunction(current_pose);
		ROS_INFO("Connecting %d with %d", current_junction, new_junction);
		graph[current_junction].edges[current_direction] = new_junction;
		graph[new_junction].edges[(current_direction + 2)%4] = current_junction;
		current_junction = new_junction;
	}
	return;
}

// Stops the robot for a determinate amount of seconds
void stop_for_seconds(ros::Publisher& pub, double seconds) {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;

    ros::Rate rate(100); 
    int ticks = static_cast<int>(seconds * 100);

    for (int i = 0; i < ticks && ros::ok(); ++i) {
        pub.publish(stop_cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

// The robot turns 90 degrees to the right if right = True, 90 degrees to the left otherwise
void turn(ros::Publisher& pub, bool right) {
	stop_for_seconds(pub, 0.5);
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
        if (fabs(error) < 0.2) break;	// Admit a small error

        pub.publish(cmd);
        rate.sleep();
    }

    cmd.angular.z = 0.0;
    pub.publish(cmd);
    stop_for_seconds(pub, 0.5);
}

// The robot moves ahead a determined distance, and then stops
void move_forward_distance(ros::Publisher& pub, double distance_meters)
{
    geometry_msgs::Pose2D start_pos = current_pose;
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.1;  
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

    move_cmd.linear.x = 0.0;
    pub.publish(move_cmd);
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
	ros::Subscriber finish_scan = n.subscribe("scan", 100, finished);
	ros::Subscriber junction_scan = n.subscribe("scan", 100, junction);
	ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::Rate rate(100);
	// Initialize the initial pose and create the first junction (necessary for later reference)
	geometry_msgs::Twist move;
	geometry_msgs::Pose2D init_pos;
	init_pos.x = current_pose.x;
	init_pos.y = current_pose.y;
	double init_theta = current_pose.theta;
	ROS_INFO("%f %f %f", init_pos.x, init_pos.y, init_theta);
	createJunction(current_pose);
	graph[0].edges[0] = 1;
	ROS_INFO("Created junction at initial position");

	// Implementation of "keep-to-the-right" technique
	while(ros::ok() )
	{
		// If the robot is out of the maze, end the while loop
		if (finish) {
			ROS_INFO("We have reached the EXIT.");
			break;
		}
		
		// If there is no wall on the right, go right
		if (right_wall == 0) {
			move_forward_distance(movement_pub, 0.1); 
			turn(movement_pub, 1);
			move.linear.x = 0.1;
			while(right_wall == 0) {
				movement_pub.publish(move);
				ros::spinOnce();
				rate.sleep();
			}
		}
		else if (front_wall == 1) {		// If there is a wall ahead (and a wall to the right), turn left
			turn(movement_pub, 0);
		}
		else {
			move.linear.x = 0.1;		// Otherwise, keep going straight
		}

		movement_pub.publish(move);

		ros::spinOnce();
		rate.sleep();
	};

	// Print the information collected in the graph and save it to the YAML file
	printGraph();
	saveGraph("/tmp/maze_graph.yaml");
	return 0;
}
