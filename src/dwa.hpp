#pragma once


#include <cfloat>
#include <iostream>

struct Pose {
    double x;
    double y;
    double yaw;
};

struct DynamicWindow {
    std::vector<float> possibleV;
    std::vector<float> possibleW;
    int nPossibleV;
    int nPossibleW;
};

struct Velocity {
    float linearVelocity;
    float angularVelocity;
};

struct Point{
    float x;
    float y;
};

//Obstacle positions or LiDAR points from obstacles
struct PointCloud {
    std::vector<Point> points;
};

struct Trajectory {
    std::vector<Pose> poses;
};

enum class RobotType {
    RECTANGLE,
    CIRCLE
};

/**
 * @brief Class for implementing dynamic window approach for obstacle avoidance 
 * 
 * This class provides methods to avoid obstacles
 * while navigating toward a goal position.
 */
class DynamicWindowApproach {
public:

    struct Configuration {
        float minSpeed;
        float maxSpeed;

        float maxYawrate;
        float maxAccel;
        float maxdYawrate;

        float velocityResolution;
        float yawrateResolution;
        float dt;

        //Minimum distance to maintain from obstacles
        float safety_distance;

        //Prediction horizon
        float predictTime;

        RobotType robot_type;

        //Rectangle robot
        float robot_length;
        float robot_width;

        //Circle robot
        float robot_radius;

        //Cost wieghts
        float heading_weight;
        float velocity_weight;
        float clearance_weight;
    };

    static inline const Configuration DEFAULT_CONFIGURATION = {
        .minSpeed = -0.5, //  [m/s]
        .maxSpeed = 0.6, //  [m/s]
        .maxYawrate = 3.2 , //  [rad/s] 
        .maxAccel = 10, //  [m/s^2]
        .maxdYawrate = 40, //  [rad/s^2] 

        .velocityResolution = 0.1, //  [m/s^2]
        .yawrateResolution = 0.1,  //  [rad/s^2] 
        .dt = 0.05, // [s]

        .safety_distance = 0.5 , // [m]
        .predictTime = 1, // [s]

        .robot_type = RobotType::CIRCLE,
        .robot_length = 1, // [m]
        .robot_width = 0.9, // [m]

        .robot_radius = 0.8, // [m]

        .heading_weight = 0.15,
        .velocity_weight = 1,
        .clearance_weight = 1
    };

    DynamicWindowApproach();

    DynamicWindowApproach(Configuration &config);

    /**
     * @brief Set the goal position
     * 
     * @param goal_position Target position to navigate toward
     */
    inline void set_goal_position(const Point& goal_position){
        m_goal_position = goal_position;
    }

    /**
     * @brief Get the current goal position
     * 
     * @return The current goal position
     */
    inline Point get_goal_position() const{
        return m_goal_position;
    }

    /**
     * @brief Runs the DynamicWindowApproach algorithm 
     * 
     * @param current_pose
     * @param current_velocity
     * @param goal_position  
     * @param point_cloud Obstacle positions or LiDAR points from obstacles
     * 
     * @return The velocity (linear and angular) chosen by the algo
     */
    Velocity planning(
        const Pose &current_pose,
        const Velocity &current_velocity,
        const Point &goal_position,
        const PointCloud &pointCloud);

private:

    /**
     * @brief Compute the dynamic window for the robot
     * 
     * @param velocity Current robot velocity
     */
    void compute_dynamic_window(
        const Velocity& velocity
    );


    /**
     * @brief Velocity cost encourages high linear velocity 
     * 
     * @param velocity Current robot velocity
     */
    inline float compute_velocity_cost(
        const Velocity& velocity
    ){
        return m_configuration.maxSpeed - velocity.linearVelocity;
    }

    /**
     * @brief Computes the trajectory performed by the robot 
     * 
     * @param initial_pose 
     * @param velocity the robot velocity (it's constant durinf the prediction)
     * @param trajectory robot pose trajectory to be populated 
     */
    void predict_trajectory(
        const Pose& initial_pose,
        const Velocity& velocity,
        Trajectory& trajectory
    );

    /**
     * @brief Heading cost encourages the robot to point the goal 
     * 
     * @param pose pose of the robot
     * @param goal_position goal that has to be reached
     */
    inline float compute_heading_cost(Pose pose, Point goal_position) {
        float dx = goal_position.x - pose.x;
        float dy = goal_position.y - pose.y;
        float angleError = atan2(dy, dx);
        float angleCost = angleError - pose.yaw;
        return fabs(atan2(sin(angleCost), cos(angleCost)));
    }

    /**
     * @brief Returns the inverse of the minimum distance from obstacles 
     *        experienced in the trajectory or a high value in case of collision
     * 
     * @param trajectory trajectory to be checked
     * @param velocity goal that has to be reached
     */
    float calculateClearanceCost(
        const Trajectory &trajectory, 
        const PointCloud &pointCloud
    );


    /**
     * @brief Compute the pose of the robot at the next time step
     * 
     * @param pose Current pose of the robot
     * @param velocity Velocity command for the robot
     * @param dt Time step
     * @return Next pose of the robot
     */
    Pose motion_model(
        const Pose& pose,
        const Velocity& velocity,
        double dt
    );

    // The algorithm configuration
    Configuration m_configuration;

    // The velocity dynamic window
    DynamicWindow m_dynamic_window;

    PointCloud m_point_cloud;

    // Goal position for navigation
    Point m_goal_position;
    
}; 
