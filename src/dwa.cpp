#include "dwa.hpp"


DynamicWindowApproach::DynamicWindowApproach() {
    m_configuration = DEFAULT_CONFIGURATION;
}

DynamicWindowApproach::DynamicWindowApproach(Configuration &config) {
    m_configuration = config;
}

void DynamicWindowApproach::compute_dynamic_window(
    const Velocity& velocity
) {

    float minV = std::max(m_configuration.minSpeed, 
        velocity.linearVelocity- m_configuration.maxAccel * m_configuration.dt);
    float maxV = std::min(m_configuration.maxSpeed, 
        velocity.linearVelocity + m_configuration.maxAccel * m_configuration.dt);
    float minW = std::max(-m_configuration.maxYawrate, 
        velocity.angularVelocity - m_configuration.maxdYawrate * m_configuration.dt);
    float maxW = std::min(m_configuration.maxYawrate, 
        velocity.angularVelocity + m_configuration.maxdYawrate * m_configuration.dt);

    int nPossibleV = (maxV - minV) / m_configuration.velocityResolution;
    int nPossibleW = (maxW - minW) / m_configuration.yawrateResolution;

    // Resize the vectors before accessing elements
    m_dynamic_window.possibleV.resize(nPossibleV);
    m_dynamic_window.possibleW.resize(nPossibleW);

    m_dynamic_window.nPossibleV = nPossibleV;
    m_dynamic_window.nPossibleW = nPossibleW;

    for(size_t i=0; i < nPossibleV; i++) {
        m_dynamic_window.possibleV[i] = minV + (float)i * m_configuration.velocityResolution;
    }

    for(size_t i=0; i < nPossibleW; i++) {
        m_dynamic_window.possibleW[i] = minW + (float)i * m_configuration.yawrateResolution;
    }

}


Pose DynamicWindowApproach::motion_model(
    const Pose& pose,
    const Velocity& velocity,
    double dt
) {
    Pose new_pose;
    new_pose.yaw = pose.yaw + velocity.angularVelocity * dt;
    new_pose.x = pose.x + velocity.linearVelocity * cos(new_pose.yaw) * dt;
    new_pose.y = pose.y + velocity.linearVelocity * sin(new_pose.yaw) * dt;
    return new_pose;
}

void DynamicWindowApproach::predict_trajectory(
    const Pose& initial_pose,
    const Velocity& velocity,
    Trajectory& trajectory
) {
    Pose sim_pose = initial_pose;
    float time = 0.0;
    while (time < m_configuration.predictTime) {
        sim_pose = motion_model(sim_pose, velocity, m_configuration.dt);
        trajectory.poses.push_back(sim_pose);
        time += m_configuration.dt;
    }
}

float DynamicWindowApproach::calculateClearanceCost(
    const Trajectory& trajectory,
    const PointCloud& pointCloud
){
    // Check if pointCloud or trajectory is empty
    if (pointCloud.points.empty() || trajectory.poses.empty()) {
    return 0.0f;  // No obstacles or trajectory, no cost
    }

    float min_r = std::numeric_limits<float>::max();

    // For each trajectory point
    for (const auto& pose : trajectory.poses) {
        // For each obstacle
        for (const auto& obstacle : pointCloud.points) {
            // Calculate Euclidean distance for minimum distance calculation
            float dx = pose.x - obstacle.x;
            float dy = pose.y - obstacle.y;
            float r = std::hypot(dx, dy);
            
            // Update minimum distance
            min_r = std::min(min_r, r);
            
            // Check for collision based on robot type
            if (m_configuration.robot_type == RobotType::RECTANGLE) {
                // Calculate vector from pose to obstacle
                float vec_x = obstacle.x - pose.x;
                float vec_y = obstacle.y - pose.y;
                
                // Calculate rotation matrix components
                float cos_yaw = std::cos(pose.yaw);
                float sin_yaw = std::sin(pose.yaw);
                
                // Transform to local coordinates (obstacle position in robot frame)
                float local_x = cos_yaw * vec_x + sin_yaw * vec_y;
                float local_y = -sin_yaw * vec_x + cos_yaw * vec_y;
                
                // Check if obstacle is inside the rectangle
                bool upper_check = local_x <= m_configuration.robot_length / 2;
                bool right_check = local_y <= m_configuration.robot_width / 2;
                bool bottom_check = local_x >= -m_configuration.robot_length / 2;
                bool left_check = local_y >= -m_configuration.robot_width / 2;
                
                if (upper_check && right_check && bottom_check && left_check) {
                    return std::numeric_limits<float>::infinity();  // Collision detected
                }
            }
            else if (m_configuration.robot_type == RobotType::CIRCLE) {
                if (r <= m_configuration.robot_radius) {
                    return std::numeric_limits<float>::infinity();  // Collision detected
                }
            }
        }
    }

    // If no collision, return inverse of minimum distance
    return 1.0f / min_r;
}

Velocity DynamicWindowApproach::planning(
    const Pose &current_pose,
    const Velocity &current_velocity,
    const Point &goal,
    const PointCloud &pointCloud
){

  compute_dynamic_window(current_velocity);
  Velocity try_velocity;
  float total_cost = FLT_MAX;
  float cost;
  Velocity bestVelocity = {0.0f, 0.0f};
  Trajectory trajectory;
  trajectory.poses.resize(m_configuration.predictTime / m_configuration.dt);

  for (int i = 0; i < m_dynamic_window.nPossibleV; ++i) {
    for (int j = 0; j < m_dynamic_window.nPossibleW; ++j) {

      try_velocity.linearVelocity = m_dynamic_window.possibleV[i];
      try_velocity.angularVelocity = m_dynamic_window.possibleW[j];
      trajectory.poses.clear();
      predict_trajectory(current_pose, try_velocity, trajectory);
    
      cost = 
        m_configuration.velocity_weight * compute_velocity_cost(try_velocity) +
        m_configuration.heading_weight * compute_heading_cost(trajectory.poses.back(), goal) +
        m_configuration.clearance_weight * calculateClearanceCost(trajectory, pointCloud);

        if (cost < total_cost) {
            total_cost = cost;
            bestVelocity = try_velocity;
        }
    }
  }

  std::cout<< "Best velocity: "<<bestVelocity.linearVelocity<<" "<<bestVelocity.angularVelocity<<std::endl;
  return bestVelocity;
}
