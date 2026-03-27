/**
 * Copyright (c) 2025, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * @file ik_solver_node.cpp
 * @brief ROS2 node implementing position-based inverse kinematics for a
 *        5-DOF robotic arm on QNX.
 *
 * This node receives Cartesian velocity commands from the arm controller
 * node, integrates them into a Cartesian position target, and uses the
 * KDL Levenberg-Marquardt (LMA) position IK solver to compute joint
 * angles that reach the target. The computed joint angles are published
 * for the arm controller to convert to servo PWM signals.
 *
 * Architecture:
 *     The node operates in an event-driven model — IK solving only occurs
 *     when Cartesian commands are received (i.e. when the joystick is
 *     actively moving). This avoids unnecessary computation when the arm
 *     is stationary.
 *
 * 5-DOF Handling:
 *     A 5-DOF arm cannot fully control all 6 Cartesian dimensions
 *     (X, Y, Z position + Roll, Pitch, Yaw orientation) simultaneously.
 *     To handle this, the LMA solver is initialized with a weight matrix
 *     that prioritizes position (X, Y, Z) over orientation (Roll, Pitch,
 *     Yaw), allowing the solver to freely choose orientation while
 *     accurately reaching the desired position.
 *
 * Joint Limit Handling:
 *     KDL's ChainIkSolverPos_LMA does not enforce joint limits during
 *     solving. Joint limits are loaded from the URDF via urdf::Model
 *     (not from the KDL chain which discards limits) and applied as
 *     post-solve clamping. Dynamic limits can also be received from the
 *     arm controller via /CurrentPositions topic to reflect runtime
 *     servo safety limits.
 *
 * Servo Hardware:
 *     - Joints 0-2: DS3218 servos, 270 degree mode
 *       Pulse range: 500-2500us, Neutral: 1500us [DS3218 Datasheet]
 *     - Joints 3-4: MG90S micro servos, 180 degree mode
 *       Pulse range: 1000-2000us, Neutral: 1500us [MG90S Datasheet]
 *     - Joint 5 (Gripper): Controlled directly by arm controller,
 *       not by IK solver.
 *
 * ROS2 Topics:
 *     Subscriptions:
 *         - /CartesianCmd (std_msgs/Float64MultiArray):
 *           Cartesian velocity commands from the arm controller.
 *           Format: [X_vel, Y_vel, Z_vel, home_flag]
 *           Only published when joystick is actively moving.
 *
 *         - /CurrentPositions (std_msgs/Float64MultiArray):
 *           Current joint positions in radians from the arm controller
 *           for state synchronization. Also receives dynamic joint
 *           limits identified by flag values appended to the message:
 *               flag 0.0 = position data
 *               flag 1.0 = lower limit data
 *               flag 2.0 = upper limit data
 *
 *     Publications:
 *         - /Mov (std_msgs/Float64MultiArray):
 *           Computed joint angles in radians for joints 0-4.
 *           Published only when IK solve succeeds.
 *
 * References:
 *     - Orocos KDL: https://www.orocos.org/wiki/orocos/kdl-wiki
 */

  #include <rclcpp/rclcpp.hpp>
  #include <sensor_msgs/msg/joy.hpp>
  #include <std_msgs/msg/float64_multi_array.hpp>
  #include <orocos_kdl/kdl/chain.hpp>
  #include <orocos_kdl/kdl/tree.hpp>
  #include <kdl_parser/kdl_parser.hpp>
  #include <urdf/model.h>
  #include <kdl/jntarray.hpp>
  #include <kdl/tree.hpp>
  #include <kdl/frames.hpp>
  #include <kdl/chainfksolverpos_recursive.hpp>
  #include <kdl/chainiksolverpos_lma.hpp>
  #include <vector>
  #include <string>
  #include <fstream>
  #include <sstream>
  #include <unistd.h>
  #include <pthread.h>

  /**
 * @class IKSolverNode
 * @brief ROS2 node that solves position-based inverse kinematics for
 *        a 5-DOF robotic arm using the KDL LMA solver.
 */
  class IKSolverNode : public rclcpp::Node {
  public:
    IKSolverNode() : Node("ik_solver_node") {
      RCLCPP_INFO(this->get_logger(), "IK Solver Node started.");

      last_update_time_ = this->now().seconds();

      this->declare_parameter<std::string>("urdf_file", "arm5dof.urdf");
      this->declare_parameter<std::string>("base_link", "world");
      this->declare_parameter<std::string>("end_effector_link", "Gripper_Assembly_1");
      this->declare_parameter<double>("update_rate", 50.0);
      this->declare_parameter<double>("velocity_scale", 0.05);
      this->declare_parameter<bool>("limits_updated_up", false);
      this->declare_parameter<bool>("limits_updated_low", false);

      std::string urdf_filename = this->get_parameter("urdf_file").as_string();

      RCLCPP_INFO(this->get_logger(), "urdf_file parameter value: '%s'", urdf_filename.c_str());
      

      
      // Get parameters
      urdf_file_ = this->get_parameter("urdf_file").as_string();
      base_link_ = this->get_parameter("base_link").as_string();
      end_effector_link_ = this->get_parameter("end_effector_link").as_string();
      velocity_scale_ = this->get_parameter("velocity_scale").as_double();
      
      
      RCLCPP_INFO(this->get_logger(), "  Configuration:");
      RCLCPP_INFO(this->get_logger(), "  URDF file: %s", urdf_file_.c_str());
      RCLCPP_INFO(this->get_logger(), "  Base link: %s", base_link_.c_str());
      RCLCPP_INFO(this->get_logger(), "  End effector: %s", end_effector_link_.c_str());

      urdf_file_ = findURDF(urdf_filename);
      
      // Check if found
      if (urdf_file_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "URDF not found. Exiting.");
        rclcpp::shutdown();
        return;
      }
      
      // ========== Load URDF ==========
      if (!loadURDF()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load URDF! Node will exit.");
        rclcpp::shutdown();
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "  URDF loaded successfully!");
      RCLCPP_INFO(this->get_logger(), "  KDL chain has %u joints", kdl_chain_.getNrOfJoints());
      RCLCPP_INFO(this->get_logger(), "  Joint names:");
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "    [%zu] %s", i, joint_names_[i].c_str());
      }

       // --- Load Joint Limits ---

      if (!loadJointLimits()) {
        RCLCPP_ERROR(this->get_logger(),
            "Failed to load joint limits. Exiting.");
        rclcpp::shutdown();
        return;
      }
      
      // --- Initialize Solvers ---

      if (!initSolvers()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize solvers. Exiting.");
        rclcpp::shutdown();
        return;
      }
      
      joint_lower_temp.resize(num_joints_ + 3, 0.0);
      joint_upper_temp.resize(num_joints_ + 3, 0.0);
      
      // Publisher for /Mov
      joint_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/Mov", 10
      );

      cartesian_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/CartesianCmd",
            10,
            std::bind(&IKSolverNode::cartesian_callback, this, std::placeholders::_1)
      );

      pos_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/CurrentPositions",
            10,
            std::bind(&IKSolverNode::current_positions_callback, this, std::placeholders::_1)
      );

    }


  private:
    // ======================================================================
    // URDF Loading and Configuration
    // ======================================================================

    /**
     * @brief Searches for the URDF file in the URDF_PATH directory.
     *
     * Uses the URDF_PATH environment variable to locate the URDF file.
     * This allows the same binary to work with different arm configurations
     * by changing the environment variable.
     *
     * @param filename: Name of the URDF file to find (e.g. "arm5dof.urdf")
     * @return Full path to the URDF file, or empty string if not found.
     */
    std::string findURDF(const std::string& filename) {
      
      // Get your environment variable
      const char* config_dir = std::getenv("URDF_PATH");
      
      if (config_dir == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "URDF_PATH not set");
        return "";
      }
      
      // Build full path
      std::string full_path = std::string(config_dir) + "/" + filename;
      
      RCLCPP_INFO(this->get_logger(), "Looking for URDF at: '%s'", full_path.c_str());
      
      // Check if file exists
      std::ifstream test(full_path);
      if (test.is_open()) {
        test.close();
        RCLCPP_INFO(this->get_logger(), "  URDF found!");
        return full_path;
      }
      
      RCLCPP_ERROR(this->get_logger(), "  URDF not found!");
      return "";
    }
    /**
     * @brief Loads the URDF file and constructs the KDL kinematic chain.
     *
     * Process:
     *     1. Parse URDF XML into a KDL tree using kdl_parser
     *     2. Extract the kinematic chain from base_link to end_effector_link
     *     3. Extract movable joint names from the chain (skip fixed joints)
     *
     * The KDL chain contains joint geometry (axes, origins, link lengths)
     * but does NOT retain joint limits. Joint limits are loaded separately
     * via loadJointLimits() using urdf::Model.
     *
     * @return true if URDF loaded and chain extracted successfully.
     */
    bool loadURDF() {
      RCLCPP_INFO(this->get_logger(), "Loading URDF from: %s", urdf_file_.c_str());

      //  Convert URDF file to KDL tree
      KDL::Tree kdl_tree;
      if (!kdl_parser::treeFromFile(urdf_file_, kdl_tree)) {
        RCLCPP_ERROR(this->get_logger(), " Failed to construct KDL tree from URDF");
        return false;
      }
      
      RCLCPP_INFO(this->get_logger(), " KDL tree constructed");
      RCLCPP_INFO(this->get_logger(), "  Tree has %u segments", kdl_tree.getNrOfSegments());
      
      // Extract chain from base to end-effector
      if (!kdl_tree.getChain(base_link_, end_effector_link_, kdl_chain_)) {
        RCLCPP_ERROR(this->get_logger(), "   Failed to extract chain from '%s' to '%s'",
                    base_link_.c_str(), end_effector_link_.c_str());
        RCLCPP_ERROR(this->get_logger(), "   Check if these link names exist in URDF:");
        
        auto segments = kdl_tree.getSegments();
        RCLCPP_INFO(this->get_logger(), "   Available segments in tree:");
        for (const auto& seg : segments) {
          RCLCPP_INFO(this->get_logger(), "     - %s", seg.first.c_str());
        }
        
        return false;
      }
      
      RCLCPP_INFO(this->get_logger(), "  KDL chain extracted");
      RCLCPP_INFO(this->get_logger(), "  Chain: %s → %s", base_link_.c_str(), end_effector_link_.c_str());
      RCLCPP_INFO(this->get_logger(), "  Segments: %u", kdl_chain_.getNrOfSegments());
      RCLCPP_INFO(this->get_logger(), "  Joints: %u", kdl_chain_.getNrOfJoints());
      
      extractJointNames();
      
      return true;
    }

    /**
     * @brief Extracts movable joint names from the KDL chain.
     *
     * Iterates through all segments in the chain and collects names
     * of joints whose type is not KDL::Joint::None (i.e. skips
     * fixed joints like the world_to_base joint).
     *
     * Joint names are used to look up limits from the URDF model.
     */
    void extractJointNames() {
      joint_names_.clear();
      
      for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i) {
        KDL::Segment segment = kdl_chain_.getSegment(i);
        KDL::Joint joint = segment.getJoint();
        
        // Only add movable joints (skip fixed joints)
        if (joint.getType() != KDL::Joint::None) {
          joint_names_.push_back(joint.getName());
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "  Extracted %zu joint names", joint_names_.size());
    }

    /**
     * @brief Loads joint limits from the URDF using urdf::Model.
     *
     * KDL does not store joint limits in its chain representation,
     * so limits must be loaded separately. This function parses
     * the URDF file again using urdf::Model which provides access
     * to the <limit> tags defined in each joint.
     *
     * Limits are stored in joint_lower_limits_ and joint_upper_limits_
     * vectors and used for post-solve clamping after IK computation.
     *
     * For joints without defined limits, defaults to +/- PI radians
     * with a warning logged.
     *
     * @return true if URDF parsed and limits loaded successfully.
     */
    bool loadJointLimits() {
    // Parse URDF model separately to get limits
      if (!urdf_model_.initFile(urdf_file_)) {
          RCLCPP_ERROR(this->get_logger(),
              "Failed to parse URDF for joint limits");
          return false;
      }

      joint_lower_limits_.clear();
      joint_upper_limits_.clear();

      // Walk through joint names extracted from KDL chain
      for (const auto& name : joint_names_) {
          auto joint = urdf_model_.getJoint(name);
          if (joint && joint->limits) {
              joint_lower_limits_.push_back(joint->limits->lower);
              joint_upper_limits_.push_back(joint->limits->upper);
              RCLCPP_INFO(this->get_logger(),
                  "Joint %s limits: [%.4f, %.4f]",
                  name.c_str(),
                  joint->limits->lower,
                  joint->limits->upper);
          } else {
              // No limits defined for this joint
              joint_lower_limits_.push_back(-M_PI);
              joint_upper_limits_.push_back(M_PI);
              RCLCPP_WARN(this->get_logger(),
                  "No limits found for joint %s, using ±π",
                  name.c_str());
          }
      }
      return true;
  }

    /**
     * @brief Initializes FK and position IK solvers.
     *
     * Creates two solvers:
     *     1. ChainFkSolverPos_recursive: Computes gripper Cartesian position
     *        from joint angles by multiplying transformation matrices through
     *        the kinematic chain. Always succeeds.
     *
     *     2. ChainIkSolverPos_LMA: Levenberg-Marquardt position IK solver.
     *        Iteratively finds joint angles that place the end effector at
     *        the target Cartesian position. May fail if target is unreachable.
     *
     * Also computes the initial Cartesian target using FK at home
     *
     * @return true if both solvers initialized successfully.
     */
    bool initSolvers() {
      // FK solver: joint angles → cartesian position
      fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

      // LMA weight vector for 5-DOF position-priority solving
      // [X, Y, Z, Roll, Pitch, Yaw]
      // Position fully weighted, orientation free
      Eigen::Matrix<double,6,1> L;
      L << 1, 1, 1, 0, 0, 0;  // Higher weight on position vs orientation

      // Position IK solver: cartesian position → joint angles
      ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_, L);

      // Initialize joint arrays
      num_joints_ = kdl_chain_.getNrOfJoints();
      current_joint_positions_ = KDL::JntArray(num_joints_);
      for (unsigned int i = 0; i < num_joints_; ++i) {
          current_joint_positions_(i) = 0.0;
      }

      // Use FK to find the cartesian position at home (all joints = 0)
      // This becomes the initial cartesian target
      KDL::Frame initial_frame;
      fk_solver_->JntToCart(current_joint_positions_, initial_frame);
      cartesian_target_ = initial_frame;
      target_initialized_ = true;

      RCLCPP_INFO(this->get_logger(), "✓ Solvers initialized");
      RCLCPP_INFO(this->get_logger(), "  Joints: %u", num_joints_);
      RCLCPP_INFO(this->get_logger(), "  Home cartesian position: [%.3f, %.3f, %.3f]",
                  initial_frame.p.x(),
                  initial_frame.p.y(),
                  initial_frame.p.z());
      return true;
    }

    /**
     * @brief Receives current joint positions or dynamic limits from
     *        the arm controller.
     *
     * The arm controller publishes to /CurrentPositions with a flag
     * value appended as the last element to identify the message type:
     *     - flag 0.0: Current joint positions in radians.
     *       Updates current_joint_positions_ and resyncs cartesian_target_
     *       via FK.
     *     - flag 1.0: Lower joint limits in radians.
     *       Stored in temporary buffer until both limits received.
     *     - flag 2.0: Upper joint limits in radians.
     *       Stored in temporary buffer until both limits received.
     *
     * @param msg: Float64MultiArray containing joint data plus flag.
     *     data[0..N-1]: Joint values (positions or limits) in radians
     *     data[N]:      Gripper placeholder (ignored)
     *     data[N+1]:    Flag (0.0=position, 1.0=lower limit, 2.0=upper limit)
     */
    void current_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
    {
      if (msg->data.size() < num_joints_) {
          RCLCPP_WARN(this->get_logger(),
              "CurrentPositions message too short. "
              "Expected at least %u values, got %zu",
              num_joints_, msg->data.size());
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Size of msg->data: %zu", msg->data.size());
      RCLCPP_INFO(this->get_logger(), "Last Index of value: %f", msg->data[msg->data.size() - 1]);
      if(msg->data[msg->data.size() - 1] == 0.0) {
      for (unsigned int i = 0; i < num_joints_; ++i) {
          current_joint_positions_(i) = msg->data[i];
      }
      fk_solver_->JntToCart(current_joint_positions_, cartesian_target_);
      }

      else if(msg->data[msg->data.size() - 1] == 1.0)
      {
        for (unsigned int i = 0; i < num_joints_; ++i) {
          joint_lower_temp[i] = msg->data[i];
        }
        limits_updated_low = true;
        RCLCPP_INFO(this->get_logger(), "Size of msg->data: %zu", msg->data.size());
      RCLCPP_INFO(this->get_logger(), "Last Index of value: %f", msg->data[msg->data.size() - 1]);
      }

      else if(msg->data[msg->data.size() - 1] == 2.0)
      {
        for (unsigned int i = 0; i < num_joints_; ++i) {
          joint_upper_temp[i] = msg->data[i];
        }
        RCLCPP_INFO(this->get_logger(), "Size of msg->data: %zu", msg->data.size());
      RCLCPP_INFO(this->get_logger(), "Last Index of value: %f", msg->data[msg->data.size() - 1]);
        limits_updated_up = true;
      }
    }

    /**
     * @brief Main IK solving callback triggered by Cartesian velocity
     *        commands from the arm controller.
     *
     * Process:
     *     1. Apply any pending dynamic joint limit updates
     *     2. Handle home button press (reset to all zeros)
     *     3. Integrate velocity commands into Cartesian target position
     *     4. Solve position IK using ChainIkSolverPos_LMA
     *     5. On failure: roll back Cartesian target to prevent drift
     *     6. On success: clamp solution to joint limits and publish
     *
     * @param msg: Float64MultiArray containing velocity command.
     *     data[0]: X velocity
     *     data[1]: Y velocity
     *     data[2]: Z velocity
     *     data[3]: Home button flag (>0.5 = pressed)
     */
    void cartesian_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
    {
      if (!target_initialized_) return;

      if (msg->data.size() < 4) {
        RCLCPP_WARN(this->get_logger(),
            "CartesianCmd message too short. "
            "Expected 4 values got %zu",
            msg->data.size()
        );
        return;
      }
      if(limits_updated_up && limits_updated_low){
        for (unsigned int i = 0; i < num_joints_; i++) {
          joint_lower_limits_[i] = joint_lower_temp[i];
          joint_upper_limits_[i] = joint_upper_temp[i];
        }
        limits_updated_up = false;
        limits_updated_low = false;
        
      }

      double x_vel        = msg->data[0];
      double y_vel        = msg->data[1];
      double z_vel        = msg->data[2];
      bool   home_pressed = (msg->data[3] > 0.5);

      // Calculate dt since last update
      double now = this->now().seconds();
      double dt = now - last_update_time_;
      if (dt > 0.5) dt = 0.02;   // cap large gaps
      last_update_time_ = now;

      if (home_pressed) {
          for (unsigned int i = 0; i < num_joints_; ++i) {
              current_joint_positions_(i) = 0.0;
          }
          fk_solver_->JntToCart(
              current_joint_positions_,
              cartesian_target_
          );
          return;
      }

      // Joystick axes move the TARGET POINT in cartesian space
      // axes[4] = Left stick U/D  → X (forward/back)
      // axes[5] = Left stick L/R  → Y (left/right)
      // axes[3] = Right stick U/D → Z (up/down)
                  
      cartesian_target_.p.x(
          cartesian_target_.p.x() + x_vel * velocity_scale_ * dt
      );
      cartesian_target_.p.y(
          cartesian_target_.p.y() + y_vel * velocity_scale_ * dt
      );
      cartesian_target_.p.z(
          cartesian_target_.p.z() + (-z_vel) * velocity_scale_ * dt
      );

      // Use current joint positions as the initial guess
      // This helps the solver find the closest solution
      KDL::JntArray solution(num_joints_);
      int result = ik_pos_solver_->CartToJnt(
          current_joint_positions_,   // initial guess
          cartesian_target_,          // desired cartesian pose
          solution                    // output joint angles
      );

      if (result < 0) {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              1000,
              "Position IK failed (result=%d). "
              "Target may be out of reach: [%.3f, %.3f, %.3f]",
              result,
              cartesian_target_.p.x(),
              cartesian_target_.p.y(),
              cartesian_target_.p.z()
          );

          cartesian_target_.p.x(
              cartesian_target_.p.x() - x_vel * velocity_scale_ * dt
          );
          cartesian_target_.p.y(
              cartesian_target_.p.y() - y_vel * velocity_scale_ * dt
          );
          cartesian_target_.p.z(
              cartesian_target_.p.z() - (-z_vel) * velocity_scale_ * dt
          );
          return;
      }

      for (unsigned int i = 0; i < num_joints_; ++i) {
        solution(i) = std::max(
            joint_lower_limits_[i],
            std::min(joint_upper_limits_[i], solution(i))
        );
      }

      // This is used as the initial guess for the next solve
      current_joint_positions_ = solution;

      publishJointCommand(solution);


    }
  
    /**
     * @brief Publishes computed joint angles to the /Mov topic.
     *
     * Converts KDL JntArray to a Float64MultiArray message containing
     * joint angles in radians for joints 0-4. 
     *
     * @param joint_angles: KDL JntArray containing solved joint angles
     *     in radians for each joint in the kinematic chain.
     */
  void publishJointCommand(const KDL::JntArray& joint_angles) 
  {
      auto msg = std_msgs::msg::Float64MultiArray();

      for (unsigned int i = 0; i < num_joints_; ++i) {
          msg.data.push_back(joint_angles(i));
      }

      joint_command_publisher_->publish(msg);
  }



    double last_update_time_ = 0.0;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_publisher_;
    

    urdf::Model urdf_model_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;

    std::vector<double> joint_lower_temp;
    std::vector<double> joint_upper_temp;

    std::string urdf_file_;
    std::string base_link_;
    std::string end_effector_link_;
    KDL::Chain kdl_chain_;
    std::vector<std::string> joint_names_;
    unsigned int num_joints_;
    KDL::JntArray current_joint_positions_;
    KDL::Frame cartesian_target_;
    bool target_initialized_ = false; 

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver_; 
    // No position solver needed!

    double velocity_scale_ = 0.01;

    bool limits_updated_up = false;
    bool limits_updated_low = false;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_subscription_;


  };

  int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
