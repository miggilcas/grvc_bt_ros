#include <ros/ros.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <gnc_functions.hpp>
#include <mutex>

using namespace BT;
// Here we will define the dummy functions for actions and conditions
//
// Simple function that return a NodeStatus
BT::NodeStatus CheckOnAir() {
  // If the % of the platform visible is greater than 1
  ros::spinOnce();
  if (current_extended_state_g.landed_state == 2 ||
      current_extended_state_g.landed_state == 3) {
    ROS_INFO("Drone On Air or already taking off\n");
    return BT::NodeStatus::SUCCESS;
  } else {

    if (current_extended_state_g.landed_state == 0 ||
        current_extended_state_g.landed_state == 4) {
      ROS_INFO("Drone landed state undefined or landing");
      return BT::NodeStatus::FAILURE;
    }
    if (current_extended_state_g.landed_state == 1) {
      ROS_INFO("Drone on ground, ready to take off ");
    }
    return BT::NodeStatus::RUNNING;
  }
}
ros::Subscriber vel_sub;
double velocity_ = 0;
void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  velocity_ = std::sqrt(std::pow(msg->twist.linear.x, 2) +
                        std::pow(msg->twist.linear.y, 2) +
                        std::pow(msg->twist.linear.z, 2));
  // ROS_INFO("COSITES velocity: %f", velocity_);
}
class WaitUntil : public ConditionNode {
public:
  WaitUntil(const std::string &name, const NodeConfiguration &config)
      : ConditionNode(name, config) {
    // Initialize ROS node handle and subscriber
  }

  static PortsList providedPorts() { return {}; }

  NodeStatus tick() override {
    sleep(0.5);
    ros::spinOnce();
    ROS_INFO("%f", velocity_);            // palante
    if (velocity_ < 0.07 && velocity_ > 0) // Check if velocity is close to zero
    {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
  }
};

// GoToGoal and custom type to specify a goal
// Custom type
struct Pose3D {
  double x, y, z, psi;
};
namespace BT {
template <> inline Pose3D convertFromString(StringView key) {
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    throw BT::RuntimeError("invalid input)");
  } else {
    Pose3D output;
    output.x = convertFromString<double>(parts[0]);
    output.y = convertFromString<double>(parts[1]);
    output.z = convertFromString<double>(parts[2]);
    output.psi = convertFromString<double>(parts[3]);
    return output;
  }
}
} // end namespace BT

class GoToGoal : public BT::AsyncActionNode {
public:
  GoToGoal(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config) {}

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts() {
    return {BT::InputPort<Pose3D>("goal")};
  }
  // You must override the virtual function tick()
  virtual BT::NodeStatus tick() override {
    // Take the goal from the input port of the nodes
    Pose3D goal;
    bool going = false;
    if (!getInput<Pose3D>("goal", goal)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [goal]");
    }
    // At first we need to specify the reference frame

    ros::ServiceClient reference_client;
    mavros_msgs::SetMavFrame srv;

    reference_client = node_.serviceClient<mavros_msgs::SetMavFrame>(
        "/mavros/setpoint_position/mav_frame");
    srv.request.mav_frame = 1;
    reference_client.call(srv);

    if (srv.response.success) {
      ROS_INFO("Changed the frame to GLOBAL");

      set_destination_local(goal.x, goal.y, goal.z, goal.psi);
      ROS_INFO("Sending goal %f %f %f %f", goal.x, goal.y, goal.z, goal.psi);
      // while (check_waypoint_reached(.3) == 0) {
      //   // polling at 50 Hz
      //   // ROS_INFO("Moving...");
      // }

      while (check_waypoint_reached(1.5) == 0) {
        // ROS_INFO("Waypoint not reached");
        ros::spinOnce();
        sleep(0.1);
      }
    } else {
      ROS_INFO("Changing the frame Failed ");
      return BT::NodeStatus::FAILURE;
    }
    // while (check_waypoint_reached(0.5) == 0) {
    //   ROS_INFO("Waypoint not reached");

    //   sleep(0.5);
    // }
    //
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle node_;
};
class TakeOff : public BT::AsyncActionNode {
public:
  TakeOff(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config) {}

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts() {
    return {BT::InputPort<Pose3D>("goal")};
  }
  // You must override the virtual function tick()
  virtual BT::NodeStatus tick() override {
    // Take the goal from the input port of the nodes
    Pose3D goal;
    if (!getInput<Pose3D>("goal", goal)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [goal]");
    }
    ROS_WARN("Taking Off to %f %f %f with heading %f", goal.x, goal.y, goal.z,
             goal.psi);

    // TBD: Check things to return running, success or failure
    // intitialize first waypoint of mission
    set_destination(0, 0, 0, goal.psi);
    for (int i = 0; i < 100; i++) {
      local_pos_pub.publish(waypoint_g);
      ros::spinOnce();
      ros::Duration(0.01).sleep();
    }
    // arming
    ROS_INFO("Arming drone");
    mavros_msgs::CommandBool arm_request;
    arm_request.request.value = true;
    while (!current_state_g.armed && !arm_request.response.success &&
           ros::ok()) {
      ros::Duration(.1).sleep();
      arming_client.call(arm_request);
      local_pos_pub.publish(waypoint_g);
    }
    if (arm_request.response.success) {
      ROS_INFO("Arming Successful");
    } else {
      ROS_WARN("Arming failed with %d", arm_request.response.success);

      return BT::NodeStatus::FAILURE;
    }

    // request takeoff

    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = goal.z;
    if (takeoff_client.call(srv_takeoff)) {
      sleep(3);
      ROS_INFO("takeoff sent %d", srv_takeoff.response.success);

    } else {
      ROS_ERROR("Failed Takeoff");
      return BT::NodeStatus::FAILURE;
    }
    sleep(2);
    ROS_INFO("Taking Off Now...");
    return BT::NodeStatus::SUCCESS;
  }
};

class Land : public BT::AsyncActionNode {
public:
  Land(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  // You must override the virtual function tick()
  virtual BT::NodeStatus tick() override {
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land) && srv_land.response.success) {
      ROS_INFO("land sent %d", srv_land.response.success);
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_ERROR("Landing failed");
      return BT::NodeStatus::FAILURE;
    }
  }
};
class RunPotter : public BT::AsyncActionNode {
public:
  RunPotter(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  // You must override the virtual function tick()
  virtual BT::NodeStatus tick() override {

    // At first we need to specify the reference frame

    ros::ServiceClient reference_client;
    mavros_msgs::SetMavFrame frame_srv;

    reference_client = node_.serviceClient<mavros_msgs::SetMavFrame>(
        "/mavros/setpoint_position/mav_frame");
    frame_srv.request.mav_frame = 8;
    reference_client.call(frame_srv);

    if (frame_srv.response.success) {
      ROS_INFO("Changed the frame to BODY NED");
      // Now we can run the whole thing
      ros::ServiceClient run_potter_client;
      std_srvs::Trigger srv;

      run_potter_client = node_.serviceClient<std_srvs::Trigger>("/potter/run");
      sleep(1);
      run_potter_client.call(srv);

      if (srv.response.success) {
        ROS_INFO("Crossing the gate");
        sleep(1);
        return BT::NodeStatus::SUCCESS;
      } else {
        ROS_INFO("Crossing Failed %s", srv.response.message.c_str());
        return BT::NodeStatus::FAILURE;
      }
    } else {
      ROS_INFO("Changing the frame Failed ");
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  ros::NodeHandle node_;
};

// FOREACH TEST
class ForEach : public ControlNode {
public:
  ForEach(const std::string &name, const NodeConfiguration &config)
      : ControlNode(name, config), current_index_(0) {
    getInput<std::vector<std::string>>("input_key", input_list_);
  }

  static PortsList providedPorts() {
    return {InputPort<std::vector<std::string>>("input_key"),
            OutputPort<std::string>("output_key")};
  }

  void halt() override {
    current_index_ = 0;
    ControlNode::halt();
  }

  NodeStatus tick() override {
    if (status() == NodeStatus::IDLE) {
      current_index_ = 0;
      setStatus(NodeStatus::RUNNING);
      std::cout << "ForEach: Starting iteration\n";
    }

    while (current_index_ < input_list_.size()) {
      std::string current_gate = input_list_[current_index_];
      std::cout << "ForEach: Current Gate: " << current_gate << std::endl;

      setOutput("output_key", current_gate);
      NodeStatus child_status = children_nodes_[0]->executeTick();

      if (child_status == NodeStatus::RUNNING) {
        return NodeStatus::RUNNING;
      } else if (child_status == NodeStatus::FAILURE) {
        std::cout << "ForEach: Child node failed\n";
        return NodeStatus::FAILURE;
      }

      current_index_++;
    }

    if (current_index_ >= input_list_.size()) {
      current_index_ = 0;
      setStatus(NodeStatus::IDLE); // Reset status to IDLE after completion
      std::cout << "ForEach: Iteration complete, returning SUCCESS\n";
      return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING; // This line should not be reached normally
  }

private:
  std::vector<std::string> input_list_;
  size_t current_index_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gates_robot2024_node");
  // ROS Stuff
  ros::NodeHandle nh("~");
  // initialize control publisher/subscribers
  init_publisher_subscriber(nh);
  vel_sub = nh.subscribe("/mavros/local_position/velocity_body", 10,
                         velocityCallback);

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  // create local reference frame
  initialize_local_frame();

  // BT Stuff
  std::string xml_filename;
  nh.param<std::string>(
      "file", xml_filename,
      "/home/user/simar/src/grvc_bt_ros/grvc_bt_ros/cfg/gates_bt.xml");
  ROS_INFO("Loading XML : %s", xml_filename.c_str());
  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;
  // The recommended way to create a Node is through inheritance.
  factory.registerNodeType<GoToGoal>("GoToGoal");
  factory.registerNodeType<WaitUntil>("WaitUntil");
  factory.registerNodeType<TakeOff>("TakeOff");
  factory.registerNodeType<RunPotter>("RunPotter");
  // factory.registerNodeType<ForEach>("ForEach");
  factory.registerNodeType<Land>("Land");

  //  factory.registerSimpleCondition("CheckBattery",
  //                                   [&](TreeNode &) { return CheckBattery();
  //                                   });

  factory.registerSimpleCondition("CheckOnAir",
                                  [&](TreeNode &) { return CheckOnAir(); });

  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  // NodeStatus status = NodeStatus::RUNNING;
  //  Keep on ticking until you get either a SUCCESS or FAILURE state
  // while (ros::ok() && status == NodeStatus::RUNNING) {
  //   status = tree.rootNode()->executeTick();
  //   // Sleep 100 milliseconds
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }
  tree.tickRootWhileRunning();

  return 0;
}
