#include <ros/ros.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <gnc_functions.hpp>

using namespace BT;
// Here we will define the dummy functions for actions and conditions
//
// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery() {
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}
float visibility = 0;
float error = 0;
ros::Subscriber statusSub;
void status_cb(const std_msgs::Float32MultiArrayConstPtr &msg) {

    visibility = msg->data[0] + msg->data[1];
    error = msg->data[2];
    //ROS_INFO("visibility %f",visibility);
  }
BT::NodeStatus PlatformVisible() {
  // If the % of the platform visible is greater than 1
  ros::spinOnce();
  if (visibility <= 0.002) {
    return BT::NodeStatus::RUNNING;
  } else {
    printf("Platform Visible\n");
    return BT::NodeStatus::SUCCESS;
  }
}
BT::NodeStatus CheckError() {
  // If the % of the platform visible is greater than 1
  ros::spinOnce();
  if (error >= 15 || error == 0) {
    return BT::NodeStatus::RUNNING;
  } else {
    printf("Acceptable error\n");
    error = 0;
    return BT::NodeStatus::SUCCESS;
  }
}
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
    set_destination(goal.x, goal.y, goal.z, goal.psi);
    ROS_INFO("Sending goal %f %f %f %f", goal.x, goal.y, goal.z, goal.psi);
    // while (check_waypoint_reached(.3) == 0) {
    //   // polling at 50 Hz
    //   // ROS_INFO("Moving...");
    // }

    while (check_waypoint_reached(0.3) == 0) {
      //ROS_INFO("Waypoint not reached");
      ros::spinOnce();
      sleep(1);
    }
    // while (check_waypoint_reached(0.5) == 0) {
    //   ROS_INFO("Waypoint not reached");

    //   sleep(0.5);
    // }
    //
    return BT::NodeStatus::SUCCESS;
  }
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

class Approach : public BT::AsyncActionNode {
public:
  Approach(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {};
  }
  // You must override the virtual function tick()
  virtual BT::NodeStatus tick() override {
 
    ros::ServiceClient approach_client;
    std_srvs::Trigger srv;

    approach_client = node_.serviceClient<std_srvs::Trigger>("/landing/mode/approach");
    approach_client.call(srv);
  
  if (srv.response.success) {
    ROS_INFO("Approaching Successfully");
    return BT::NodeStatus::SUCCESS;
  } else {
    ROS_INFO("Approach Failed %s", srv.response.message.c_str());
    return BT::NodeStatus::FAILURE;
  }
    
  }

private:
  ros::NodeHandle node_;
};

class Fine : public BT::AsyncActionNode {
public:
  Fine(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {};
  }
  // You must override the virtual function tick()
  virtual BT::NodeStatus tick() override {
 
    ros::ServiceClient fine_client;
    std_srvs::Trigger srv;

    fine_client = node_.serviceClient<std_srvs::Trigger>("/landing/mode/fine");
    fine_client.call(srv);
  
  if (srv.response.success) {
    ROS_INFO("fining Successfully");
    return BT::NodeStatus::SUCCESS;
  } else {
    ROS_INFO("fine Failed %s", srv.response.message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  }
private:
  ros::NodeHandle node_;
  };
class Land : public BT::AsyncActionNode {
public:
  Land(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {};
  }
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
int main(int argc, char **argv) {
  ros::init(argc, argv, "test_bt");
  // ROS Stuff
  ros::NodeHandle nh("~");
  // initialize control publisher/subscribers
  init_publisher_subscriber(nh);
  statusSub = nh.subscribe<std_msgs::Float32MultiArray>("/landing/status", 10, status_cb);

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  // create local reference frame
  initialize_local_frame();

  // BT Stuff
  std::string xml_filename;
  nh.param<std::string>(
      "file", xml_filename,
      "/home/user/simar/src/grvc_bt_ros/grvc_bt_ros/cfg/dummy_bt.xml");
  ROS_INFO("Loading XML : %s", xml_filename.c_str());
  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;
  // The recommended way to create a Node is through inheritance.
  factory.registerNodeType<GoToGoal>("GoToGoal");
  factory.registerNodeType<TakeOff>("TakeOff");
  factory.registerNodeType<Approach>("Approach");
  factory.registerNodeType<Fine>("Fine");
  factory.registerNodeType<Land>("Land");


  factory.registerSimpleCondition("CheckBattery",
                                  [&](TreeNode &) { return CheckBattery(); });
  factory.registerSimpleCondition(
      "PlatformVisible", [&](TreeNode &) { return PlatformVisible(); });
  factory.registerSimpleCondition(
      "CheckError", [&](TreeNode &) { return CheckError(); });

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
