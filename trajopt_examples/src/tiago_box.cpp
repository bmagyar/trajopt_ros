
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <trajopt_examples/rosconsole_extras.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

#include <numeric>

using namespace trajopt;
using namespace tesseract;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
const std::string TRAJOPT_DESCRIPTION_PARAM = "trajopt_description";

static urdf::ModelInterfaceSharedPtr urdf_model_; /**< URDF Model */
static srdf::ModelSharedPtr srdf_model_;          /**< SRDF Model */
static tesseract_ros::KDLEnvPtr env_;             /**< Trajopt Basic Environment */

std::vector<Eigen::Isometry3d> jointToCartesianTrajectory(const tesseract_ros::KDLEnvPtr &external_env,
                                                          const TrajArray &joint_traj)
{
    tesseract_ros::KDLEnv env = *external_env;
    const std::vector<std::string> joint_names = { "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
                                                   "arm_5_joint", "arm_6_joint", "arm_7_joint", "torso_lift_joint" };
    ROS_YELLOW_STREAM("Joint names: " << joint_names);

    std::vector<Eigen::Isometry3d> result;
    for (int i = 0; i < joint_traj.rows(); ++i)
    {
        const std::vector<double> joint_values = trajToDblVec(i, joint_traj);
        env.setState(joint_names, joint_values);
        const Eigen::Isometry3d tool_position = env.getLinkTransform("arm_tool_link");
        result.push_back(tool_position);
    }

    ROS_GREEN_STREAM("Start state joint states " << trajToDblVec(0, joint_traj));
    ROS_GREEN_STREAM("Start state FK is " << result[0].translation());
    ROS_BLUE_STREAM("Goal state joint states " << trajToDblVec(joint_traj.rows() - 1, joint_traj));
    ROS_BLUE_STREAM("Goal state FK is " << result[result.size() - 1].translation());
    return result;
}

TrajOptProbPtr jsonMethod()
{
    ros::NodeHandle nh;
    std::string trajopt_config;

    nh.getParam(TRAJOPT_DESCRIPTION_PARAM, trajopt_config);

    Json::Value root;
    Json::Reader reader;
    bool parse_success = reader.parse(trajopt_config.c_str(), root);
    if (!parse_success)
    {
        ROS_FATAL("Failed to load trajopt json file from ros parameter");
    }

    return ConstructProblem(root, env_);
}

void PrintHeader(std::ofstream &out)
{
    out << "Planning time (s), Trajectory length (rad), Trajectory length (m), "
           "Smoothness, Average min distance from joint limit (normalised), "
           "Success, Date\n";
}

void PrintResult(double planning_time, double smoothness, bool success, std::ofstream &out)
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    char *fmt = "%y.%m.%d-%H:%M:%S";
    out << planning_time << ", 0, 0," << smoothness << ", 0," << success << ", " << std::put_time(&tm, fmt) << "\n";
}

Eigen::Vector3d vAbs(const Eigen::Vector3d &v)
{
    return { std::abs(v.x()), std::abs(v.y()), std::abs(v.z()) };
}

double calculateSmoothness(const std::vector<Eigen::Isometry3d> &cartesian_traj)
{
    Eigen::Isometry3d first_pose = cartesian_traj[0];

    std::vector<double> pos_smoothness;

    for (size_t i = 0; i < cartesian_traj.size() - 2; ++i)
    {
        const Eigen::Isometry3d &pose0 = cartesian_traj[i];
        const Eigen::Isometry3d &pose1 = cartesian_traj[i + 1];
        const Eigen::Isometry3d &pose2 = cartesian_traj[i + 2];

        const Eigen::Vector3d f0 = vAbs(first_pose.translation() - pose0.translation());
        const Eigen::Vector3d f1 = vAbs(first_pose.translation() - pose1.translation());
        const Eigen::Vector3d f2 = vAbs(first_pose.translation() - pose2.translation());
        // f2.array() = f2.array().abs();
        const Eigen::Vector3d res = (f2 + f1 - 2.0 * f0);
        pos_smoothness.push_back(std::abs(res.x()));
        pos_smoothness.push_back(std::abs(res.y()));
        pos_smoothness.push_back(std::abs(res.z()));
    }

    const double smoothness =
        std::accumulate(pos_smoothness.begin(), pos_smoothness.end(), 0.0) / pos_smoothness.size();
    return smoothness;
}

int main(int argc, char *argv[])
{
    std::cout << "Hello Tiago!" << std::endl;
    std::ofstream out_file;
    const std::string filename = "/tmp/box_trajopt_arm_torso.csv";
    out_file.open(filename, std::ofstream::out | std::ofstream::trunc);
    if (out_file.is_open())
    {
        PrintHeader(out_file);
    }
    else
    {
        std::cout << "Could not open output file" << std::endl;
        return 0;
    }

    ros::init(argc, argv, "glass_up_right_plan");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    // Initial setup
    std::string urdf_xml_string, srdf_xml_string;
    nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
    urdf_model_ = urdf::parseURDF(urdf_xml_string);

    srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model_->initString(*urdf_model_, srdf_xml_string);
    env_ = tesseract_ros::KDLEnvPtr(new tesseract_ros::KDLEnv);
    assert(urdf_model_ != nullptr);
    assert(env_ != nullptr);

    const bool success = env_->init(urdf_model_, srdf_model_);
    assert(success);

    // Create Plotting tool
    tesseract_ros::ROSBasicPlottingPtr plotter(new tesseract_ros::ROSBasicPlotting(env_));

    // Add sphere
    AttachableObjectPtr obj(new AttachableObject());
    std::shared_ptr<shapes::Shape> collision_object(new shapes::Box(0.2, 0.2, 0.3));
    // std::shared_ptr<shapes::Shape> collision_object(new shapes::Sphere(0.15));
    Eigen::Isometry3d coll_object_pose;

    coll_object_pose.setIdentity();
    coll_object_pose.translation() = Eigen::Vector3d(0.5, 0, 0.55);

    obj->name = "sphere_attached";
    obj->visual.shapes.push_back(collision_object);
    obj->visual.shape_poses.push_back(coll_object_pose);
    obj->collision.shapes.push_back(collision_object);
    obj->collision.shape_poses.push_back(coll_object_pose);
    obj->collision.collision_object_types.push_back(CollisionObjectType::UseShapeType);

    env_->addAttachableObject(obj);

    AttachedBodyInfo attached_body;
    attached_body.object_name = "sphere_attached";
    attached_body.parent_link_name = "base_link";
    attached_body.transform.setIdentity();
    // attached_body.touch_links = {};  // This element enables the attached body
    //  to collide with other links

    env_->attachBody(attached_body);

    const auto environment_objects = env_->getAttachableObjects();
    for (const auto &elem : environment_objects)
    {
        std::cout << "***   " << elem.first << std::endl;
        elem.second->collision.shapes[0]->print();
    }

    plotter->plotScene();

    // Set Log Level
    util::gLogLevel = util::LevelInfo;

    // Setup Problem
    TrajOptProbPtr prob = jsonMethod();

    // Solve Trajectory
    ROS_INFO("glass upright plan example");

    std::vector<tesseract::ContactResultMap> collisions;
    ContinuousContactManagerBasePtr manager = prob->GetEnv()->getContinuousContactManager();
    manager->setActiveCollisionObjects(prob->GetKin()->getLinkNames());
    manager->setContactDistanceThreshold(0);

    bool found = tesseract::continuousCollisionCheckTrajectory(*manager, *prob->GetEnv(), *prob->GetKin(),
                                                               prob->GetInitTraj(), collisions);

    ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

    const size_t num_exp = 3;
    for (size_t i = 0; i < num_exp; ++i)
    {
        sco::BasicTrustRegionSQP opt(prob);

        opt.initialize(trajToDblVec(prob->GetInitTraj()));
        ros::Time tStart = ros::Time::now();
        opt.optimize();

        double d = 0;
        const TrajArray traj = getTraj(opt.x(), prob->GetVars());
        for (unsigned i = 1; i < traj.rows(); ++i)
        {
            for (unsigned j = 0; j < traj.cols(); ++j)
            {
                d += std::abs(traj(i, j) - traj(i - 1, j));
            }
        }
        ROS_ERROR("trajectory norm: %.3f", d);
        ROS_WARN_STREAM("Final trajectory: " << std::endl << traj);

        collisions.clear();
        found = tesseract::continuousCollisionCheckTrajectory(*manager, *prob->GetEnv(), *prob->GetKin(),
                                                              prob->GetInitTraj(), collisions);

        ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));

        double duration = (ros::Time::now() - tStart).toSec();
        ROS_ERROR("planning time: %.3f", duration);

        const auto cartesian_trajectory = jointToCartesianTrajectory(env_, traj);
        const double cartesian_smoothness = calculateSmoothness(cartesian_trajectory);

        PrintResult(duration, cartesian_smoothness, !found, out_file);
    }

    out_file.close();
    return 0;
}
