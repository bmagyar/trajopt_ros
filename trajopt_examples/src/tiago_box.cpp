
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

using namespace trajopt;
using namespace tesseract;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
const std::string TRAJOPT_DESCRIPTION_PARAM = "trajopt_description";

static bool plotting_ = false;
static int steps_ = 5;
static urdf::ModelInterfaceSharedPtr urdf_model_; /**< URDF Model */
static srdf::ModelSharedPtr srdf_model_;          /**< SRDF Model */
static tesseract_ros::KDLEnvPtr env_;             /**< Trajopt Basic Environment */

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

int main(int argc, char *argv[])
{
    std::cout << "Hello Tiago!" << std::endl;
    std::ofstream out_file;
    std::string filename = "/tmp/trajopr_results.csv";
    out_file.open("/tmp/trajopr_results.csv", std::ofstream::out | std::ofstream::trunc);
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
    //  attached_body.touch_links = {}; // This element enables the attached body
    //  to collide with other links

    env_->attachBody(attached_body);

    const auto environment_objects = env_->getAttachableObjects();
    for (const auto &elem : environment_objects)
    {
        std::cout << "***   " << elem.first << std::endl;
        elem.second->collision.shapes[0]->print();
    }

    // Get ROS Parameters
    pnh.param<int>("steps", steps_, steps_);

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
        if (plotting_)
        {
            opt.addCallback(PlotCallback(*prob, plotter));
        }

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

        if (plotting_)
        {
            plotter->clear();
        }
        collisions.clear();
        found = tesseract::continuousCollisionCheckTrajectory(*manager, *prob->GetEnv(), *prob->GetKin(),
                                                              prob->GetInitTraj(), collisions);

        ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));

        double duration = (ros::Time::now() - tStart).toSec();
        ROS_ERROR("planning time: %.3f", duration);

        // const std::vector<double> &joint_values = traj[0];
        // const auto &joint_names = env_->getJointNames();
        // ROS_ERROR_STREAM("Joint names: " << joint_names);
        // ROS_ERROR_STREAM("First state joint states " << joint_values);
        // env_->setState(joint_names, joint_values);
        // const Eigen::Isometry3d &tool_position = env_->getLinkTransform("arm_tool_link");
        // ROS_ERROR_STREAM("Start state FK is " << tool_position.translation());

        PrintResult(duration, 666, !found, out_file);
    }

    out_file.close();
    return 0;
}
