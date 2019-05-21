
/**
 * @file glass_up_right_plan.cpp
 * @brief Example using Trajopt for constrained free space planning
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace tesseract;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
const std::string TRAJOPT_DESCRIPTION_PARAM =
    "trajopt_description"; /**< Default ROS parameter for trajopt description */

static bool plotting_ = false;
static bool write_to_file_ = false;
static int steps_ = 5;
static std::string method_ = "json";
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

int main(int argc, char** argv)
{
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

  bool success = env_->init(urdf_model_, srdf_model_);
  assert(success);

  std::vector<std::array<double, 3> > dims = {
    {0.4, 1.4, 0.02},
    {0.3, 0.02, 0.45},
    {0.3, 0.02, 0.45},
    {0.3, 0.52, 0.02},
    {1.1, 0.3, 0.02},
    {0.02, 0.2, 0.3},
    {0.02, 0.2, 0.3},
    {0.02, 0.2, 0.3},
    {0.42, 0.3, 0.02},
    {0.02, 0.1, 0.06},
    {0.02, 0.1, 0.06},
    {0.02, 0.1, 0.06},
    {0.42, 0.02, 0.06},
    {0.42, 0.02, 0.06},
    {0.42, 0.02, 0.34},
    {0.42, 0.02, 0.02}
  };
  std::vector<std::array<double, 3> > positions = {
    {0.75, -0.1, 0.7},
    {0.8, 0.25, 0.925},
    {0.8, -0.25, 0.925},
    {0.8, 0.0, 1.16},
    {0.4, -0.65, 0.7},
    {0.1, -0.7, 0.85},
    {0.3, -0.7, 0.85},
    {0.5, -0.7, 0.85},
    {0.3, -0.65, 0.71},
    {0.1, -0.55, 0.73},
    {0.3, -0.55, 0.73},
    {0.5, -0.55, 0.73},
    {0.3, -0.5, 0.73},
    {0.3, -0.59, 0.93},
    {0.3, -0.79, 0.87},
    {0.3, -0.7, 0.9}
  };


  // Create Plotting tool
  tesseract_ros::ROSBasicPlottingPtr plotter(new tesseract_ros::ROSBasicPlotting(env_));

  // Add sphere
  AttachableObjectPtr obj(new AttachableObject());

  auto dim_iter = dims.begin();
  auto pos_iter = positions.begin();
  size_t idx = 0;
  for (; dim_iter != dims.end(); ++dim_iter, ++pos_iter, ++idx) {
    std::shared_ptr<shapes::Shape> collision_object(new shapes::Box(dim_iter->at(0), dim_iter->at(1), dim_iter->at(2)));
    Eigen::Isometry3d coll_object_pose;

    coll_object_pose.setIdentity();
    coll_object_pose.translation() = Eigen::Vector3d((*pos_iter)[0], (*pos_iter)[1], (*pos_iter)[2]);

    obj->visual.shapes.push_back(collision_object);
    obj->visual.shape_poses.push_back(coll_object_pose);
    obj->collision.shapes.push_back(collision_object);
    obj->collision.shape_poses.push_back(coll_object_pose);
    obj->collision.collision_object_types.push_back(CollisionObjectType::UseShapeType);
  }
  obj->name = "sphere_attached"; 
  env_->addAttachableObject(obj);

  AttachedBodyInfo attached_body;
  attached_body.object_name = "sphere_attached";
  attached_body.parent_link_name = "base_link";
  attached_body.transform.setIdentity();
  //  attached_body.touch_links = {}; // This element enables the attached body
  //  to collide with other links

  env_->attachBody(attached_body);

  const auto environment_objects = env_->getAttachableObjects();
  for( const auto& elem : environment_objects)
  {
    std::cout << "***   " << elem.first << std::endl;
    elem.second->collision.shapes[0]->print();
  }

  // Get ROS Parameters
  pnh.param("plotting", plotting_, plotting_);
  pnh.param("write_to_file", write_to_file_, write_to_file_);
  pnh.param<std::string>("method", method_, method_);
  pnh.param<int>("steps", steps_, steps_);

  // Set the robot initial state
  std::unordered_map<std::string, double> ipos;
  ipos["torso_lift_joint"] =  0.00038880942889491296;
  ipos["arm_1_joint"] =  1.8412129654366185;
  ipos["arm_2_joint"] = -0.4905036166476348;
  ipos["arm_3_joint"] = -3.49;
  ipos["arm_4_joint"] = -1.5598026200058066;
  ipos["arm_5_joint"] = 1.5469586195777936;
  ipos["arm_6_joint"] = 0.812078221806306;
  ipos["arm_7_joint"] = -1.3109285867767095;
  env_->setState(ipos);

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

  bool found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), prob->GetInitTraj(), collisions);

  ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting_)
  {
    opt.addCallback(PlotCallback(*prob, plotter));
  }

  std::shared_ptr<std::ofstream> stream_ptr;
  if (write_to_file_)
  {
    // Create file write callback discarding any of the file's current contents
    stream_ptr.reset(new std::ofstream);
    std::string path = ros::package::getPath("trajopt") + "/scripts/glass_up_right_plan.csv";
    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    opt.addCallback(trajopt::WriteCallback(stream_ptr, prob));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  ros::Time tStart = ros::Time::now();
  opt.optimize();
  ROS_ERROR("planning time: %.3f", (ros::Time::now() - tStart).toSec());

  double d = 0;
  TrajArray traj = getTraj(opt.x(), prob->GetVars());
  for (unsigned i = 1; i < traj.rows(); ++i)
  {
    for (unsigned j = 0; j < traj.cols(); ++j)
    {
      d += std::abs(traj(i, j) - traj(i - 1, j));
    }
  }
  ROS_ERROR("trajectory norm: %.3f", d);

  if (plotting_)
  {
    plotter->clear();
  }
  if (write_to_file_)
  {
    stream_ptr->close();
    ROS_INFO("Data written to file. Evaluate using scripts in trajopt/scripts.");
  }
  collisions.clear();
  found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), prob->GetInitTraj(), collisions);

  ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}
