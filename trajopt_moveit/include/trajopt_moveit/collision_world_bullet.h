/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_WORLD_
#define MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_WORLD_

#include "trajopt_moveit/collision_robot_bullet.h"
#include <memory>
#include <moveit/macros/deprecation.h>

namespace collision_detection
{
class CollisionWorldBullet : public CollisionWorld
{
public:
  CollisionWorldBullet();
  explicit CollisionWorldBullet(const WorldPtr& world);
  CollisionWorldBullet(const CollisionWorldBullet& other, const WorldPtr& world);
  virtual ~CollisionWorldBullet();

  virtual void checkRobotCollision(const CollisionRequest& req,
                                   CollisionResult& res,
                                   const CollisionRobot& robot,
                                   const robot_state::RobotState& state) const;
  virtual void checkRobotCollision(const CollisionRequest& req,
                                   CollisionResult& res,
                                   const CollisionRobot& robot,
                                   const robot_state::RobotState& state,
                                   const AllowedCollisionMatrix& acm) const;
  virtual void checkRobotCollision(const CollisionRequest& req,
                                   CollisionResult& res,
                                   const CollisionRobot& robot,
                                   const robot_state::RobotState& state1,
                                   const robot_state::RobotState& state2) const;
  virtual void checkRobotCollision(const CollisionRequest& req,
                                   CollisionResult& res,
                                   const CollisionRobot& robot,
                                   const robot_state::RobotState& state1,
                                   const robot_state::RobotState& state2,
                                   const AllowedCollisionMatrix& acm) const;
  virtual void checkWorldCollision(const CollisionRequest& req,
                                   CollisionResult& res,
                                   const CollisionWorld& other_world) const;
  virtual void checkWorldCollision(const CollisionRequest& req,
                                   CollisionResult& res,
                                   const CollisionWorld& other_world,
                                   const AllowedCollisionMatrix& acm) const;

  MOVEIT_DEPRECATED
  virtual double distanceRobot(const CollisionRobot& robot,
                               const robot_state::RobotState& state,
                               bool verbose = false) const;

  MOVEIT_DEPRECATED
  virtual double distanceRobot(const CollisionRobot& robot,
                               const robot_state::RobotState& state,
                               const AllowedCollisionMatrix& acm,
                               bool verbose = false) const;

  MOVEIT_DEPRECATED
  virtual double distanceWorld(const CollisionWorld& world, bool verbose = false) const;

  MOVEIT_DEPRECATED
  virtual double distanceWorld(const CollisionWorld& world,
                               const AllowedCollisionMatrix& acm,
                               bool verbose = false) const;

  virtual void distanceRobot(const DistanceRequest& req,
                             DistanceResult& res,
                             const CollisionRobot& robot,
                             const robot_state::RobotState& state) const override;

  virtual void distanceRobot(const DistanceRequest& req,
                             DistanceResult& res,
                             const CollisionRobot& robot,
                             const robot_state::RobotState& state1,
                             const robot_state::RobotState& state2) const override;

  virtual void distanceWorld(const DistanceRequest& req,
                             DistanceResult& res,
                             const CollisionWorld& world) const override;

  virtual void setWorld(const WorldPtr& world);

protected:
  void checkWorldCollisionHelper(const CollisionRequest& req,
                                 CollisionResult& res,
                                 const CollisionWorld& other_world,
                                 const AllowedCollisionMatrix* acm) const;

  void checkRobotCollisionHelper(const CollisionRequest& req,
                                 CollisionResult& res,
                                 const CollisionRobot& robot,
                                 const robot_state::RobotState& state,
                                 const AllowedCollisionMatrix* acm) const;
  void checkRobotCollisionHelper(const CollisionRequest& req,
                                 CollisionResult& res,
                                 const CollisionRobot& robot,
                                 const robot_state::RobotState& state1,
                                 const robot_state::RobotState& state2,
                                 const AllowedCollisionMatrix* acm) const;

  void distanceRobotHelper(const DistanceRequest& req,
                           DistanceResult& res,
                           const CollisionRobot& robot,
                           const robot_state::RobotState& state) const;
  void distanceRobotHelper(const DistanceRequest& req,
                           DistanceResult& res,
                           const CollisionRobot& robot,
                           const robot_state::RobotState& state1,
                           const robot_state::RobotState& state2) const;

  void distanceWorldHelper(const DistanceRequest& req, DistanceResult& res, const CollisionWorld& world) const;

  void constructBulletObject(Link2Cow& collision_objects,
                             double contact_distance,
                             bool allow_static2static = false) const;
  void updateBulletObject(const std::string& id);

  Link2ConstCow m_link2cow;

private:
  void initialize();
  void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);
  World::ObserverHandle observer_handle_;
};
}

#endif
