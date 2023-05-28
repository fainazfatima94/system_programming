#ifndef DTE3607_PHYSENGINE_FIXTURES_H
#define DTE3607_PHYSENGINE_FIXTURES_H

#include "../bits/types.h"


// stl
#include <variant>

namespace dte3607::physengine::fixtures
{

using namespace types;

  namespace rigid_bodies{

    struct Body {

        Point3 pos;
        Vector3 vel;
        Vector3 normal;
        ValueType radius;

    };

  }


  struct FixtureLevel1 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;




    /*** API concept required methods ***/



    // Global properties
    size_t noRigidBodies() const { return {}; }
    void   setGravity([[maybe_unused]] Forces G) {}

    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
        return{};
    }

    /*** Fixture unit-test setup API ***/
    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0})
    {
      return {};
    }

    /*** END API requirements ***/


  };



  struct FixtureLevel2 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

      std::vector<rigid_bodies::Body>  rigid_bodies;
      std::vector<size_t>              nonfixed_sphere_indexes;
      std::vector<size_t>              fixed_plane_indexes;

      Forces external_forces;
    /*** API concept required methods ***/

    // Environment
    void setGravity(Forces f) { external_forces = f;}
    Vector3 externalForces() const { return external_forces;}

    // RBs
    size_t              noRigidBodies() const { return rigid_bodies.size(); }
    std::vector<size_t> nonFixedSphereRBs() const { return nonfixed_sphere_indexes; }
    std::vector<size_t> fixedInfPlaneRBs() const { return fixed_plane_indexes; }

    ValueType rbSphereRadius([[maybe_unused]] size_t s_rid) const {
        return rigid_bodies.at(s_rid).radius; }
    Vector3   rbSpherePos([[maybe_unused]] size_t s_rid) const { return {}; }
    Vector3   rbSphereVelocity([[maybe_unused]] size_t s_rid) const {
        return rigid_bodies.at(s_rid).vel;; }
    Vector3   rbPlaneNormal([[maybe_unused]] size_t p_rid) const {
        return rigid_bodies.at(p_rid).normal;; }

    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
       return rigid_bodies.at(rid).pos;
    }

    inline void setPosition(size_t rid, Point3 p){
        rigid_bodies[rid].pos = p;
    }
    inline void setVelocity(size_t rid, Vector3 v){
        rigid_bodies[rid].vel = v;
    }


    /*** Fixture unit-test setup API ***/

    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0})
    {

        rigid_bodies::Body sphere {translation, velocity, {}, radius};
        rigid_bodies.push_back(sphere);
        auto const sphere_id = rigid_bodies.size() - 1;
        nonfixed_sphere_indexes.push_back(sphere_id);
        return sphere_id;
    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0})
    {
        rigid_bodies::Body plane{translation, {}, normal, {}};
        rigid_bodies.push_back(plane);
        auto const plane_id = rigid_bodies.size() - 1;
        fixed_plane_indexes.push_back(plane_id);
        return plane_id;
    }


    /*** END API requirements ***/
  };


  struct FixtureLevel3 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;


    /*** API concept required methods ***/

    // Environment
    void setGravity(Forces) {}

    ValueType rbPlaneMaxFrictionCoef() const { return {}; }
    ValueType rbSphereMaxFrictionCoef() const { return {}; }

    RBState rbState([[maybe_unused]] size_t rid) const { return {}; }


    /*** Fixture unit-test setup API ***/

    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0},
                        [[maybe_unused]] RBState initial_state = RBState::Free,
                        [[maybe_unused]] ValueType friction_coef = 0.,
                        [[maybe_unused]] ValueType mass          = 1.)
    {
        return{};
    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0},
                               [[maybe_unused]] ValueType friction_coef = 0.)
    {
      return {};
    }

    /*** END API requirements ***/
  };


  struct FixtureLevel4 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;



    /*** END API requirements ***/
  };


}   // namespace dte3607::physengine::fixtures


#endif   // DTE3607_PHYSENGINE_FIXTURES_H
