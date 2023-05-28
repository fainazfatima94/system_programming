#ifndef DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H
#define DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H


#include "types.h"

// stl
#include <concepts>


namespace dte3607::physengine::concepts
{



  namespace detail
  {

    /////////////////////////
    // Common concepts groups


    // Basic types
    template <typename Fixture_T>
    concept BasicTypesDefined = requires
    {
      typename Fixture_T::ValueType;
      typename Fixture_T::Point3;
      typename Fixture_T::Vector3;
    };

    // Query the number of rigid bodies
    template <typename Fixture_T>
    concept NoRigidBodiesQueryable = requires(Fixture_T const& fixture)
    {
      {
        fixture.noRigidBodies()
        } -> std::convertible_to<size_t>;
    };

    // Query the global frame of rigid body with ID
    template <typename Fixture_T>
    concept GlobalFramePositionQueryable
      = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.globalFramePosition(rid)
        } -> std::convertible_to<types::Point3>;
    };


    template <typename Fixture_T>
    concept GravitySettable
      = requires(Fixture_T fixture, types::ValueType value,
                 types::Vector3 vector)
    {
      {
        fixture.setGravity(vector)
        } -> std::same_as<void>;
    };



    // Query the global frame of rigid body with ID
    template <typename Fixture_T>
    concept ExternalForcesQueryable
      = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.externalForces()
        } -> std::convertible_to<types::Vector3>;
    };

    template <typename Fixture_T>
    concept NonFixedSphereRBsQueryable
      = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.nonFixedSphereRBs()
        } -> std::same_as<std::vector<size_t>>;
    };

    template <typename Fixture_T>
    concept FixedInfPlaneRBsQueryable
      = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.fixedInfPlaneRBs()
        } -> std::same_as<std::vector<size_t>>;
    };


    template <typename Fixture_T>
    concept RadiusOfSphereRBQueryable
      = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.rbSphereRadius(rid)
        } -> std::convertible_to<types::ValueType>;
    };


    template <typename Fixture_T>
    concept NormalOnInfPlaneRBQueryable
      = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.rbPlaneNormal(rid)
        } -> std::convertible_to<types::Vector3>;
    };

    template <typename Fixture_T>
    concept RBModeQueryable = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.rbMode(rid)
        } -> std::convertible_to<types::RBMode>;
    };

    template <typename Fixture_T>
    concept RBStateQueryable = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.rbState(rid)
        } -> std::convertible_to<types::RBState>;
    };

    template <typename Fixture_T>
    concept MaxFrictionCoefOfInfPlaneRBQueryable
      = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.rbPlaneMaxFrictionCoef(rid)
        } -> std::convertible_to<types::ValueType>;
    };


    template <typename Fixture_T>
    concept MaxFrictionCoefOfSphereRBQueryable
      = requires(Fixture_T const& fixture, size_t rid)
    {
      {
        fixture.rbSphereMaxFrictionCoef(rid)
        } -> std::convertible_to<types::ValueType>;
    };




  }   // namespace detail





  //////////////////////////////////////////
  // Solver Level 0 -- Scenario Requirements
  template <typename Fixture_T>
  concept SolverFixtureLevel0 =

    detail::BasicTypesDefined<Fixture_T> and

    detail::NoRigidBodiesQueryable<Fixture_T> and

    detail::GlobalFramePositionQueryable<Fixture_T> and

    detail::GravitySettable<Fixture_T> and

    // setters
    requires(Fixture_T fixture, types::ValueType value, types::Vector3 vector)
  {

    // Create sphere (radius, velocity, initial translation)
    {
      fixture.createSphere(value, vector, vector)
      } -> std::same_as<size_t>;
  };


  //////////////////////////////////////////
  // Solver Level 1 -- Scenario Requirements
  template <typename Fixture_T>
  concept SolverFixtureLevel1 = SolverFixtureLevel0<Fixture_T>;



  //////////////////////////////////////////
  // Solver Level 2 -- Scenario Requirements
  template <typename Fixture_T>
  concept SolverFixtureLevel2 =

    detail::BasicTypesDefined<Fixture_T> and

    detail::NoRigidBodiesQueryable<Fixture_T> and

    detail::GlobalFramePositionQueryable<Fixture_T> and

    detail::GravitySettable<Fixture_T> and

    detail::NonFixedSphereRBsQueryable<Fixture_T> and

    detail::FixedInfPlaneRBsQueryable<Fixture_T> and

    detail::RadiusOfSphereRBQueryable<Fixture_T> and

    detail::NormalOnInfPlaneRBQueryable<Fixture_T> and

    // setters
    requires(Fixture_T fixture, types::ValueType value, types::Vector3 vector)
  {

    // Create sphere (radius, velocity, initial translation)
    {
      fixture.createSphere(value, vector, vector)
      } -> std::same_as<size_t>;

    // Create fixed infinite plane (normal, initial translation)
    {
      fixture.createFixedInfPlane(vector, vector)
      } -> std::same_as<size_t>;
  }

  ;


  template <typename Fixture_T>
  concept SolverFixtureLevel3 =

    detail::BasicTypesDefined<Fixture_T> and

    detail::RBStateQueryable<Fixture_T> and


    // setters
    requires(Fixture_T fixture, types::ValueType value, types::Vector3 vector,
             types::RBState rb_state)
  {

    // Create sphere (radius, velocity, initial translation, state, friction
    // coefficient, mass)
    {
      fixture.createSphere(value, vector, vector, rb_state, value, value)
      } -> std::same_as<size_t>;

    // Create fixed infinite plane (normal, initial translation, friction
    // coefficient)
    {
      fixture.createFixedInfPlane(vector, vector, value)
      } -> std::same_as<size_t>;
  }

  ;




  template <typename Fixture_T>
  concept SolverFixtureLevel4 =

    detail::BasicTypesDefined<Fixture_T>

    // and ...

    ;




  // The requirements on the concept used for the sover API

  template <typename Fixture_T>
  concept SolverFixture = SolverFixtureLevel2<Fixture_T>;





}   // namespace dte3607::physengine::concepts


#endif   // DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H
