#ifndef DTE3607_PHYSENGINE_TYPES_OOP_H
#define DTE3607_PHYSENGINE_TYPES_OOP_H

#include "types.h"

namespace dte3607::physengine::types_ext
{
  namespace rb_oop
  {

    struct SpaceObjectBase {

      // Concept requirements
      using ValueType   = types::Point3::ElementType;
      using Point3      = types::Point3;
      using Point3H     = types::Point3H;
      using Vector3     = types::Vector3;
      using Vector3H    = types::Vector3H;
      using SpaceObject = types::ProjectiveSpaceObject;
      using Timepoint   = types::HighResolutionTP;


      // Common properties
      SpaceObject m_object;

      // Lin. alg. prop. access
      virtual SpaceObject&       spaceObject();
      virtual SpaceObject const& spaceObject() const;

      typename SpaceObject::ASFrameH pFrame() const;

      typename SpaceObject::Frame vFrame() const;

      Point3 globalFramePosition() const;
    };


    namespace rb_shapes
    {
      enum class ShapeType { Sphere, InfPlane };

      struct Shape : SpaceObjectBase {
        Shape(ShapeType p_shape_type);
        ShapeType m_shape_type;
      };


      struct Sphere : Shape {
        Sphere(Vector3::ElementType const& radius = 1.);

        // Sphere prop. access
        Point3    point() const;
        ValueType radius() const;

      private:
        ValueType m_radius{1.0};
      };

      struct InfPlane : Shape {
        InfPlane(Vector3 const& n = {.0, 1., .0});

        // Plane prop. access
        Point3  point() const;
        Vector3 normal() const;

      private:
        Vector3 m_n{0, 1, 0};
      };

    }   // namespace rb_shapes



    struct RigidBodyPart : SpaceObjectBase {

      using Shape = rb_shapes::Shape;

      // Class type constants
      static constexpr ValueType frictionCoefMin{0.0};
      static constexpr ValueType frictionCoefMax{1.0};

      ValueType frictionCoef() const;
      void      setFrictionCoef(ValueType const& friction_coef);

      ValueType m_mass{1.0};

      Shape* shape();
      Shape* m_shape;

    private:
      ValueType m_friction_coef{0.0};   // 0 == no friction
    };

    struct RigidBody : SpaceObjectBase {

      RigidBody(const std::string& name = "");

      // Concept requirements
      using ValueType        = types::Point3::ElementType;
      using Point3           = types::Point3;
      using Point3H          = types::Point3H;
      using Vector3          = types::Vector3;
      using Vector3H         = types::Vector3H;
      using SpaceObjectFrame = types::ProjectiveSpaceObject;
      using Timepoint        = types::HighResolutionTP;
      using Mode             = types::RBMode;
      using State            = types::RBState;


      // Mechanics property access
      Vector3 velocity() const;

      /** a; in the "parent" spacial frame */
      void addAcceleration(types::Vector3 const& a);

      /** v; in the "parent" spacial frame */
      void setVelocity(types::Vector3 const& v);


      // Mode
      Mode mode() const;
      void setMode(Mode p_mode);

      // States
      State state() const;
      void  setState(State p_state);

    private:
      Timepoint m_timepoint;
      Vector3   m_velocity{0, 0, 0};
      Mode      m_mode{Mode::NonFixed};
      State     m_state{State::Free};


    public:
      std::string m_name;

      using Part    = RigidBodyPart;
      using PartPtr = std::unique_ptr<RigidBodyPart>;
      using Parts   = std::vector<PartPtr>;

      Parts m_parts;
    };

  }   // namespace rb_oop

  struct FixtureOOP {

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
    using RigidBody    = rb_oop::RigidBody;
    using RigidBodyPtr = std::unique_ptr<RigidBody>;
    using RigidBodies  = std::vector<RigidBodyPtr>;

    using RBShape    = rb_oop::rb_shapes::Shape;
    using RBShapePtr = std::unique_ptr<RBShape>;
    using RBShapes   = std::vector<RBShapePtr>;

    using RBPart  = rb_oop::RigidBodyPart;
    using RBMode  = RigidBody::Mode;
    using RBState = RigidBody::State;


    /*** API concept required methods ***/

    // Global properties
    size_t noRigidBodies() const;
    Forces externalForces() const;

    void setGravity(Forces);

    // RB properties
    types::Point3  globalFramePosition(size_t rid) const;
    types::Vector3 globalVelocity(size_t rid) const;

    // Modes and states
    RigidBody::Mode mode(size_t rid) const;

    // Transform
    void translateParent(size_t rid, Vector3 lin_trajectory);
    void setVelocity(size_t rid, Vector3 velocity);
    void addAcceleration(size_t rid, Vector3 accel);


    /*** END API requirements ***/


    /*** Members ***/
    RigidBodies m_rigid_bodies;
    RBShapes    m_rb_shapes;

    Forces m_forces;


    /*** Custom API methods ***/

    size_t createSphere(ValueType radius = 1., Vector3 velocity = {0, 0, 0},
                        Vector3   translation   = {0, 0, 0},
                        ValueType friction_coef = 1.);

    size_t createFixedInfPlane(Vector3   normal        = {0, 0, 1},
                               Vector3   translation   = {0, 0, 0},
                               ValueType friction_coef = 1.);
  };




  /****
   * Implementations
   */




  namespace rb_oop
  {

    inline SpaceObjectBase::SpaceObject& SpaceObjectBase::spaceObject()
    {
      return m_object;
    }

    inline const SpaceObjectBase::SpaceObject&
    SpaceObjectBase::spaceObject() const
    {
      return m_object;
    }

    inline SpaceObjectBase::SpaceObject::ASFrameH SpaceObjectBase::pFrame() const
    {
      return spaceObject().pSpaceFrameParent();
    }

    inline SpaceObjectBase::SpaceObject::Frame SpaceObjectBase::vFrame() const
    {
      return spaceObject().vSpaceFrameParent();
    }

    inline SpaceObjectBase::Point3 SpaceObjectBase::globalFramePosition() const
    {
      return m_object.frameOriginParent();
    }

    inline SpaceObjectBase::ValueType RigidBodyPart::frictionCoef() const
    {
      return m_friction_coef;
    }

    inline void RigidBodyPart::setFrictionCoef(const ValueType& friction_coef)
    {
      m_friction_coef
        = std::clamp(friction_coef, frictionCoefMin, frictionCoefMax);
    }

    inline RigidBodyPart::Shape* RigidBodyPart::shape() { return m_shape; }

    inline RigidBody::RigidBody(const std::string& name) : m_name{name} {}

    inline RigidBody::Vector3 RigidBody::velocity() const { return m_velocity; }

    inline void RigidBody::addAcceleration(const types::Vector3& a)
    {
      m_velocity += a;
    }

    inline void RigidBody::setVelocity(const types::Vector3& v)
    {
      m_velocity = v;
    }

    inline RigidBody::Mode RigidBody::mode() const { return m_mode; }

    inline void RigidBody::setMode(Mode p_mode) { m_mode = p_mode; }

    inline RigidBody::State RigidBody::state() const { return m_state; }

    inline void RigidBody::setState(State p_state) { m_state = p_state; }


    namespace rb_shapes
    {
      inline Shape::Shape(ShapeType p_shape_type) : m_shape_type{p_shape_type}
      {
      }

      inline Sphere::Sphere(const Vector3::ElementType& radius)
        : Shape(ShapeType::Sphere), m_radius{radius}
      {
      }

      inline SpaceObjectBase::Point3 Sphere::point() const
      {
        return spaceObject().frameOriginParent();
      }

      inline SpaceObjectBase::ValueType Sphere::radius() const
      {
        return m_radius;
      }

      inline InfPlane::InfPlane(const Vector3& n)
        : Shape(ShapeType::InfPlane), m_n{n}
      {
      }

      inline SpaceObjectBase::Point3 InfPlane::point() const
      {
        return spaceObject().frameOriginParent();
      }

      inline SpaceObjectBase::Vector3 InfPlane::normal() const
      {
        return spaceObject().vSpaceFrameParent() * m_n;
      }

    }   // namespace rb_shapes



  }   // namespace rb_oop





  inline size_t FixtureOOP::noRigidBodies() const
  {
    return m_rigid_bodies.size();
  }

  inline FixtureOOP::Forces FixtureOOP::externalForces() const
  {
    return m_forces;
  }

  inline void FixtureOOP::setGravity(Forces G) { m_forces = G; }

  inline FixtureOOP::Point3 FixtureOOP::globalFramePosition(size_t rid) const
  {

    return m_rigid_bodies[rid]->globalFramePosition();
  }

  inline FixtureOOP::Vector3 FixtureOOP::globalVelocity(size_t rid) const
  {

    return m_rigid_bodies[rid]->velocity();
  }

  inline FixtureOOP::RBMode FixtureOOP::mode(size_t rid) const
  {

    return m_rigid_bodies[rid]->mode();
  }

  inline void FixtureOOP::translateParent(size_t rid, Vector3 lin_trajectory)
  {
    m_rigid_bodies[rid]->m_object.translateParent(lin_trajectory);
  }

  inline void FixtureOOP::addAcceleration(size_t rid, Vector3 accel)
  {
    m_rigid_bodies[rid]->addAcceleration(accel);
  }

  inline void FixtureOOP::setVelocity(size_t rid, Vector3 velocity)
  {
    m_rigid_bodies[rid]->setVelocity(velocity);
  }

  inline size_t FixtureOOP::createSphere(ValueType radius, Vector3 velocity,
                                         Vector3   translation,
                                         ValueType friction_coef)
  {
    // Create Rigid body
    m_rigid_bodies.emplace_back(std::make_unique<RigidBody>());
    auto& rb = m_rigid_bodies.back();
    auto const rbi = m_rigid_bodies.size() - 1;
    rb->m_object.translateParent(translation);
    rb->setMode(RigidBody::Mode::NonFixed);
    rb->setVelocity(velocity);

    // Create Shape
    m_rb_shapes.emplace_back(
      std::make_unique<rb_oop::rb_shapes::Sphere>(radius));
    auto* sphere = m_rb_shapes.back().get();

    // Create RB body part and add sphere as shape to body part
    rb->m_parts.emplace_back(std::make_unique<RBPart>());
    auto& rbp    = rb->m_parts.back();
    rbp->m_shape = sphere;
    rbp->setFrictionCoef(friction_coef);

    return rbi;
  }

  inline size_t FixtureOOP::createFixedInfPlane(Vector3   normal,
                                                Vector3   translation,
                                                ValueType friction_coef)
  {
    // Create Rigid body
    m_rigid_bodies.emplace_back(std::make_unique<RigidBody>());
    auto& rb = m_rigid_bodies.back();
    auto const rbi = m_rigid_bodies.size() - 1;
    rb->m_object.translateParent(translation);
    rb->setMode(RigidBody::Mode::Fixed);

    // Create Shape
    m_rb_shapes.emplace_back(
      std::make_unique<rb_oop::rb_shapes::InfPlane>(normal));
    auto* plane = m_rb_shapes.back().get();

    // Create RB body part and add sphere as shape to body part
    rb->m_parts.emplace_back(std::make_unique<RBPart>());
    auto& rbp    = rb->m_parts.back();
    rbp->m_shape = plane;
    rbp->setFrictionCoef(friction_coef);

    return rbi;
  }

}   // namespace dte3607::physengine::types_ext

#endif   // DTE3607_PHYSENGINE_TYPES_OOP_H
