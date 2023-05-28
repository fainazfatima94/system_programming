#include "dte3607_types.h"


namespace frb
{

  SpaceObjectBase::SpaceObject& SpaceObjectBase::spaceObject()
  {
    return m_object;
  }

  const SpaceObjectBase::SpaceObject& SpaceObjectBase::spaceObject() const
  {
    return m_object;
  }

  SpaceObjectBase::SpaceObject::ASFrameH SpaceObjectBase::pFrame() const
  {
    return spaceObject().pSpaceFrameParent();
  }

  SpaceObjectBase::SpaceObject::Frame SpaceObjectBase::vFrame() const
  {
    return spaceObject().vSpaceFrameParent();
  }

  SpaceObjectBase::Point3 SpaceObjectBase::globalFramePosition() const
  {
    return m_object.frameOriginParent();
  }

  rb_shapes::GodotShape::GodotShape(PhysicsServer::ShapeType p_shape_type)
    : m_shape_type{p_shape_type}
  {
  }

  rb_shapes::SphereRB::SphereRB(const ValueType& radius)
    : Shape(PhysicsServer::ShapeType::SHAPE_SPHERE), m_radius{radius}
  {
  }

  SpaceObjectBase::Point3 rb_shapes::SphereRB::point() const
  {
    return spaceObject().frameOriginParent();
  }

  SpaceObjectBase::ValueType rb_shapes::SphereRB::radius() const
  {
    return m_radius;
  }

  SpaceObjectBase::Point3 rb_shapes::SphereRB::centerOfMass() const
  {
    return point();
  }

  void rb_shapes::SphereRB::setDataFromGodot(const Variant& p_data)
  {
    real_t radius = p_data;
    m_radius      = radius;

    std::cout << "setting radius of " << m_shape_rid.get_id() << " to "
              << m_radius << '\n';
  }

  rb_shapes::PlaneRB::PlaneRB(const types::Vector3& n)
    : Shape(PhysicsServer::ShapeType::SHAPE_PLANE), m_n{n}
  {
  }

  SpaceObjectBase::Point3 rb_shapes::PlaneRB::point() const
  {
    return spaceObject().frameOriginParent();
  }

  SpaceObjectBase::Vector3 rb_shapes::PlaneRB::normal() const
  {
    return spaceObject().vSpaceFrameParent() * m_n;
  }

  SpaceObjectBase::Point3 rb_shapes::PlaneRB::centerOfMass() const
  {
    return point();
  }

  void rb_shapes::PlaneRB::setDataFromGodot(const Variant& p_data) {}

  SpaceObjectBase::ValueType RigidBodyPart::frictionCoef() const
  {
    return m_friction_coef;
  }

  void RigidBodyPart::setFrictionCoef(const ValueType& friction_coef)
  {
    m_friction_coef
      = std::clamp(friction_coef, frictionCoefMin, frictionCoefMax);
  }

  SpaceObjectBase::ValueType RigidBodyPart::mass() const { return m_mass; }

  void RigidBodyPart::setMass(const ValueType& mass) { m_mass = mass; }

  SpaceObjectBase::Point3 RigidBodyPart::centerOfMass() const
  {

    return m_shape->centerOfMass();   // Multiplied with frame or something
  }

  RigidBodyPart::Shape* RigidBodyPart::shape() { return m_shape; }

  RigidBody::RigidBody(const std::string& name) : m_name{name} {}

  RigidBody::Vector3 RigidBody::velocity() const { return m_velocity; }

  void RigidBody::addAcceleration(const types::Vector3& a) { m_velocity += a; }

  void RigidBody::setVelocity(const types::Vector3& v) { m_velocity = v; }

  RigidBody::Vector3 RigidBody::centerOfMass()
  {

    // Descr
    // Sum of com. of parts avgd. wrt. individual weight
    auto com = Vector3(0);
    for (auto const& p : m_parts) com += p->centerOfMass();
    return vFrame() * com;
  }

  RigidBody::Mode& RigidBody::mode() { return m_mode; }

  RigidBody::State& RigidBody::state() { return m_state; }

  size_t Fixture::noRigidBodies() const
  {
    return m_rigid_bodies.size();
  }

  Fixture::Forces Fixture::externalForces() const
  {
    return m_forces;
  }

  void Fixture::setGravity(Forces G) { m_forces = G; }

  types::Point3 Fixture::globalFramePosition(size_t rid) const
  {
    return m_rigid_bodies[rid]->globalFramePosition();
  }

  types::Vector3 Fixture::globalVelocity(size_t rid) const
  {
    return m_rigid_bodies[rid]->velocity();
  }

  RigidBody::Mode Fixture::mode(size_t rid) const
  {
    return m_rigid_bodies[rid]->mode();
  }

  void Fixture::translateParent(size_t rid, Vector3 lin_trajectory)
  {
    m_rigid_bodies[rid]->m_object.translateParent(lin_trajectory);
  }

  void Fixture::setVelocity(size_t rid, Vector3 velocity)
  {
    m_rigid_bodies[rid]->setVelocity(velocity);
  }

  void Fixture::addAcceleration(size_t rid, Vector3 accel)
  {
    m_rigid_bodies[rid]->addAcceleration(accel);
  }





}   // namespace frb
