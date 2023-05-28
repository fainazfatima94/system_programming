#include "dte3607_entry_point.h"

// DTE-3607 PhysEngine
#include <physengine/api.h>

// godot -- physrv2 module
#include "broad_phase_bvh.h"
#include "broad_phase_octree.h"

// stl
#include <memory>
#include <iostream>
#include <string>
#include <optional>


namespace detail
{
  auto const updateLinearVelocity(auto& rb, const Vector3& v) {
    rb->setVelocity({v[0], v[1], v[2]});
  }

  auto const updateSofFromTransform = [](auto& so, Transform const& tr) {
    //    auto body_tr =  body->get_transform();
    auto b  = tr.get_basis();
    auto b0 = b[0];
    auto b1 = b[1];
    auto b2 = b[2];
    auto bo = tr.get_origin();

    //    std::cout << "Body before solve [" << body->get_id() << "]\n";
    //    std::cout << " Transform:\n";
    //    std::cout << "  o: " << bo[0] << ',' << bo[1] << ',' << bo[2] << '\n';
    //    std::cout << "  B0: " << b0[0] << ',' << b0[1] << ',' << b0[2] <<
    //    '\n'; std::cout << "  B1: " << b1[0] << ',' << b1[1] << ',' << b1[2]
    //    << '\n'; std::cout << "  B2: " << b2[0] << ',' << b2[1] << ',' <<
    //    b2[2] << '\n';


    // Axis e0
    so.m_frame(0, 0) = b0[0];
    so.m_frame(1, 0) = b0[1];
    so.m_frame(2, 0) = b0[2];
    so.m_frame(3, 0) = 0.0;

    // Axis e1
    so.m_frame(0, 1) = b1[0];
    so.m_frame(1, 1) = b1[1];
    so.m_frame(2, 1) = b1[2];
    so.m_frame(3, 1) = 0.0;

    // Axis e2
    so.m_frame(0, 2) = b2[0];
    so.m_frame(1, 2) = b2[1];
    so.m_frame(2, 2) = b2[2];
    so.m_frame(3, 2) = 0.0;

    // Origin
    so.m_frame(0, 3) = bo[0];
    so.m_frame(1, 3) = bo[1];
    so.m_frame(2, 3) = bo[2];
    so.m_frame(3, 3) = 1.0;
  };

  const auto setMode = [](auto* rb, auto* body) {
    switch (body->get_mode()) {
      case PhysicsServerSW::BodyMode::BODY_MODE_RIGID:
        rb->m_mode = frb::RigidBody::Mode::NonFixed;
        break;
      case PhysicsServerSW::BodyMode::BODY_MODE_STATIC:
        rb->m_mode = frb::RigidBody::Mode::Fixed;
        break;
      case PhysicsServerSW::BodyMode::BODY_MODE_CHARACTER:
        [[fallthrough]];
      case PhysicsServerSW::BodyMode::BODY_MODE_KINEMATIC:
        [[fallthrough]];
      default:
        rb->m_mode = frb::RigidBody::Mode::Hmm;
        break;
    }
  };

  auto const getRBFromBackend
    = [](auto& rbs, RID p_body) -> std::optional<frb::RigidBody*> {
    // Get p_body in backend
    auto rb_itr = std::find_if(rbs.begin(), rbs.end(), [p_body](auto& rb) {
      if (rb->m_body_rid == p_body) return true;
      return false;
    });

    // Return if p_body doesn't exist in backend -- not supported
    if (rb_itr == rbs.end()) return {};


    // Backend RB
    return rb_itr->get();
  };

  auto const getRBShapeFromBackend
    = [](auto& p_shapes, RID p_shape) -> std::optional<frb::rb_shapes::Shape*> {
    // Get p_body in backend
    auto itr
      = std::find_if(p_shapes.begin(), p_shapes.end(), [p_shape](auto& shape) {
          if (shape->m_shape_rid == p_shape) return true;
          return false;
        });

    // Return if p_body doesn't exist in backend -- not supported
    if (itr == p_shapes.end()) return {};


    // Backend RB
    return itr->get();
  };
}   // namespace detail


/// [BEGIN] Keep this function here as this is highly modified
//void PhysicsServerSW::step(real_t p_step)
//{
//#ifndef _3D_DISABLED

//  if (!active) {
//    return;
//  }

//  _update_shapes();

//  last_step                                 = p_step;
//  PhysicsDirectBodyStateSW::singleton->step = p_step;


//  //	island_count = 0;
//  //	active_objects = 0;
//  //	collision_pairs = 0;
//  //	for (Set<const SpaceSW *>::Element *E = active_spaces.front(); E; E =
//  // E->next()) { 		stepper->step((SpaceSW *)E->get(), p_step, iterations);
//  //		island_count += E->get()->get_island_count();
//  //		active_objects += E->get()->get_active_objects();
//  //		collision_pairs += E->get()->get_collision_pairs();
//  //	}

//#endif
//}
void PhysicsServerSW::step(real_t p_step) {
#ifndef _3D_DISABLED

  if (!active) {
    return;
  }

  _update_shapes();

  last_step = p_step;
  PhysicsDirectBodyStateSW::singleton->step = p_step;

  m_entry_point->step(p_step);

//	island_count = 0;
//	active_objects = 0;
//	collision_pairs = 0;
//	for (Set<const SpaceSW *>::Element *E = active_spaces.front(); E; E = E->next()) {
//		stepper->step((SpaceSW *)E->get(), p_step, iterations);
//		island_count += E->get()->get_island_count();
//		active_objects += E->get()->get_active_objects();
//		collision_pairs += E->get()->get_collision_pairs();
//  }
#endif
}
/// [END]

void Dte3607EntryPoint::step(real_t p_step)
{
  // Convert to chrono time
  auto timestep = dte3607::physengine::types::SecondsD(p_step);
  auto timestep_ns
    = std::chrono::duration_cast<dte3607::physengine::types::NanoSeconds>(
      timestep);

  dte3607::physengine::api::solve(m_fixture, timestep_ns);

  bodyUpdateFrontend();
}


void Dte3607EntryPoint::bodyUpdateFrontend(RID /*p_body*/)
{
  for (auto const& rb : m_fixture.m_rigid_bodies) {

    auto rid = rb->m_body_rid;

    // Set frontend transformations
    auto const sof = rb->pFrame();
    auto const tr  = Transform(
      // Axis e0
      sof(0, 0), sof(1, 0), sof(2, 0),
      // Axis e1
      sof(0, 1), sof(1, 1), sof(2, 1),
      // Axis e2
      sof(0, 2), sof(1, 2), sof(2, 2),
      // Origin
      sof(0, 3), sof(1, 3), sof(2, 3));

    auto b_state = m_server.body_get_direct_state(rid);
    b_state->set_transform(tr);

    // Trigger Godot fronten update for a given object
    BodySW* body = m_server.body_owner.get(rid);
    body->triggerSomethingWhichTriggersRedraw();
  }
}


void Dte3607EntryPoint::body_create(RID rid)
{
  BodySW* body = m_server.body_owner.get(rid);

  m_fixture.m_rigid_bodies.emplace_back(std::make_unique<frb::RigidBody>());
  auto& rb       = m_fixture.m_rigid_bodies.back();
  rb->m_body_rid = rid;

  // Set mode in backend
  detail::setMode(rb.get(), body);
}

void Dte3607EntryPoint::body_set_mode(RID                     p_body,
                                      PhysicsServer::BodyMode p_mode)
{
  // Get RB from backend
  auto rb_opt = detail::getRBFromBackend(m_fixture.m_rigid_bodies, p_body);
  if (not rb_opt) return;
  auto* rb = rb_opt.value();

  // Get body from server
  BodySW* body = m_server.body_owner.get(p_body);

  // Set mode in backend
  detail::setMode(rb, body);
}




void Dte3607EntryPoint::body_add_shape(RID p_body, RID p_shape,
                                       const Transform& p_transform,
                                       bool             p_disabled)
{
  // Get RB from backend
  auto rb_opt = detail::getRBFromBackend(m_fixture.m_rigid_bodies, p_body);
  if (not rb_opt) return;
  auto* rb = rb_opt.value();

  // Get Shape from backend
  auto shape_opt
    = detail::getRBShapeFromBackend(m_fixture.m_rb_shapes, p_shape);
  if (not shape_opt) return;
  auto* shape = shape_opt.value();


  rb->m_parts.emplace_back(std::make_unique<frb::RigidBodyPart>());
  auto& rbp = rb->m_parts.back();

  rbp->m_shape = shape;
  detail::updateSofFromTransform(rbp->spaceObject(), p_transform);
  rbp->setFrictionCoef(1.0);
}

void Dte3607EntryPoint::body_set_state(RID                      p_body,
                                       PhysicsServer::BodyState p_state,
                                       const Variant&           p_variant)
{
  // Get RB from backend
  auto rb_opt = detail::getRBFromBackend(m_fixture.m_rigid_bodies, p_body);
  if (not rb_opt) return;
  auto* rb = rb_opt.value();

  switch (p_state) {
  case PhysicsServer::BodyState::BODY_STATE_TRANSFORM: {
    // Update Backend Sof
    detail::updateSofFromTransform(rb->spaceObject(), p_variant);
  } break;
  case PhysicsServer::BodyState::BODY_STATE_LINEAR_VELOCITY: {
    // Update Backend linear velocity
    detail::updateLinearVelocity(rb, p_variant);
  }break;
  case PhysicsServer::BODY_STATE_ANGULAR_VELOCITY:
  case PhysicsServer::BODY_STATE_SLEEPING:
  case PhysicsServer::BODY_STATE_CAN_SLEEP:
    break;
  }
}

void Dte3607EntryPoint::shape_create(RID p_shape)
{
  ShapeSW* shape = m_server.shape_owner.get(p_shape);

  switch (shape->get_type()) {
    case PhysicsServerSW::ShapeType::SHAPE_PLANE: {
      m_fixture.m_rb_shapes.emplace_back(
        std::make_unique<frb::rb_shapes::PlaneRB>());
      auto& rbp        = m_fixture.m_rb_shapes.back();
      rbp->m_shape_rid = p_shape;
    } break;

    case PhysicsServerSW::ShapeType::SHAPE_SPHERE: {
      m_fixture.m_rb_shapes.emplace_back(
        std::make_unique<frb::rb_shapes::SphereRB>());
      auto& rbp        = m_fixture.m_rb_shapes.back();
      rbp->m_shape_rid = p_shape;
    } break;

    default:
      break;
  }
}

void Dte3607EntryPoint::shape_set_data(RID p_shape, Variant const& p_data)
{
  // Get RB from backend
  auto shape_opt
    = detail::getRBShapeFromBackend(m_fixture.m_rb_shapes, p_shape);
  if (not shape_opt) return;
  auto* shape = shape_opt.value();

  shape->setDataFromGodot(p_data);
}

void BodySW::triggerSomethingWhichTriggersRedraw()
{
  if (fi_callback) {
    get_space()->body_add_to_state_query_list(&direct_state_query_list);
  }
}
