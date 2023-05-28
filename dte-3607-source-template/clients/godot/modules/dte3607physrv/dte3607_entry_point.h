#ifndef DTE3607_ENTRY_POINT_H
#define DTE3607_ENTRY_POINT_H

#include "physics_server_sw.h"
#include "dte3607_types.h"


class Dte3607EntryPoint {
public:
  Dte3607EntryPoint(PhysicsServerSW& p_server) : m_server{p_server}
  {

    m_fixture.m_forces = {0.0, -9.8, 0.0};
  }

  frb::Fixture m_fixture;

  // Frontend integration functions (force godot 'trigger' update)
  void bodyUpdateFrontend(RID p_body = {});

  // Server relay integration functions
  // Should have same names as in server -- for easy xref
  void step(real_t p_step);
  void body_create(RID rid);
  void body_set_mode(RID p_body, PhysicsServerSW::BodyMode p_mode);
  void body_add_shape(RID p_body, RID p_shape, const Transform& p_transform,
                      bool p_disabled);
  void body_set_state(RID p_body, PhysicsServerSW::BodyState p_state,
                      Variant const& p_variant);

  void shape_create(RID rid);
  void shape_set_data(RID p_shape, Variant const& p_data);

private:
  PhysicsServerSW& m_server;
};


#endif   // DTE3607_ENTRY_POINT_H
