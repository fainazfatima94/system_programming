#include "register_types.h"

#include "physics_server_sw.h"

// godot
#include "core/class_db.h"
#include "core/engine.h"


#ifndef _3D_DISABLED
PhysicsServer* _createDte3607PhyServCallback()
{
  return memnew(PhysicsServerSW);
}
#endif

void register_dte3607physrv_types()
{
#ifndef _3D_DISABLED
  PhysicsServerManager::register_server("DTE-3607 PhysEngine 2021",
      &_createDte3607PhyServCallback);
#endif
}

void unregister_dte3607physrv_types() {}
