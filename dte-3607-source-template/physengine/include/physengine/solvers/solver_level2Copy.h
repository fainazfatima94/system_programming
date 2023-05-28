#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../bits/types.h"
#include "../bits/types_oop.h"
#include "../bits/fixtures.h"
#include "../bits/concepts.h"
#include "../utils/type_conversion.h"
#include "../mechanics/compute_trajectory.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include <set>


namespace dte3607::physengine::solver_dev::level2{

using namespace dte3607::physengine::types;
using TP = types::HighResolutionTP;

//Computing cache data

struct Params{
    Vector3 F; // External forces (environment)
    types::Duration dt; // Time step (system)
};

struct CacheProcDataBlock{
    size_t      rb_id;
    Point3      in_P;   // Position
    Vector3     in_v;   // Velocity
    TP          in_tp;  // Timepoint
    Vector3     out_ds; // Trajectory
    Vector3     out_a;  // Acceleration
};

//Simulating objects over ∆t

struct SimProcDataBlock{
    Point3      in_p;   // Position
    Vector3     in_ds;  // Trajectory
    DtRep       ds_scalar;
    Point3      out_p;  // New Position
};

struct SphereGeomDataBlock{
    Point3      P = {};  // Position in global coordinate space
    ValueType   r = {};  // Radius
    Vector3     ds = {}; // Trajectory in global coordinate space
    TP          tp = {};
    size_t      cache_index = {};

    bool operator<(SphereGeomDataBlock const& rhs) const {
        return tp<rhs.tp;
    }
};

struct InfPlaneGeomDataBlock{
    Point3      P; // Position in global coordiante space
    Vector3     n; // Normal in global coordinate
    size_t      rb_id; // rigid body id
};

struct IntsectStatusDataBlock{
    bool        is_collision;  // Whether there was a collision or not
    TP          col_tp;        // Time point in frame (t0, t0 + ∆t)
};

struct IntersectDetProcDataBlock{
    SphereGeomDataBlock      in_sphere;  // Sphere data
    InfPlaneGeomDataBlock    in_plane;   // Inf plane data
    IntsectStatusDataBlock    out_status; // Intersection status data
};

using CacheProcData        = std::vector<CacheProcDataBlock>;
using SimProcData          = std::vector<SimProcDataBlock>;
using IntersectDetProcData = std::vector<IntersectDetProcDataBlock>;
//using SphereGeoData        = std::vector<SphereGeomDataBlock>;
//using InfPlaneGeoData      = std::vector<InfPlaneGeomDataBlock>;
//using IntsectStatusData    = std::vector<IntsectStatusDataBlock>;

namespace detail {

// Compute cache values
void computeCache( CacheProcData& data, Params const& params){
    auto const proc_kernel   = [&params](auto& data_block){
        auto const& [F, dt]  = params;
        auto& [rb_id, pos, vel, in_tp, out_ds, out_a] = data_block;
        auto traj_accel = mechanics::computeLinearTrajectory ( vel, F, dt);
        out_ds  = traj_accel.first;
        out_a   = traj_accel.second;
    };
    std::ranges::for_each(data, proc_kernel);
}

auto const ds_scalar = [](auto const &s, auto const &e, auto const &dt){
    auto es  = e - s;
    auto ess = utils::toDtScalar (es);
    auto dts = utils::toDtScalar (dt);
    return ess/dts;
};

//Compute the simulation

auto const sim_kernel = [](auto& data_block){
    auto& [p, ds, scalar, out_p] = data_block;
    out_p = p + ds * scalar;
};

void computeIntersection(IntersectDetProcDataBlock& data, HighResolutionTP t_0, Duration dt){
    auto const det_kernel = [&](auto& int_data_block){
        auto& [s, p, out_status] = int_data_block;
        if(auto col_tp = mechanics::implementation::computeIntersectionSpFp (s.tp, s.p, s.ds, p.p, p.n, t_0, dt)){
            out_status.is_collision = true;
            out_status.col_tp = col_tp.value();
        }
        else{
            out_status.is_collision = false;
        }
    };
    std::ranges::for_each(data, det_kernel); //why-----------------****
}

template<typename IntersectContainer_T>
         void makesortAndUnique(IntersectContainer_T& intersections){
         //sort
         std::sort (std::begin(intersections), std::end(intersections),
         [](auto const& intblock_a, auto const& intblock_b){
         return intblock_a.out_status.col_tp < intblock_b.out_status.col_tp;
});
// make unique
    using IntUniqueMemSet = std::set<SphereGeomDataBlock>;
    IntUniqueMemSet int_unique_set;
    std::erase_if(intersections, [&int_unique_set](auto const& obj){
        SphereGeomDataBlock const& s = obj.in_sphere;
        if(int_unique_set.contains(s)) return true;
        int_unique_set.insert(s);
        return false;
    });
    //reverse (smallest first)
    std::reverse (std::begin(intersections), std::end(intersections));
}
}

template <concepts::SolverFixtureLevel2 fixtures_T>
void solve([[maybe_unused]] fixtures_T& scenario,
[[maybe_unused]] types::NanoSeconds timestep)
{
   // Initialize

    auto O = scenario.nonFixedSphereRBs();
    auto F = scenario.fixedInfPlaneRBs();

    CacheProcData cache_data;

    // Time point initialized equal for all RBs
    auto const t_0 = types::HighResolutionClock::now ();

    // Generating our cache process data blocks
    for(auto o : O){
        auto const& pos = scenario.globalFramePosition(o);
        auto const& vel = scenario.rbSphereVelocity(o);
        CacheProcDataBlock db{o, pos, vel, t_0, {}, {}};
        cache_data.push_back(db);
    }

    //Environment params
    Params env_params{scenario.externalForces(), timestep};

    //Then we compute the cache properties
    detail::computeCache(cache_data, env_params);

    /// Procedure step
    /// Detect Collision
    /// Set up data blocks for detecting collision
    /// Loop through all spheres and planes and create intersection
    /// Data blocks

    IntersectDetProcData int_data;
    std::vector<SphereGeomDataBlock> s_g_blocks;
    std::vector<InfPlaneGeomDataBlock> p_g_blocks;
    size_t i = 0;

    for(auto o = O.begin(); o != O.end(); ++o, ++i){
        // Obtain p, ds, tp from cache data block
        // and r from sphere object

        auto const& sp = *o;
        auto const& sc = cache_data[i];

        SphereGeomDataBlock s_geom{sc.in_P, scenario.rbSphereRadius(sp), sc.out_ds, sc.in_tp, i};
        s_g_blocks.push_back(s_geom);
    }

    for(auto f : F){
        InfPlaneGeomDataBlock p_geom{scenario.globalFramePosition(f), scenario.rbPlaneNormal(f), f};

        p_g_blocks.push_back(p_geom);
    }

    // Nested loop to check intersection of every plane
    for(auto s : s_g_blocks){
        for(auto p : p_g_blocks){
            IntersectDetProcDataBlock i_data_block {s, p, {}};
            int_data.push_back(i_data_block);
        }
    }

     // Make a call to compute intersection
    detail::computeIntersection(int_data, t_0, timestep);
    //Now the intersection candidates reside in the int_data
    detail::makesortAndUnique(int_data);

    /// Then handle collision event
    /// 1. take first element from int_data
    /// 2. simulate sphere up to time of impact
    /// 3. compute intersection response (new velocity vector, new direction)
    /// 4. Update cache properties(recompute trajectory with update velocity)
    /// 5. Check for new collisions after the impact response

    while(not int_data.empty()){
        //1. take first collision element(at the back)
        auto const col = int_data.back();
        int_data.pop_back ();
        if(not col.out_status.is_collision)
            continue;

        //2. simulate sphere up to time of impact
        auto& c = cache_data.at(col.in_sphere.cache_index);
        auto const scalar = detail::ds_scalar(col.out_status.col_tp, c.in_tp, timestep);
        SimProcDataBlock s{c.in_P, c.out_ds, scalar, {}};
        detail::sim_kernel(s);
        scenario.setPosition(c.rb_id, s.out_p);

        //3. Compute impact response
        auto const new_v = mechanics::computeImpactResponseSphereFixedPlane(c.in_v, col.in_plane.n);
        scenario.setVelocity(c.rb_id, new_v);

        //4. Update cache properties (modify)
        c.in_v = new_v;
        c.in_tp = col.out_status.col_tp;
        c.in_P  = s.out_p;

        //Computing kernel (should be using the cache computing kernel lambda instead...
        auto const &[ext_forces, dt] = env_params;
        auto traj_accel = mechanics::computeLinearTrajectory(c.in_v, ext_forces, dt);
        c.out_ds = traj_accel.first;
        c.out_a  = traj_accel.second;

        // 5. Check for new collisions
        // - Implement here.
        // For the sphere in question, check collisions against other planes
        // Except the one just collided with
        IntersectDetProcDataBlock new_int_candidates;
        for(auto p : p_g_blocks){
            if(col.in_plane != p.rb_id){
                SphereGeomDataBlock sp_geom{c.in_P, col.in_sphere.r, c.out_ds, c.in_tp, col.in_sphere.cache_index
                };

                IntersectDetProcDataBlock i_data_block {sp_geom, p, {}};
                new_int_candidates.push_back(i_data_block);
            }
        }

        detail::computeIntersection (new_int_candidates, t_0, timestep);
        //Add new intersection candidates to the list of possible intersection
        for(auto candidate : new_int_candidates)
            if(candidate.out_status.is_collision)
               int_data.push_back (candidate);

        //Sort again, since we might have appended new candidates
        detail::makesortAndUnique(int_data);

    }

    //Create sim process data from cache data
    SimProcData sim_data;
    for(auto c : cache_data){
        auto const scalar = detail::ds_scalar(c.in_tp, t_0 + timestep, timestep);
        SimProcDataBlock s {c.in_P, c.out_ds, scalar, {}};
        sim_data.push_back(s);
    }

    detail::computeSimulation(sim_data);

    //Write result of simulation back to original data objects (in scenario)
    SimProcData::const_iterator sim_proc_data_ptr = sim_data.begin();
    CacheProcData::const_iterator cache_proc_data_ptr = cache_data.begin();

    for(auto o:O){
        scenario.setPosition(o, *sim_proc_data_ptr->out_p);
        ++sim_proc_data_ptr;
        ++cache_proc_data_ptr;
    }
}
}
#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
