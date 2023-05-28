#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../bits/types_oop.h"
#include "../bits/fixtures.h"
#include "../utils/type_conversion.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_sphere_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include <set>

namespace dte3607::physengine::solver_dev::level2
{
  using namespace dte3607::physengine::types;
  using TP        = types::HighResolutionTP;

  struct Params{
    Vector3     F; // externalForces
    Duration    dt;
  };
  struct CacheProcDataBlock{
    size_t  rb_id;
    Point3  in_p; // Position
    Vector3 in_v; // Velocity

    TP      in_tp;

    Vector3 out_ds; // Trajectory
    Vector3 out_a;
  };

  struct SimProcDataBlock{
    Point3  in_p; // Position
    Vector3 in_ds; // Trajectory
    DtRep   in_ds_scalar;
    Point3  out_p; // New Position
  };
  struct SphereGeomDataBlock{


    Point3      in_p ={};        // Position in global coordinate space
    ValueType   r= {};           // radius
    Vector3     ds ={};         // Trajectory in global coordinate space
    TP          sp_time = {};
    size_t      cache_index= {} ;
    bool operator<(SphereGeomDataBlock const& rhs) const {
      return sp_time < rhs.sp_time;
    }
  };
  struct InfPlaneGeomDataBlock{
    Point3  in_p;        // Position in global coordinate space
    Vector3 n;// Normal in global coordinate space
    size_t rb_id;

  };
  struct IntsecStatusDataBlock{
    bool    is_collision;  // whether there was a collision or not
    TP      col_tp;      // Time Point in frame (t_0,t_0+delta t)
  };

  struct IntersectDetProcDataBlock{
    SphereGeomDataBlock     in_sphere; // Sphere data
    InfPlaneGeomDataBlock   in_plane; // Inf plane data
    IntsecStatusDataBlock   out_status; // Intersection status data
  };


  using CacheProcData             = std::vector<CacheProcDataBlock>;
  using SimProcData               = std::vector<SimProcDataBlock>;

  using IntersectDetProcData      = std::vector<IntersectDetProcDataBlock>;
  //using SphereGeoData         = std::vector<SphereGeomDataBlock>;
  //using InfPlaneGeomData      = std::vector<InfPlaneGeomDataBlock>;
  // using IntsecStatusData      = std::vector<IntsecStatusDataBlock>;

  namespace detail {
    //        [[maybe_unused]]
    void computeCache(CacheProcData& data, Params& params)
    {
      auto const cache_proc_kernel = [&params](auto& data_block)
      {
        auto const& [F,dt] = params;
        auto& [rb_id , pos, vel,in_tp, out_ds, out_a] = data_block;
        auto traj_accel = mechanics::computeLinearTrajectory(vel, F, dt);
        out_ds = traj_accel.first;
        out_a= traj_accel.second;
      };
      std::ranges::for_each(data, cache_proc_kernel);        }

    [[maybe_unused]] auto const ds_scalar = [](auto const& s, auto const& e, auto const& dt){
      auto es =  e-s;
      auto ess = utils::toDtScalar(es);
      auto dts = utils::toDtScalar(dt);
      return ess / dts;
    };
    auto const sim_kernel = [](auto& data_block){
      auto& [p, ds,in_ds_scalar, out_p] = data_block;
      out_p = p + ds * in_ds_scalar;
    };
    void computeSimulation( SimProcData& data){
      std::ranges::for_each(data, sim_kernel);
    }
    void computeIntersection( IntersectDetProcData& data,
                             HighResolutionTP t_0, Duration dt)
    {
      auto const insect_kernel = [&](auto& data_block){
        auto& [s, f, out_status] = data_block;
        auto col_tp = mechanics::implementation::computeIntersectionSpFp(
          s.sp_time, s.in_p, s.r, s.ds,
          f.in_p, f.n,
          t_0, dt);
        if(col_tp)
        {

          out_status.is_collision = true;
          out_status.col_tp       = col_tp.value();
        }
        else
        {
          out_status.is_collision = false;
        }
      };
      std::ranges::for_each(data, insect_kernel);
    }

    template <typename IntersectContainer_T>
    void
    makeSortedAndUnique(IntersectContainer_T& intersections)
    {
      std::sort(std::begin(intersections), std::end(intersections),
                [](auto const& intblock_a , auto const& intblock_b){
                  return intblock_a.out_status.col_tp < intblock_b.out_status.col_tp;
                });
      //make unique
      using IntUniqueMemSet = std::set<size_t>;
      IntUniqueMemSet int_unique_set;
      std::erase_if(intersections, [&int_unique_set](auto const& obj){

        if(not obj.out_status.is_collision)
          return true;
        SphereGeomDataBlock const& s = obj.in_sphere;
        if(int_unique_set.contains(s.cache_index))
          return true;
        int_unique_set.insert(s.cache_index);
        return false;
      });

               //reverse (Smallest first
      std::reverse(std::begin(intersections), std::end(intersections));

    }
    //        template <typename Sphere_T>
    //        void
    //        handleCollision(Sphere_T& s1, Sphere_T& s2)
    //        {
    //            Vector3 nv1;
    //            Vector3 nv2;
    //            nv1 = s1.velocity;
    //            nv2 = s2.velocity;
    //        }
  }




  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
    //Initialization step
    auto non_fixed_rbs  = scenario.nonFixedSphereRBs();
    auto fixed_rbs      = scenario.fixedInfPlaneRBs();

    //timpe point initialized equalr for all RBS;
    auto const t_0 = types::HighResolutionClock().now();

    //cache initialization
    CacheProcData cache_data;
    for(auto o: non_fixed_rbs)
    {
      auto const& pos = scenario.globalFramePosition(o);
      auto const& vel = scenario.rbSphereVelocity(o);

      CacheProcDataBlock db{o, pos,vel,t_0,{},{}};
      cache_data.push_back(db);
    }

    Params env_params{scenario.externalForces(),timestep};

    //then we compute cache properties
    detail::computeCache(cache_data, env_params);


    //create sim process from cache data

    //Procedure step
    //detect_collision_RBS starts
    //set up data blocks for collision detection
    //loop through all sphere and planes and create intersection
    //datablocks
    IntersectDetProcData int_data;
    std::vector<SphereGeomDataBlock>    s_g_block;
    std::vector<InfPlaneGeomDataBlock>  p_g_block;
    size_t i    =   0 ;

             //IntsecStatusData insect_data;
    for(auto m = non_fixed_rbs.begin(); m!= non_fixed_rbs.end(); ++m,++i)
    {
      //obtain p ,ds, tp from cache data block
      //and r from sphere object

      auto const& sp = *m;
      auto const& sc = cache_data[i];
      SphereGeomDataBlock s_geom{sc.in_p , scenario.rbSphereRadius(sp),
                                 sc.out_ds, sc.in_tp, i};
      s_g_block.push_back(s_geom);

    }
    for (auto f : fixed_rbs)
    {
      InfPlaneGeomDataBlock p_geom{scenario.globalFramePosition(f),
                                   scenario.rbPlaneNormal(f), f};
      p_g_block.push_back(p_geom);
    }

             //nested loop to check intersection between every sphere agains every plane
    for(auto s : s_g_block)
    {
      for(auto p : p_g_block)
      {
        IntersectDetProcDataBlock i_data_block {s, p, {}};
        int_data.push_back(i_data_block);
      }
    }
    //make a call to computeIntersection
    detail::computeIntersection(int_data, t_0, timestep);

             // Now, the intersection candidates reside in the int_data
    detail::makeSortedAndUnique(int_data);


             //auto const& time = std::sort(int_data.begin(),int_data.end());

             //then handle collision event
             // 1. Take first element from int_Data
             // 2. simulate sphere up to time of import
             // 3. compute intersection response (new velocity ector, new direction)
             // 4. Update cache properties (re-compute trajectory with updated velocity)
             // 5. Check for new collisions after the impact response
    while(not int_data.empty())
    {
  //1. Take first element from int_Data
      auto const col = int_data.back();
      int_data.pop_back();
      if(not col.out_status.is_collision)
        continue;
  //2. simulate sphere up to time of impact
      auto& c = cache_data.at(col.in_sphere.cache_index);

      auto const scalar  = detail::ds_scalar(c.in_tp, col.out_status.col_tp, timestep); //cached timepoint
      SimProcDataBlock s {c.in_p, c.out_ds, scalar, {}};
      detail::sim_kernel(s);

      scenario.setPosition(c.rb_id, s.out_p);

    //3. compute impact response
      auto const new_v = mechanics::computeImpactResponseSphereFixedPlane(c.in_v, col.in_plane.n);
      scenario.setVelocity(c.rb_id, new_v);

    //4. update cache properties (modify in_parameter velocity, against time frame
      c.in_v  = new_v;
      c.in_tp = col.out_status.col_tp;
      c.in_p  = s.out_p;

      //detail::computeCache();
      //this is ugly, should be using cache comuting kernel lambda instead
      auto const& [ext_forces, dt] = env_params;
      //            auto& [rb_id , pos, vel,in_tp, out_ds, out_a] = data_block;
      auto traj_accel = mechanics::computeLinearTrajectory(c.in_v, ext_forces, dt);
      c.out_ds = traj_accel.first;
      c.out_a= traj_accel.second;

   //5. Check for new collisions
   // for the sphere in question check collisons against other planes except the one just collided with

      IntersectDetProcData new_int_candidates;
        for(auto p : p_g_block){
          if (col.in_plane.rb_id != p.rb_id){
              SphereGeomDataBlock sp_geom{c.in_p, col.in_sphere.r, c.out_ds, c.in_tp, col.in_sphere.cache_index};
          IntersectDetProcDataBlock i_data_block {sp_geom, p, {}};
          new_int_candidates.push_back(i_data_block);
        }
        }

        detail::computeIntersection(new_int_candidates, t_0, timestep);
        // add new intersection candidates to the list of possible intersections
        for (auto candidate : new_int_candidates)
        int_data.push_back(candidate);

        //sort again, since we might have new candidates
        detail:: makeSortedAndUnique(int_data);
      }

    //Create sim process data from cache data
    SimProcData sim_data;
    for(auto c:cache_data)
    {
      auto const scalar = detail::ds_scalar(c.in_tp , t_0+timestep, timestep);
      SimProcDataBlock s{c.in_p, c.out_ds, scalar, {}};
      sim_data.push_back(s);
    }
    detail::computeSimulation(sim_data);

             //write result of simulation back to original data objects (in scenario)

    SimProcData::const_iterator sim_proc_data_ptr      = sim_data.begin();
    CacheProcData::const_iterator cache_proc_data_ptr  = cache_data.begin();

    for(auto o: non_fixed_rbs)
    {
        scenario.setPosition(o, *sim_proc_data_ptr->out_p);

        //scenario.addAcceleration(o, *cache_proc_data_ptr->out_a);
        ++sim_proc_data_ptr;
        ++cache_proc_data_ptr;
    }

  }

}   // namespace dte3607::physengine::solver_dev::level2

#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
