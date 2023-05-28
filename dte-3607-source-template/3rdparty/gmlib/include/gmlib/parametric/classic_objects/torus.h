#ifndef GM2_PARAMETRIC_SURFACE_TORUS_H
#define GM2_PARAMETRIC_SURFACE_TORUS_H


#include "../tensorproductsurface.h"
#include "../parameterizations.h"

#include "../../core/coreutils.h"


// gmconcepts
#include <gmconcepts/spaces.h>
#include <gmconcepts/parameterization.h>
#include <gmconcepts/prototype.h>

// stl
#include <numbers>


namespace gm::parametric
{

  namespace torus
  {

    namespace detail
    {

      template <typename Type_T>
      auto evaluateHelp(Type_T const& u, Type_T const& v,
                        Type_T const& wheelradius, Type_T const& tuberadius1,
                        Type_T const& tuberadius2)
      {
        Type_T     su   = std::sin(u);
        Type_T     sv   = std::sin(v);
        Type_T     cu   = std::cos(u);
        Type_T     cv   = std::cos(v);
        auto const bcva = tuberadius1 * cv + wheelradius;
        auto const cucv = tuberadius1 * cu * cv;
        auto const cusv = tuberadius1 * cu * sv;
        auto const sucv = tuberadius1 * su * cv;
        auto const susv = tuberadius1 * su * sv;
        sv *= tuberadius2;
        cv *= tuberadius2;
        cu *= bcva;
        su *= bcva;

        return std::tuple(su, sv, cu, cv, bcva, cucv, cusv, sucv, susv);
      }

    }   // namespace detail

    template <typename PointH_T, typename Type_T>
    PointH_T evaluateAt(Type_T const& u, Type_T const& v,
                        Type_T const& wheelradius, Type_T const& tuberadius1,
                        Type_T const& tuberadius2)
    {
      auto const [su, sv, cu, cv, bcva, cucv, cusv, sucv, susv]
        = detail::evaluateHelp(u, v, wheelradius, tuberadius1, tuberadius2);
      return {cu, su, sv, 1};
    }

    template <typename VectorH_T, typename Type_T>
    VectorH_T derivativeAt(size_t order_u, size_t order_v, Type_T const& u,
                           Type_T const& v, Type_T const& wheelradius,
                           Type_T const& tuberadius1, Type_T const& tuberadius2)
    {
      assert(order_u > 0 or order_v > 0);

      auto const [su, sv, cu, cv, bcva, cucv, cusv, sucv, susv]
        = detail::evaluateHelp(u, v, wheelradius, tuberadius1, tuberadius2);

      // clang-format off
      if       (order_u == 0 and order_v == 1) return {-cusv, -susv, cv, 0};   // Sv
      else if  (order_u == 0 and order_v == 2) return {-cucv, -sucv, -sv, 0};  // Svv
      else if  (order_u == 0 and order_v == 3) return {cusv, susv, -cv, 0};    // Svvv
      else if  (order_u == 1 and order_v == 0) return {-su, cu, 0, 0};         // Su
      else if  (order_u == 1 and order_v == 1) return {susv, -cusv, 0, 0};     // Suv
      else if  (order_u == 1 and order_v == 2) return {sucv, cucv, 0, 0};      // Suvv
      else if  (order_u == 1 and order_v == 3) return {-susv, cusv, 0, 0};     // Suvvv
      else if  (order_u == 2 and order_v == 0) return {-cu, -su, 0, 0};        // Suu
      else if  (order_u == 2 and order_v == 1) return {cusv, susv, 0, 0};      // Suuv
      else if  (order_u == 2 and order_v == 2) return {cucv, sucv, 0, 0};   	 // Suuvv
      else if  (order_u == 2 and order_v == 3) return {-cusv, -susv, 0, 0};    // Suuvvv
      else if  (order_u == 3 and order_v == 0) return {su, -cu, 0, 0};         // Suuu
      else if  (order_u == 3 and order_v == 1) return {-susv, cusv, 0, 0};     // Suuuv
      else if  (order_u == 3 and order_v == 2) return {-sucv, cucv, 0, 0};     // Suuuvv
      else if  (order_u == 3 and order_v == 3) return {susv, cusv, 0, 0};      // Suuuvvv
      // clang-format on
      return {0};
    }


  }   // namespace torus





  template <gmc::spaces::P3 EmbedSpace_T = spaces::ProjectiveSpace<double, 3ul>>
  struct Torus : TensorProductSurface<EmbedSpace_T> {

    /***
     * Boiler plate */

    /* Base */
    using Base             = TensorProductSurface<EmbedSpace_T>;
    using EmbedSpaceObject = typename Base::EmbedSpaceObject;
    using EmbedSpace       = typename Base::EmbedSpace;
    using Type             = typename Base::Type;

    /* Domain space */
    using DomainSpace      = typename Base::DomainSpace;
    using Domain           = typename Base::Domain;
    using DomainInfo       = typename Base::DomainInfo;
    using Parameterization = typename Base::Parameterization;
    GM2_MAPPING_KERNEL_BOILER_PLATE

    /* Interval Meta */
    using TraceDirection = typename Parameterization::TraceDirection;
    using DTraceDirArray = typename Parameterization::TraceDirArray;

    /*** Boiler plate END */





    // Constructor(s)
    template <typename... Ts>
    Torus(Type wheelradius = Type(3), Type tuberadius1 = Type(1),
          Type tuberadius2 = Type(1), Ts&&... ts)
      : Base(std::forward<Ts>(ts)...), m_wheelradius{wheelradius},
        m_tuberadius1{tuberadius1}, m_tuberadius2{tuberadius2}
    {
    }


    // Members
    Type m_wheelradius{3};
    Type m_tuberadius1{1};
    Type m_tuberadius2{1};


    // Methods
    DomainInfo const& domain() const override
    {
      constexpr auto          pi = std::numbers::pi_v<Type>;
      static const DomainInfo d{
        .Uq = Parameterization::TensorAxisTopology::Qualification::Loop,
        .Vq = Parameterization::TensorAxisTopology::Qualification::Loop,
        .U  = {{0, 2 * pi}},
        .V  = {{0, 2 * pi}}};
      return d;
    }


    PointH evaluateAt(DPoint const& uv, DTraceDirArray const& /*trace_dir*/ = {
                                          TraceDirection::LeftToRight,
                                          TraceDirection::LeftToRight,
                                        }) const override
    {
      return torus::evaluateAt<PointH>(uv[0], uv[1], m_wheelradius,
                                       m_tuberadius1, m_tuberadius2);
    }

//    std::optional<VectorH>
//    partialDerivativeAt(DSizeArray const& order, DPoint const& uv,
//                        DTraceDirArray const& /*trace_dir*/ = {
//                          TraceDirection::LeftToRight,
//                          TraceDirection::LeftToRight,
//                        }) const override
//    {
//      return torus::derivativeAt<VectorH>(order[0], order[1], uv[0], uv[1],
//                                          m_wheelradius, m_tuberadius1,
//                                          m_tuberadius2);
//    }
  };


}   // namespace gm::parametric


#endif   // GM2_PARAMETRIC_SURFACE_TORUS_H
