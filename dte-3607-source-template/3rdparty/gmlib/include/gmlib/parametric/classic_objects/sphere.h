#ifndef GM2_PARAMETRIC_CURVE_SPHERE_H
#define GM2_PARAMETRIC_CURVE_SPHERE_H


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


  namespace sphere {

    namespace detail {
      template <typename Type_T>
      auto evaluateHelp(Type_T const& u, Type_T const& v)
      {
        Type_T const su = std::sin(u);
        Type_T const sv = std::sin(v);
        Type_T const cu = std::cos(u);
        Type_T const cv = std::cos(v);

        return std::tuple(su, sv, cu, cv);
      }
    }

    template <typename PointH_T, typename Type_T>
    PointH_T evaluateAt(Type_T const& u, Type_T const& v, Type_T const& radius)
    {
      auto const [su, sv, cu, cv] = detail::evaluateHelp(u, v);
      auto res                    = PointH_T{cv * cu, cv * su, sv, 1};
      blaze::subvector<0ul, 3ul>(res) *= radius;
      return res;
    }

    template <typename VectorH_T, typename Type_T>
    VectorH_T derivativeAt(size_t order_u, size_t order_v, Type_T const& u,
                           Type_T const& v, Type_T const& radius)
    {
      assert(order_u > 0 or order_v > 0);

      auto const [su, sv, cu, cv] = detail::evaluateHelp(u, v);



//    const Unit r    = m_radius;
//    const Unit cu   = std::cos(u);
//    const Unit cv   = r * std::cos(v);
//    const Unit su   = std::sin(u);
//    const Unit sv   = r * std::sin(v);
//    const Unit cucv = cu * cv;
//    const Unit cusv = cu * sv;
//    const Unit sucv = su * cv;
//    const Unit susv = su * sv;

//    p(0, 0) = {cucv, sucv, sv, 1};                              // S
//    if (no_der_u) p(1, 0) = {-sucv, cucv, 0, 0};                // Su
//    if (no_der_v) p(0, 1) = {-cusv, -susv, cv, 0};              // Sv
//    if (no_der_u and no_der_v) p(1, 1) = {susv, -cusv, 0, 0};   // Suv

      return {0};
    }

  }   // namespace sphere


  template <gmc::spaces::P3 EmbedSpace_T = spaces::ProjectiveSpace<double, 3ul>>
  struct Sphere : TensorProductSurface<EmbedSpace_T> {

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
    Sphere(Type radius = Type(3), Ts&&... ts)
      : Base(std::forward<Ts>(ts)...), m_radius{radius}
    {
    }


    // Members
    Type m_radius{1};


    // Methods
    DomainInfo const& domain() const override
    {
      constexpr auto          pi = std::numbers::pi_v<Type>;
      static const DomainInfo d{
        .Uq = Parameterization::TensorAxisTopology::Qualification::Loop,
        .Vq = Parameterization::TensorAxisTopology::Qualification::Open,
        .U  = {{0, 2 * pi}},
        .V  = {{-pi / Type(2), pi / Type(2)}}};
      return d;
    }


    PointH evaluateAt(DPoint const& uv, DTraceDirArray const& /*trace_dir*/ = {
                                          TraceDirection::LeftToRight,
                                          TraceDirection::LeftToRight,
                                        }) const override
    {
      return sphere::evaluateAt<PointH>(uv[0], uv[1], m_radius);
    }
  };



}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_CURVE_SPHERE_H
