#ifndef GM2_PARAMETRIC_POLYGONALSURFACE_POLYGONSURFACE_H
#define GM2_PARAMETRIC_POLYGONALSURFACE_POLYGONSURFACE_H


#include "../polygonalsurface.h"

#include "../../core/divideddifference/finitedifference.h"
#include "../../core/coreutils.h"
#include "../../core/gbc/mvc.h"

namespace gm::parametric
{


  template <gmc::spaces::EucledianSpace EmbedSpace_T
            = spaces::ProjectiveSpace<double, 3ul>>
  struct PolygonSurface
    : PolygonalSurface<mappingkernel::MVCPolygonalSurfaceMappingKernel,
                       EmbedSpace_T> {


    /***
     * Boiler plate */

    /* Base */
    using Base = PolygonalSurface<mappingkernel::MVCPolygonalSurfaceMappingKernel,
                                EmbedSpace_T>;
    using EmbedSpaceObject = typename Base::EmbedSpaceObject;
    using EmbedSpace       = typename Base::EmbedSpace;
    using Type             = typename Base::Type;


    /* Domain space */
    using DomainSpace      = typename Base::DomainSpace;
    using Domain           = typename Base::Domain;
    using DomainInfo       = typename Base::DomainInfo;
    using Parameterization = typename Base::Parameterization;
    GM2_MAPPING_KERNEL_BOILER_PLATE

    /*** Boiler plate END */


    // Polygon construction types
//    template <typename Element_T>
//    using PolygonContainer = DVectorT<Element_T>;

//    using Lambda    = PolygonContainer<Type>;
//    using Lambdas   = PolygonContainer<Lambda>;
    using Vertices = DVectorT<Point>;



    // Constructor(s)
    template <typename... Ts>
    PolygonSurface(Vertices const& vertices, Ts&&... ts)
      : Base(std::forward<Ts>(ts)...), m_vertices(vertices)

    {
      m_domain.polygon
        = polygonutils::generateRegularPolygon2D<>(vertices.size());
    }

    Vertices   m_vertices;
    DomainInfo m_domain;


    DomainInfo const& domain() const
    {
      return m_domain;
    }

//    EvaluationResult evaluate(PSpacePoint const&     par,
//                              PSpaceSizeArray const& no_der,
//                              PSpaceBoolArray const& /*from_left*/) const
//    {
//      const auto eval_fn
//        = [this](const auto& at_par) { return evaluatePosition(at_par); };

//      // Results
//      EvaluationResult p(no_der[0] + 1, no_der[1] + 1);

//      // S
//      p(0, 0) = utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
//        eval_fn(par), Unit(1));

//      // Du S
//      if (no_der[0] > 0) {
//        const auto par_d0 = PSpaceVector{1e-6, 0};
//        const auto Ds0    = D(par, par_d0, eval_fn);

//        p(1, 0)
//          = utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
//            Ds0, Unit(1));
//      }

//      // Dv S
//      if (no_der[1] > 0) {
//        const auto par_d1 = PSpaceVector{0, 1e-6};
//        const auto Ds1    = D(par, par_d1, eval_fn);

//        p(0, 1)
//          = utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
//            Ds1, Unit(1));
//      }

//      // Duv S
//      if (no_der[0] > 0 and no_der[1] > 0)
//        p(1, 1) = typename EvaluationResult::ElementType(0);

//      return p;
//    }

  private:
    PointH evaluateAtGBC(typename Domain::Lambda const& l) const
    {
      auto const&  V    = m_vertices;
      auto const   eval = blaze::inner(l, V);

      return utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
        eval, Type(1));
    }

//    PointH evaluateAt(const DPoint& par) const
//    {
//      auto const&  v    = par;
//      auto const   l    = gbc::mvc(m_domain.polygon, v);
//      auto const&  V    = m_vertices;
//      auto const   eval = blaze::inner(l, V);

//      return utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
//        eval, Type(1));
//    }

    static constexpr divideddifference::fd::CentralDifference D{};
  };


}   // namespace gm::parametric


#endif   // GM2_PARAMETRIC_POLYGONALSURFACE_POLYGONSURFACE_H
