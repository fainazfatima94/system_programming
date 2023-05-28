#ifndef GM2_PARAMETRIC_SURFACE_BEZIERSURFACE_H
#define GM2_PARAMETRIC_SURFACE_BEZIERSURFACE_H


#include "../tensorproductsurface.h"

#include "../../core/basisgenerators/bernsteinbasisgenerators.h"
#include "../../core/coreutils.h"

namespace gm::parametric
{


  template <gmc::spaces::ProjectiveSpace EmbedSpace_T
            = spaces::ProjectiveSpace<double, 3ul>>
  struct BezierSurface : TensorProductSurface<EmbedSpace_T> {

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


    using ControlNet = DMatrixT<Point>;


    // Constructor(s)
    template <typename... Ts>
   /* requires gmc::spaces::R2<EmbedSpace> */BezierSurface(const ControlNet& C = {},
                                                       Ts&&... ts)
      : Base(std::forward<Ts>(ts)...), m_C{C}
    {
    }

    // Members
    ControlNet m_C;

    // Methods
    DomainInfo const& domain() const override
    {
      static const DomainInfo d{
        .Uq = Parameterization::TensorAxisTopology::Qualification::Closed,
        .Vq = Parameterization::TensorAxisTopology::Qualification::Closed,
        .U  = {{Type(0), Type(1)}},
        .V  = {{Type(0), Type(1)}}};
      return d;
    }


    PointH evaluateAt(DPoint const& uv, DTraceDirArray const& /*trace_dir*/ = {
                                          TraceDirection::LeftToRight,
                                          TraceDirection::LeftToRight,
                                        }) const override
    {
      const auto u = uv[0];
      const auto v = uv[1];

      const auto Bu
        = basis::generateBernsteinBasisMatrix(int(m_C.rows() - 1), u);
      const auto Bv
        = basis::generateBernsteinBasisMatrix(int(m_C.columns() - 1), v);

      const auto res = Bu * (m_C * blaze::trans(Bv));

      return utils::extendStaticContainer<Point, PointH, VectorDim, 1UL>(
        res(0, 0), Type(1));
    }

    std::optional<VectorH>
    partialDerivativeAt(DSizeArray const& order, DPoint const& uv,
                        DTraceDirArray const& /*trace_dir*/ = {
                          TraceDirection::LeftToRight,
                          TraceDirection::LeftToRight,
                        }) const override
    {

      const auto order_u = order[0];
      const auto order_v = order[1];

      assert(order_u > 0 or order_v > 0);
      assert(order_u <= 1 and order_v <= 1);

      const auto u = uv[0];
      const auto v = uv[1];

      const auto Bu
        = basis::generateBernsteinBasisMatrix(int(m_C.rows() - 1), u);
      const auto Bv
        = basis::generateBernsteinBasisMatrix(int(m_C.columns() - 1), v);

      const auto res = Bu * (m_C * blaze::trans(Bv));

      return utils::extendStaticContainer<Vector, VectorH, VectorDim, 1UL>(
        res(order[0], order[1]), 1.0);
    }
  };


}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_SURFACE_BEZIERSURFACE_H
