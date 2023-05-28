#ifndef GM2_PARAMETRIC_TENSORPRODUCTSURFACE_H
#define GM2_PARAMETRIC_TENSORPRODUCTSURFACE_H

#include "mappingobjects.h"
#include "parameterizations.h"
#include "../core/datastructures.h"
#include "utils/numeric_differentiation.h"

// gmconcepts
#include <gmconcepts/manifolds.h>

// stl
#include <optional>
#include <vector>

namespace gm::parametric
{

  namespace mappingkernel
  {
    template <typename EmbedSpace_T>
    struct TensorProductSurfaceMappingKernel : SpaceObject<EmbedSpace_T> {

      /***
       * Boiler plate */

      /* Base */
      using Base             = SpaceObject<EmbedSpace_T>;
      using EmbedSpaceObject = Base;
      using EmbedSpace       = typename EmbedSpaceObject::Space;
      using Type             = typename EmbedSpace::Type;

      /* Domain space */
      using DomainSpace      = spaces::ProjectiveSpace<Type, 2ul>;
      using Domain           = domain::TensorProductDomain<DomainSpace>;
      using DomainInfo       = typename Domain::DomainInfo;
      using Parameterization = typename Domain::Parameterization;
      GM2_MAPPING_KERNEL_BOILER_PLATE

      /* Interval Meta */
      using TraceDirection = typename Parameterization::TraceDirection;
      using DTraceDirArray = typename Parameterization::TraceDirArray;
      using DTraceDDPair   = typename Parameterization::TraceDDPair;
      /*** Boiler plate END */




      using Base::Base;
      virtual ~TensorProductSurfaceMappingKernel() = default;


      virtual DomainInfo const& domain() const = 0;

      virtual PointH evaluateAt(DPoint const&         uv,
                                DTraceDirArray const& trace_dir
                                = {TraceDirection::LeftToRight,
                                   TraceDirection::LeftToRight}) const = 0;

      virtual std::optional<VectorH> partialDerivativeAt(
        DSizeArray const& order, DPoint const& uv,
        DTraceDirArray const& trace_dir
        = {TraceDirection::LeftToRight, TraceDirection::LeftToRight}) const
      {
        auto const& [order_u, order_v] = order;
        if (order_u == 0ul and order_v == 0ul) return {};

        static constexpr numericdiff::finitediff::CentralFD D;
        auto fe = [this, trace_dir](DPoint uv0) {
          return evaluateAt(uv0, trace_dir);
        };

        auto const h  = 1e-6;
        if (order_u == 1ul and order_v == 0ul)
          return {D(uv, DPoint{h, 0}, fe)};

        else if (order_u == 0ul and order_v == 1ul)
          return {D(uv, DPoint{0, h}, fe)};

        return {};
      }

//      virtual std::vector<std::optional<VectorH>>
//      directionalDerivativesAt(size_t order, DPoint const& uv,
//                               DTraceDDPair const& dirs) const
//      {
//        return {};
//      }

//      virtual std::optional<VectorH>
//      directionalDerivativeAt(size_t order, DPoint const& uv,
//                              DTraceDDPair const& dir) const
//      {
//        auto const dds = directionalDerivativesAt(order, uv, dir);
//        if (dds.size() < order) return {};

//        return dds[order - 1];
//      }

//      virtual std::vector<std::optional<VectorH>>
//      directionalDerivativesAt(DPoint const&                    uv,
//                               std::vector<DTraceDDPair> const& dirs) const
//      {
//        return {};
//      }

//      virtual std::optional<VectorH>
//      directionalDerivativeAt(DPoint const&                    uv,
//                              std::vector<DTraceDDPair> const& dirs) const
//      {
//        auto const dds = directionalDerivativesAt(uv, dirs);
//        if (dds.size() < dirs.size()) return {};

//        auto const dd_no = dirs.size() - 1ul;
//        return dds[dd_no];
//      }

    };

  }   // namespace mappingkernel


  template <gmc::spaces::Space EmbedSpace_T>
  using TensorProductSurface
    = MappingObject<mappingkernel::TensorProductSurfaceMappingKernel,
                    EmbedSpace_T>;

  namespace tensorproductsurface
  {

    template <gmc::TensorProductSurface Surface_T>
    typename Surface_T::PointH
    evaluateAtLocal(Surface_T const& obj, typename Surface_T::DPoint const& uv,
                    typename Surface_T::DTraceDirArray const& trace_dir
                    = {{Surface_T::TraceDirection::LeftToRight,
                        Surface_T::TraceDirection::LeftToRight}})
    {
      return obj.evaluateAt(uv, trace_dir);
    }


    template <gmc::TensorProductSurface Surface_T>
    typename Surface_T::PointH
    evaluateAtParent(Surface_T const& obj, typename Surface_T::DPoint const& uv,
                     typename Surface_T::DTraceDirArray const& trace_dir)
    {
      const auto trace  = obj.evaluateAt(uv, trace_dir);
      const auto pframe = obj.pSpaceFrameParent();
      return pframe * trace;
    }







    template <gmc::TensorProductSurface Surface_T>
    typename datastructures::parametrics::tensorproductsurface::
      EvaluationResult<typename Surface_T::VectorH>
      evaluateLocal(Surface_T const& obj, typename Surface_T::DPoint const& par,
                    typename Surface_T::DSizeArray const& no_der,
                    typename Surface_T::DBoolArray const& /*from_left*/)
    {
      //      return obj.evaluate(par, no_der, from_left);

      using EvaluationResult = typename datastructures::parametrics::
        tensorproductsurface::EvaluationResult<typename Surface_T::VectorH>;
      // DEV HACK -- start (hack reason: unsafe)
      EvaluationResult res(no_der[0] + 1, no_der[1] + 1);
//      res(0, 0) = obj.evaluateAt(par[0], par[1], from_left[0], from_left[1]);
//      if (no_der[0] > 0)
//        res(1, 0) = obj.template derivativeAt<1ul, 0ul>(
//          par[0], par[1], from_left[0], from_left[1]);
//      if (no_der[1] > 0)
//        res(0, 1) = obj.template derivativeAt<0ul, 1ul>(
//          par[0], par[1], from_left[0], from_left[1]);
//      if (no_der[0] > 0 and no_der[1] > 0)
//        res(1, 1) = obj.template derivativeAt<1ul, 1ul>(
//          par[0], par[1], from_left[0], from_left[1]);

      res(0, 0) = obj.evaluateAt(par);
      if (no_der[0] > 0)
        res(1, 0) = obj.partialDerivativeAt({1ul, 0ul}, par)
                      .value_or(typename Surface_T::VectorH(0));
      if (no_der[1] > 0)
        res(0, 1) = obj.partialDerivativeAt({0ul, 1ul}, par)
                      .value_or(typename Surface_T::VectorH(0));
      if (no_der[0] > 0 and no_der[1] > 0)
        res(1, 1) = obj.partialDerivativeAt({1ul, 1ul}, par)
                      .value_or(typename Surface_T::VectorH(0));
      // DEV HACK -- end

      return res;
    }


    template <gmc::TensorProductSurface Surface_T>
    typename datastructures::parametrics::tensorproductsurface::
      EvaluationResult<typename Surface_T::VectorH>
      evaluateParent(Surface_T const&                      obj,
                     typename Surface_T::DPoint const&     par,
                     typename Surface_T::DSizeArray const& no_der,
                     typename Surface_T::DBoolArray const& from_left)
    {
      auto const eval_res = evaluateLocal(obj,par,no_der,from_left);
      const auto pframe = obj.pSpaceFrameParent();

      using EvaluationResult = typename datastructures::parametrics::
        tensorproductsurface::EvaluationResult<typename Surface_T::VectorH>;
      EvaluationResult res = blaze::map(eval_res, [pframe](const auto& ele) {
        return blaze::evaluate(pframe * ele);
      });
      return res;
    }


    template <gmc::TensorProductSurface Surface_T>

    datastructures::parametrics::tensorproductsurface::SamplingResult<
      typename Surface_T::VectorH>
    sample(Surface_T const& pobj, typename Surface_T::DPoint const& par_start,
           typename Surface_T::DPoint const&     par_end,
           typename Surface_T::DSizeArray const& no_samples,
           typename Surface_T::DSizeArray const& no_derivatives)
    {
      using Type           = typename Surface_T::Type;
      using DPoint         = typename Surface_T::DPoint;
      using SamplingResult
        = datastructures::parametrics::tensorproductsurface::SamplingResult<
          typename Surface_T::VectorH>;

      auto const m1  = no_samples[0];
      auto const m2  = no_samples[1];
      auto const s_u = par_start[0];
      auto const s_v = par_start[1];
      auto const e_u = par_end[0];
      auto const e_v = par_end[1];

      Type du = (e_u - s_u) / (Type(m1) - 1);
      Type dv = (e_v - s_v) / (Type(m2) - 1);

      SamplingResult p(m1, m2);

      for (int r = 0; r < int(m1) - 1; r++) {
        auto const i = Type(r);
        Type const u = s_u + i * du;
        for (int c = 0; c < int(m2) - 1; c++) {
          auto const j = Type(c);
          p(size_t(r), size_t(c)) = evaluateLocal(
            pobj, DPoint{u, s_v + j * dv}, no_derivatives, {{true, true}});
        }
        p(size_t(r), m2 - 1) = evaluateLocal(pobj, DPoint{u, e_v},
                                             no_derivatives, {{true, false}});
      }

      for (int c = 0; c < int(m2) - 1; c++) {
        auto const j              = Type(c);
        p(m1 - 1, size_t(c)) = evaluateLocal(pobj, DPoint{e_u, s_v + j * dv},
                                             no_derivatives, {{false, true}});
      }

      p(m1 - 1, m2 - 1) = evaluateLocal(pobj, DPoint{e_u, e_v}, no_derivatives,
                                        {{false, false}});

      return p;
    }

  }   // namespace tensorproductsurface


}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_TENSORPRODUCTSURFACE_H
