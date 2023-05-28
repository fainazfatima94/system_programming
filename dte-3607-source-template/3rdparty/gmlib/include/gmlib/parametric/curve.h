#ifndef GM2_PARAMETRIC_CURVE_H
#define GM2_PARAMETRIC_CURVE_H


#include "mappingobjects.h"
#include "parameterizations.h"
#include "../core/datastructures.h"
#include "utils/numeric_differentiation.h"

// gmconcepts
#include <gmconcepts/manifolds.h>

namespace gm::parametric
{

  namespace mappingkernel
  {

    template <typename EmbedSpace_T>
    struct CurveMappingKernel : SpaceObject<EmbedSpace_T> {

      /***
       * Boiler plate */

      /* Base */
      using Base             = SpaceObject<EmbedSpace_T>;
      using EmbedSpaceObject = Base;
      using EmbedSpace       = typename EmbedSpaceObject::Space;
      using Type             = typename EmbedSpace::Type;


      /* Domain space */
      using DomainSpace      = spaces::ProjectiveSpace<double, 1ul>;
      using Domain           = domain::IntervalDomain<DomainSpace>;
      using DomainInfo       = typename Domain::DomainInfo;
      using Parameterization = typename Domain::Parameterization;
      GM2_MAPPING_KERNEL_BOILER_PLATE

      /* Interval Meta */
      using TraceDirection = typename Parameterization::TraceDirection;

      /*** Boiler plate END */

      using Base::Base;
      virtual ~CurveMappingKernel() = default;

      virtual DomainInfo const& domain() const = 0;

      virtual PointH evaluateAt(Type const&    t,
                                TraceDirection trace_dir
                                = TraceDirection::LeftToRight) const = 0;

      virtual std::optional<VectorH>
      derivativeAt(size_t order, Type const& t,
                   [[maybe_unused]] TraceDirection trace_dir
                   = TraceDirection::LeftToRight) const
      {
        if (order not_eq 1ul) return {};

        static constexpr numericdiff::finitediff::CentralFD D;

        auto fe
          = [this, trace_dir](Type t0) { return evaluateAt(t0, trace_dir); };
        auto const h  = 1e-6;
        auto const Ds = D(t, h, fe);

        return {Ds};
      }

    };

  }   // namespace mappingkernel

  template <gmc::spaces::Space EmbedSpace_T>
  using Curve = MappingObject<mappingkernel::CurveMappingKernel, EmbedSpace_T>;






  namespace curve
  {

    template <typename Curve_T>
    typename Curve_T::PointH
    evaluateAtLocal(Curve_T const& obj, typename Curve_T::Type const& t,
                    typename Curve_T::TraceDirection
                      trace_dir) requires gmc::ParametricCurve<Curve_T>
    {
      return obj.evaluateAt(t, trace_dir);
    }

    template <typename Curve_T>
    typename Curve_T::PointH
    evaluateAtParent(Curve_T const& obj, typename Curve_T::Type const& t,
                     typename Curve_T::TraceDirection
                       trace_dir) requires gmc::ParametricCurve<Curve_T>
    {
      const auto trace  = obj.evaluateAt(t, trace_dir);
      const auto pframe = obj.pSpaceFrameParent();
      return pframe * trace;
    }

    template <typename Curve_T>
    std::optional<typename Curve_T::VectorH>
    derivativeAtLocal(Curve_T const& obj, size_t order,
                      typename Curve_T::Type const& t,
                      typename Curve_T::TraceDirection
                        trace_dir) requires gmc::ParametricCurve<Curve_T>
    {
      return obj.derivativeAt(order, t, trace_dir);
    }

    template <typename Curve_T>
    std::optional<typename Curve_T::VectorH>
    derivativeAtParent(Curve_T const& obj, size_t order,
                       typename Curve_T::Type const&    t,
                       typename Curve_T::TraceDirection trace_dir)

      requires gmc::ParametricCurve<Curve_T>
    {
      const auto trace = obj.derivativeAt(order, t, trace_dir);
      if (not trace) return {};

      const auto pframe = obj.pSpaceFrameParent();
      return pframe * trace;
    }


    template <typename Curve_T>
    datastructures::parametrics::curve::SamplingResult<
      typename Curve_T::VectorH>
    sample(Curve_T const&                                      pobj,
           typename Curve_T::Parameterization::Interval const& I,
           size_t                                              no_samples,
           bool /*no_derivatives*/) requires gmc::ParametricCurve<Curve_T>
    {
      using Type           = typename Curve_T::Type;
      using SamplingResult = datastructures::parametrics::curve::SamplingResult<
        typename Curve_T::VectorH>;
      using TD = typename Curve_T::TraceDirection;

      auto const m = no_samples;
      auto const s = I.start();
      auto const e = I.end();

      Type const dt = (e - s) / (Type(m) - 1);

      SamplingResult p(m);

      for (int i = 0; i < int(m) - 1; i++) {
        auto const p_i = size_t(i);
        p[p_i].resize(2ul);
        p[p_i][0] = evaluateAtLocal(pobj, s + Type(i) * dt, TD::LeftToRight);
        p[p_i][1]
          = derivativeAtLocal(pobj, 1ul, s + Type(i) * dt, TD::LeftToRight)
              .value_or(typename Curve_T::VectorH(0));
      }
      p[m - 1].resize(2ul);
      p[m - 1][0] = evaluateAtLocal(pobj, I.end(), TD::RightToLeft);
      p[m - 1][1] = derivativeAtLocal(pobj, 1ul, I.end(), TD::RightToLeft)
                      .value_or(typename Curve_T::VectorH(0));

      return p;
    }




    template <gmc::ParametricCurve Curve_T>
    datastructures::parametrics::tensorproductsurface::SamplingResult<
      typename Curve_T::VectorH>
    sampleAsTube(Curve_T const&                                      pobj,
                 typename Curve_T::Parameterization::Interval const& I,
                 size_t no_samples, size_t no_slices,
                 typename Curve_T::Type radius)
    {

      // Curve types
      using Type                    = typename Curve_T::Type;
      using Point [[maybe_unused]]  = typename Curve_T::Point;
      using Vector [[maybe_unused]] = typename Curve_T::Vector;
      using VectorH                 = typename Curve_T::VectorH;
      auto constexpr VectorDim      = Curve_T::VectorDim;

      // Tube sampling types
      using EvaluationResult
        = datastructures::parametrics::tensorproductsurface::EvaluationResult<
          VectorH>;
      using SamplingResult
        = datastructures::parametrics::tensorproductsurface::SamplingResult<
          VectorH>;

      // Sample curve
      auto const curve_samples = sample(pobj, I, {no_samples}, {1ul});

      auto sampPos = [curve_samples](size_t idx) -> VectorH {
        return curve_samples[idx][0ul];
      };

      //      auto sampDer = [curve_samples](size_t idx) -> VectorH {
      //        return curve_samples[idx][1ul];
      //      };

      auto sampDir = [curve_samples, sampPos](size_t idx) -> VectorH {
        auto is  = 0ul;
        auto is2 = 1ul;
        auto ie  = curve_samples.size() - 1ul;
        auto ie2 = curve_samples.size() - 2ul;
        if (idx < is2)
          return blaze::evaluate(sampPos(is2) - sampPos(is));
        else if (idx > ie2)
          return blaze::evaluate(sampPos(ie) - sampPos(ie2));
        else
          return blaze::evaluate((sampPos(idx + 1) - sampPos(idx - 1))
                                 * Type(0.5));
      };


      // Tube slice
      auto slice
        = gm::DVectorT<gm::DVectorT<gm::VectorT<double, 4UL>>>(
          size_t(no_slices));
      {
        double ps = 0.0;
        double pe = 2.0 * M_PI;
        double pd = pe - ps;
        double dt = pd / double(no_slices - 1);

        for (size_t s = 0ul; s < no_slices; ++s) {

          auto const i    = double(s);
          auto const c_t  = ps + dt * i;
          auto const c_r  = double(radius);
          auto const c_ct = c_r * std::cos(c_t);
          auto const c_st = c_r * std::sin(c_t);

          slice[size_t(i)]
            = {gm::VectorT<double, 4UL>{0.0, c_ct, c_st, 1.0},
               gm::VectorT<double, 4UL>{0.0, -c_st, c_ct, 0.0}};
        }
      }

      // Curve Tube sample set
      SamplingResult p(no_samples, no_slices);

      for (auto sa = 0ul; sa < no_samples; ++sa)
        for (auto sl = 0ul; sl < no_slices; ++sl)
          p(sa, sl) = EvaluationResult(2ul, 2ul, VectorH(0));


      // Rotation frames
      auto frame = gm::spaces::projectivespace::identityFrame<
        gm::spaces::ProjectiveSpace<Type, 3ul>>();
      auto rframe = blaze::submatrix<0ul, 0ul, 3ul, 3ul>(frame);

      {
        rframe = gm::algorithms::linearIndependetFrameTo(
          static_cast<gm::DVectorT<double>>(
            blaze::subvector<0ul, VectorDim>(sampDir(0UL))));

        auto const& pos = sampPos(0ul);
        for (auto sl = 0ul; sl < slice.size(); ++sl) {

          auto const& slice_p = slice[sl][0];
          auto const& slice_v = slice[sl][1];

          auto const pt = blaze::evaluate(
            pos
            + ((frame * slice_p)
               - gm::VectorT<double, 4>{0.0, 0.0, 0.0, 1.0}));
          auto const u = blaze::evaluate(
            frame * gm::VectorT<double, 4>{1.0, 0.0, 0.0, 0.0});
          const auto v = blaze::evaluate(frame * slice_v);

          p(0ul, sl)(0, 0) = pt;
          p(0ul, sl)(1, 0) = u;
          p(0ul, sl)(0, 1) = v;
          p(0ul, sl)(1, 1) = VectorH(0);
        }
      }

      for (auto i = 1ul; i < curve_samples.size(); ++i) {

        auto const ri = blaze::evaluate(blaze::column(rframe, 2UL));
        auto const ti = blaze::evaluate(blaze::column(rframe, 0UL));

        auto const prev_cpos
          = blaze::subvector<0ul, VectorDim>(sampPos(i - 1ul));
        auto const cpos = blaze::subvector<0ul, VectorDim>(sampPos(i));
        auto const cder = blaze::subvector<0ul, VectorDim>(sampDir(i));

        // RMF

        rframe = gm::algorithms::rotationMinimizingFrameMSDR<double, 3>(
          static_cast<gm::DVectorT<double>>(ri),
          static_cast<gm::DVectorT<double>>(prev_cpos),
          static_cast<gm::DVectorT<double>>(cpos),
          static_cast<gm::DVectorT<double>>(ti),
          static_cast<gm::DVectorT<double>>(cder));

        // Fill slice
        auto const& cposH = sampPos(i);

        for (auto sl = 0ul; sl < slice.size(); ++sl) {

          const auto pt = blaze::evaluate(
            cposH
            + ((frame * slice[sl][0])
               - gm::VectorT<double, 4>{0.0, 0.0, 0.0, 1.0}));
          const auto u = blaze::evaluate(
            frame * gm::VectorT<double, 4>{1.0, 0.0, 0.0, 0.0});
          const auto v = blaze::evaluate(frame * slice[sl][1]);

          p(i, sl)(0, 0) = pt;
          p(i, sl)(1, 0) = u;
          p(i, sl)(0, 1) = v;
          p(i, sl)(1, 1) = VectorH(0);
        }
      }

      return p;
    }


  }   // namespace curve


}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_CURVE_H
