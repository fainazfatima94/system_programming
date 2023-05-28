#ifndef GM2_PARAMETRIC_SUBOBJECT_SUBCURVE_H
#define GM2_PARAMETRIC_SUBOBJECT_SUBCURVE_H



#include "../curve.h"
#include "../tensorproductsurface.h"
#include "../polygonalsurface.h"
//#include "../volume.h"

#include "../mappingobjects.h"
#include "../utils/numeric_differentiation.h"

// gmconcept
#include <gmconcepts/manifolds.h>
#include <gmconcepts/prototype.h>


// stl
#include <type_traits>

namespace gm::parametric
{




  namespace curveinsurface
  {

    template <typename PointH_T, typename Type_T, typename MappingCurve_T,
              typename EmbedSurface_T>
    PointH_T evaluateAt(Type_T const&                           t,
                        typename MappingCurve_T::TraceDirection trace_dir,
                        MappingCurve_T& mcurve, EmbedSurface_T& esurface)
    {
      // Evaluate pspace curve -- result in homogeneous coords ^^,
      const auto meval = curve::evaluateAtParent(mcurve, t, trace_dir);

      // Make pspace curve result not homeogeneous coords
      const auto uv
        = blaze::subvector<0UL, EmbedSurface_T::DVectorDim>(meval);

      // Evaluate surface along pspace curve
      const auto eval = tensorproductsurface::evaluateAtLocal(esurface, uv);

      // S(C)
      return eval;
    }

  }   // namespace curveinsurface








  namespace mappingkernel
  {

    template <template <typename /*EmbedSpace (MappingSpace) */>
              typename MappingSpaceObject_T,
              typename EmbedSpace_T>
    struct CurveInTensorProductSurfaceMappingKernel
      : MappingObject<CurveMappingKernel, EmbedSpace_T>

    {

      /***
       * Boiler plate */

      /* Base */
      using Base = MappingObject<CurveMappingKernel, EmbedSpace_T>;
      using Type = typename Base::Type;

      using ESO
        = MappingObject<TensorProductSurfaceMappingKernel, EmbedSpace_T>;
      using MSO = MappingSpaceObject_T<typename ESO::DomainSpace>;

      GM2_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MSO, ESO)

      /* Interval Meta */
      using TraceDirection = typename Parameterization::TraceDirection;

      /*** Boiler plate END */


      using MappingCurve = MappingObjectType;
      using EmbedSurface = EmbedObjectType;

      // Members
      MappingCurve  m_mcurve;
      EmbedSurface& m_esurface;

      // Constructor(s)
      template <typename... Ts>
      CurveInTensorProductSurfaceMappingKernel(EmbedSurface& esurface,
                                              Ts&&... ts)
        : m_mcurve(std::forward<Ts>(ts)...), m_esurface{esurface}
      {
      }

      // Methods
      DomainInfo const& domain() const override { return m_mcurve.domain(); }

      PointH evaluateAt(Type const& t, TraceDirection trace_dir) const override
      {
        return curveinsurface::evaluateAt<PointH>(t, trace_dir, m_mcurve,
                                                  m_esurface);
      }

      std::optional<VectorH>
      derivativeAt(size_t /*order*/, Type const& t,
                   TraceDirection trace_dir) const override
      {
        auto fm = [this, trace_dir](Type tloc) {
          return m_mcurve.evaluateAt(tloc, trace_dir);
        };

        auto fe = [this](MPoint par) { return m_esurface.evaluateAt(par); };

        static constexpr numericdiff::
          DirectionalDerivativeForMappingMappingKernel<
            numericdiff::finitediff::CentralFD, MappingCurve, EmbedSurface>
             D{};
        auto res = D(t, 1e-5, fm, fe);
//        std::cout << "SC derivativeAt: " << res << std::endl;
        return {res};
      }
    };



    template <template <typename /*EmbedSpace (MappingSpace) */>
              typename MappingSpaceCurve_T,
              typename EmbedSpace_T>
    struct CurveInPolygonalSurfaceMappingKernel
      : MappingObject<CurveMappingKernel, EmbedSpace_T>

    {

      /***
       * Boiler plate */

      /* Base */
      using Base = MappingObject<CurveMappingKernel, EmbedSpace_T>;
      using Type = typename Base::Type;

      using ESO = MappingObject<MVCPolygonalSurfaceMappingKernel, EmbedSpace_T>;
      using MSO = MappingSpaceCurve_T<typename ESO::DomainSpace>;

      GM2_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MSO, ESO)

      /* Interval Meta */
      using TraceDirection = typename Parameterization::TraceDirection;

      /*** Boiler plate END */


      using MappingCurve = MappingObjectType;
      using EmbedSurface = EmbedObjectType;

      // Members
      MappingCurve  m_mcurve;
      EmbedSurface& m_esurface;

      // Constructor(s)
      template <typename... Ts>
      CurveInPolygonalSurfaceMappingKernel(EmbedSurface& esurface,
                                              Ts&&... ts)
        : m_mcurve(std::forward<Ts>(ts)...), m_esurface{esurface}
      {
      }

      // Methods
      DomainInfo const& domain() const override { return m_mcurve.domain(); }

      PointH evaluateAt(Type const& t, TraceDirection trace_dir) const override
      {
        // Evaluate pspace curve -- result in homogeneous coords ^^,
        const auto meval = curve::evaluateAtParent(m_mcurve, t, trace_dir);

        // Make pspace curve result not homeogeneous coords
        const auto uv = blaze::subvector<0UL, MVectorDim>(meval);

        // Evaluate surface along pspace curve
        const auto eval = polygonsurface::evaluateLocal(m_esurface, uv, {});

        // S(C)
        return eval[0];
      }

      std::optional<VectorH>
      derivativeAt(size_t order, Type const& t,
                   TraceDirection trace_dir) const override
      {
        auto fm = [this, trace_dir](Type par) {
          return m_mcurve.evaluateAt(par, trace_dir);
        };

        auto fe = [this](MPoint par) { return m_esurface.evaluateAt(par); };

        static constexpr numericdiff::
          DirectionalDerivativeForMappingMappingKernel<
            numericdiff::finitediff::CentralFD, MappingCurve, EmbedSurface>
             D{};

        VectorH res;
        if (order == 1)
          res = D(t, 1e-5, fm, fe);
        else
          res = VectorH(0);

        return {res};
      }
    };



  }   // namespace mappingkernel

  template <template <typename /*Expands to MappingSpace*/>
            typename MappingSpaceObject_T,
            typename EmbedSpace_T>
  using CurveInTensorProductSurface = MappingMappingObject<
    mappingkernel::CurveInTensorProductSurfaceMappingKernel,
    MappingSpaceObject_T, EmbedSpace_T>;




  template <template <typename /*Expands to MappingSpace*/>
            typename MappingSpaceObject_T,
            typename EmbedSpace_T>
  using CurveInPolygonalSurface
    = MappingMappingObject<mappingkernel::CurveInPolygonalSurfaceMappingKernel,
                           MappingSpaceObject_T, EmbedSpace_T>;



  //  namespace evaluationctrl
  //  {

  //    //////////SubCurveEvalCtrl////////
  //    /// evaluate(...)
  //    /// SubCurve in Curve
  //    ///
  //    template <typename SubParametricObject_T>
  //    struct SubCurveInCurveEvalCtrl {

  //      GM2_DEFINE_DEFAULT_PARAMETRIC_SUBOBJECT_EVALUATIONCTRL_TYPES


  //      using PSpaceCurve     = PSpaceObject;
  //      using ParametricCurve = PObject;

  //      static EvaluationResult
  //      evaluate(const PSpaceCurve& pspace_curve, const ParametricCurve*
  //      pcurve,
  //               const typename PSpaceCurve::PSpacePoint&     par,
  //               const typename PSpaceCurve::PSpaceSizeArray& no_der,
  //               const typename PSpaceCurve::PSpaceBoolArray& /*from_left*/)
  //      {
  //        // Evaluate pspace curve -- result in homogeneous coords ^^,
  //        const auto pspace_eval
  //          = pspace_curve.evaluateParent(par, no_der, {{true}});

  //        // Make pspace curve result not homeogeneous coords
  //        const auto pcurve_par
  //          = blaze::subvector<0UL, ParametricCurve::PSpaceVectorDim>(
  //            pspace_eval[0UL]);

  //        // Evaluate surface along pspace curve
  //        const auto pcurve_eval
  //          = pcurve->evaluateLocal(pcurve_par, no_der, {{true}});

  //        // Copy the result and scale the derivatives.
  //        // The scaling is not (yet) sane for no_der > 1.
  //        EvaluationResult p(pcurve_eval);


  //        auto d_scale{0.0};
  //        for (size_t i{1}; i <= no_der[0]; ++i) {
  //          d_scale
  //            += blaze::length(
  //                 blaze::subvector<0UL,
  //                 PSpaceCurve::VectorDim>(pspace_eval[i]))
  //               / (pcurve->endParameters()[0] -
  //               pcurve->startParameters()[0]);
  //          p[i] *= d_scale;
  //        }

  //        return p;
  //      }
  //    };




  //    ////////////////////
  //    /// evaluate(...)
  //    /// SubCurve in Surface
  //    ///
  //    template <typename SubParametricObject_T>
  //    struct SubCurveInSurfaceEvalCtrl {

  //      GM2_DEFINE_DEFAULT_PARAMETRIC_SUBOBJECT_EVALUATIONCTRL_TYPES


  //      using PSpaceCurve       = PSpaceObject;
  //      using ParametricSurface = PObject;

  //      static EvaluationResult
  //      evaluate(const PSpaceCurve&                           pspace_curve,
  //               const ParametricSurface*                     psurface,
  //               const typename PSpaceCurve::PSpacePoint&     par,
  //               const typename PSpaceCurve::PSpaceSizeArray& no_der,
  //               const typename PSpaceCurve::PSpaceBoolArray& /*from_left*/)
  //      {

  //        // Evaluate pspace curve -- result in homogeneous coords ^^,
  //        const auto pspace_eval
  //          = pspace_curve.evaluateParent(par, {{0}}, {{true}});

  //        // Make pspace curve result not homeogeneous coords
  //        const auto psurface_par
  //          = blaze::subvector<0UL, ParametricSurface::PSpaceVectorDim>(
  //            pspace_eval[0UL]);

  //        // Evaluate surface along pspace curve
  //        const auto psurface_eval
  //          = psurface->evaluateLocal(psurface_par, {{0, 0}}, {{true, true}});


  //        // Results
  //        EvaluationResult p(no_der[0] + 1);

  //        // S(C)
  //        p[0] = PObjEvalCtrl::toPositionH(psurface_eval);

  //        // D_DtC S(C)
  //        if (no_der[0] > 0) {
  //          const auto pspace_d = typename PSpaceCurve::PSpaceVector{1e-6};
  //          const auto [Dc_psurface, Dc_psurface_scale]
  //            = D(pspace_curve, par, pspace_d, *psurface);

  //          p[1] = Dc_psurface * Dc_psurface_scale;
  //        }

  //        return p;
  //      }

  //    private:
  //      // Differntial operator
  //      static constexpr gm::parametric::approximation::
  //        SubObjectDirectionalDerivativeLocal<>
  //          D{};
  //    };



  //    ////////////////////
  //    /// evaluate(...)
  //    /// SubCurve in Volume
  //    ///
  //    template <typename SubParametricObject_T>
  //    struct SubCurveInVolumeEvalCtrl {

  //      GM2_DEFINE_DEFAULT_PARAMETRIC_SUBOBJECT_EVALUATIONCTRL_TYPES


  //      using PSpaceCurve      = PSpaceObject;
  //      using ParametricVolume = PObject;

  //      static EvaluationResult
  //      evaluate(const PSpaceCurve& pspace_curve, const ParametricVolume*
  //      pvolume,
  //               const typename PSpaceCurve::PSpacePoint&     par,
  //               const typename PSpaceCurve::PSpaceSizeArray& no_der,
  //               const typename PSpaceCurve::PSpaceBoolArray& /*from_left*/)
  //      {

  //        // Evaluate pspace curve -- result in homogeneous coords ^^,
  //        const auto pspace_eval
  //          = pspace_curve.evaluateParent(par, {{0}}, {{true}});

  //        // Make pspace curve result not homeogeneous coords
  //        const auto pvolume_par
  //          = blaze::subvector<0UL, ParametricVolume::PSpaceVectorDim>(
  //            pspace_eval[0UL]);

  //        // Evaluate volume along pspace curve
  //        const auto pvolume_eval
  //          = pvolume->evaluateLocal(pvolume_par, {{0, 0}}, {{true, true}});


  //        // Results
  //        EvaluationResult p(no_der[0] + 1);

  //        // V(C)
  //        p[0] = PObjEvalCtrl::toPositionH(pvolume_eval);

  //        // D_DtC V(C)
  //        if (no_der[0] > 0) {
  //          const auto pspace_d = typename PSpaceCurve::PSpaceVector{1e-6};
  //          const auto [Dc_pvolume, Dc_pvolume_scale]
  //            = D(pspace_curve, par, pspace_d, *pvolume);

  //          p[1] = Dc_pvolume * Dc_pvolume_scale;
  //        }
  //        return p;
  //      }

  //    private:
  //      // Differntial operator
  //      static constexpr gm::parametric::approximation::
  //        SubObjectDirectionalDerivativeLocal<>
  //          D{};
  //    };

  //  }   // namespace evaluationctrl




  //  ////////////////////////////////
  //  /// Parametric SubCurve In Curve

  //  template <
  //    template <typename, template <typename> typename> typename
  //    PSpaceCurve_T, typename PObject_T, typename BaseSpaceObjectBase_T =
  //    typename PObject_T::BaseSpaceObjectBase, template <typename> typename
  //    PSubObjEvalCtrl_T = evaluationctrl::SubCurveInCurveEvalCtrl, template
  //    <typename> typename PSpaceObject_PObjEvalCtrl_T =
  //    evaluationctrl::CurveEvalCtrl, template <typename> typename
  //    PSubObj_PObjEvalCtrl_T = evaluationctrl::CurveEvalCtrl>
  //  using SubCurveInCurve
  //    = ParametricSubObject<PSpaceCurve_T, PObject_T, PSubObjEvalCtrl_T,
  //                          PSpaceObject_PObjEvalCtrl_T,
  //                          PSubObj_PObjEvalCtrl_T, BaseSpaceObjectBase_T>;

  //  //////////////////////////////////
  //  /// Parametric SubCurve In Surface
  //  ///
  //  template <
  //    template <typename, template <typename> typename> typename
  //    PSpaceCurve_T, typename PObject_T, typename BaseSpaceObjectBase_T =
  //    typename PObject_T::BaseSpaceObjectBase, template <typename> typename
  //    PSubObjEvalCtrl_T = evaluationctrl::SubCurveInSurfaceEvalCtrl, template
  //    <typename> typename PSpaceObject_PObjEvalCtrl_T =
  //    evaluationctrl::CurveEvalCtrl, template <typename> typename
  //    PSubObj_PObjEvalCtrl_T = evaluationctrl::CurveEvalCtrl>
  //  using SubCurveInSurface
  //    = ParametricSubObject<PSpaceCurve_T, PObject_T, PSubObjEvalCtrl_T,
  //                          PSpaceObject_PObjEvalCtrl_T,
  //                          PSubObj_PObjEvalCtrl_T, BaseSpaceObjectBase_T>;

  //  //////////////////////////////////
  //  /// Parametric SubCurve In Volume
  //  ///
  //  template <
  //    template <typename, template <typename> typename> typename
  //    PSpaceCurve_T, typename PObject_T, typename BaseSpaceObjectBase_T =
  //    typename PObject_T::BaseSpaceObjectBase, template <typename> typename
  //    PSubObjEvalCtrl_T = evaluationctrl::SubCurveInVolumeEvalCtrl, template
  //    <typename> typename PSpaceObject_PObjEvalCtrl_T =
  //    evaluationctrl::CurveEvalCtrl, template <typename> typename
  //    PSubObj_PObjEvalCtrl_T = evaluationctrl::CurveEvalCtrl>
  //  using SubCurveInVolume
  //    = ParametricSubObject<PSpaceCurve_T, PObject_T, PSubObjEvalCtrl_T,
  //                          PSpaceObject_PObjEvalCtrl_T,
  //                          PSubObj_PObjEvalCtrl_T, BaseSpaceObjectBase_T>;


}   // namespace gm::parametric





#endif   // GM2_PARAMETRIC_SUBOBJECT_SUBCURVE_H
