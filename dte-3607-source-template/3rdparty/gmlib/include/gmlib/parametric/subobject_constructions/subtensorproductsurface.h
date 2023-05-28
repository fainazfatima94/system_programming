#ifndef GM2_PARAMETRIC_SUBOBJECT_SUBTENSORPRODUCTSURFACE_H
#define GM2_PARAMETRIC_SUBOBJECT_SUBTENSORPRODUCTSURFACE_H

#include "../tensorproductsurface.h"
//#include "../volume.h"

#include "../mappingobjects.h"
#include "../utils/numeric_differentiation.h"


namespace gm::parametric
{

  namespace surfaceinsurface
  {
    template <typename PointH_T, typename MappingSurface_T,
              typename EmbedSurface_T>
    PointH_T
    evaluateAt(typename MappingSurface_T::DPoint const&         muv,
               typename MappingSurface_T::DTraceDirArray const& trace_dir,
               MappingSurface_T& msurface, EmbedSurface_T& esurface)
    {
      // Evaluate pspace curve -- result in homogeneous coords ^^,
      const auto meval
        = tensorproductsurface::evaluateAtParent(msurface, muv, trace_dir);

      // Make pspace curve result not homeogeneous coords
      const auto uv = blaze::subvector<0UL, EmbedSurface_T::DVectorDim>(meval);

      // Evaluate surface along pspace curve
      const auto eval = tensorproductsurface::evaluateAtLocal(esurface, uv);

      // S(C)
      return eval;
    }

  }   // namespace surfaceinsurface


  namespace mappingkernel
  {

    template <template <typename /*EmbedSpace (MappingSpace) */>
              typename MappingSpaceObject_T,
              typename EmbedSpace_T>
    struct TPSurfaceInTensorProductSurfaceMappingKernel
      : MappingObject<TensorProductSurfaceMappingKernel, EmbedSpace_T>

    {

      /***
       * Boiler plate */

      /* Base */
      using Base
        = MappingObject<TensorProductSurfaceMappingKernel, EmbedSpace_T>;
      using Type = typename Base::Type;



      using ESO
        = MappingObject<TensorProductSurfaceMappingKernel, EmbedSpace_T>;
      using MSO = MappingSpaceObject_T<typename ESO::DomainSpace>;

      GM2_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MSO, ESO)

    /* Interval Meta */
    using TraceDirection = typename Parameterization::TraceDirection;
    using DTraceDirArray = typename Parameterization::TraceDirArray;


      /*** Boiler plate END */


      using MappingSurface = MappingObjectType;
      using EmbedSurface   = EmbedObjectType;

      // Members
      MappingSurface m_msurface;
      EmbedSurface&  m_esurface;


      // Constructor(s)
      template <typename... Ts>
      TPSurfaceInTensorProductSurfaceMappingKernel(EmbedSurface& esurface,
                                                   Ts&&... ts)
        : m_msurface(std::forward<Ts>(ts)...), m_esurface{esurface}
      {
      }



      // Methods
      DomainInfo const& domain() const override { return m_msurface.domain(); }

      PointH evaluateAt(DPoint const& uv, DTraceDirArray const& trace_dir = {
                                            TraceDirection::LeftToRight,
                                            TraceDirection::LeftToRight,
                                          }) const override
      {
        return surfaceinsurface::evaluateAt<PointH>(uv, trace_dir, m_msurface,
                                                    m_esurface);
      }

      std::optional<VectorH>
      partialDerivativeAt(DSizeArray const& order, DPoint const& uv,
                          DTraceDirArray const& trace_dir = {
                            TraceDirection::LeftToRight,
                            TraceDirection::LeftToRight,
                          }) const override
      {
        auto fm = [this, trace_dir](DPoint par) {
          return m_msurface.evaluateAt(par, trace_dir);
        };

        auto fe = [this](MPoint par) { return m_esurface.evaluateAt(par); };

        static constexpr numericdiff::
          DirectionalDerivativeForMappingMappingKernel<
            numericdiff::finitediff::CentralFD, MappingSurface, EmbedSurface>
             D{};

        auto const& [order_u, order_v] = order;

        VectorH res;
        if (order_u == 1 and order_v == 0)
          res = D(uv, MVector{1e-5, 0}, fm, fe);
        else if (order_u == 0 and order_v == 1)
          res = D(uv, MVector{0, 1e-5}, fm, fe);
        else
          res = VectorH(0);

        return {res};
      }
    };







//    template <gmc::Surface MappingObject_T, gmc::Surface EmbedObject_T>
//    struct SurfaceInSurface : SpaceObject<typename EmbedObject_T::EmbedSpace> {

//      /*** Boiler plate */

//      /* Type, base class and parametric space */
//      using Base = SpaceObject<typename EmbedObject_T::EmbedSpace>;
//      using Type = typename Base::Type;

//      GM2_SURFACE_IN_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MappingObject_T,
//                                                         EmbedObject_T)
//      /*** Boiler plate END */


//      using MappingSurface = MappingObject;
//      using EmbedSurface   = EmbedObject;


//      template <typename... Ts>
//      explicit SurfaceInSurface(EmbedSurface& esurface, Ts&&... ts)
//        : Base(), m_msurface(std::forward<Ts>(ts)...), m_esurface{esurface}
//      {
//      }

//      // members

//      MappingSurface m_msurface;
//      EmbedSurface&  m_esurface;

//      MappingSurface&       mappingObject() { return m_msurface; }
//      EmbedSurface&         embedObject() { return m_esurface; }
//      MappingSurface const& mappingObject() const { return m_msurface; }
//      EmbedSurface const&   embedObject() const { return m_esurface; }


//      PointH evaluateAt(Type const& u, Type const& v, bool from_left_u,
//                        bool from_left_v) const
//      {
//        return surfaceinsurface::evaluateAt<PointH>(
//          u, v, from_left_u, from_left_v, m_msurface, m_esurface);
//      }


//      template <size_t OrderU_T, size_t OrderV_T>
//        // return value
//        VectorH
//        // function declaration
//        derivativeAt(Type const& u, Type const& v, bool from_left_u = true,
//                     bool from_left_v = true) const
//        // type requirements
//        requires
//        //
//        gmc::proto::OrderBoundTo<OrderU_T, 0ul, 1ul>and gmc::proto::
//          OrderBoundTo<OrderV_T, 0ul, 1ul> and
//        //
//        (not(gmc::proto::OrderEqualTo<
//               OrderU_T, 0ul> and gmc::proto::OrderEqualTo<OrderV_T, 0ul>))
//      // function definition
//      {
////        auto fm = [this, from_left_u, from_left_v](MPoint par) {
////          return m_msurface.evaluateAt(par[0], par[1], from_left_u,
////                                       from_left_v);
////        };

////        auto fe = [this, from_left_u, from_left_v](MPoint par) {
////          return m_esurface.evaluateAt(par[0], par[1], from_left_u,
////                                       from_left_v);
////        };

////        static constexpr numericdiff::
////          DirectionalDerivativeForMappingMappingKernel<
////            numericdiff::fdkernel::FD_CentralDifference<>, MappingSurface,
////            EmbedSurface>
////            D{};

////        if constexpr (OrderU_T == 1ul and OrderV_T == 0ul) {
////          return D(DPoint{u, v}, DVector{1e-6, 0}, fm, fe);
////        }

////        else if constexpr (OrderU_T == 0ul and OrderV_T == 1ul) {
////          return D(DPoint{u, v}, DVector{1e-6, 0}, fm, fe);
////        }
////        else {
//          return VectorH(0);
////        }
//      }










//      EvaluationResult evaluate(DPoint const& par, DSizeArray const& no_der,
//                                DBoolArray const& /*from_left*/) const
//      {

//        // Evaluate pspace curve -- result in homogeneous coords ^^,
//        const auto meval
//          = surface::evaluateParent(m_msurface, par, {{0, 0}}, {{true, true}});

//        // Make pspace curve result not homeogeneous coords
//        const auto mpar = blaze::subvector<0UL, MVectorDim>(meval(0UL, 0UL));

//        // Evaluate surface along pspace curve
//        const auto psurface_eval
//          = surface::evaluateLocal(m_esurface, mpar, {{0, 0}}, {{true, true}});


//        // Results
//        EvaluationResult p(no_der[0] + 1, no_der[1] + 1);

//        // S(S0)
//        //        p(0, 0) = PObjEvalCtrl::toPositionH(psurface_eval);
//        p(0, 0) = psurface_eval(0, 0);

//        // D_DuS0 S(S0)
//        if (no_der[0] > 0) {
//          //          const auto pspace_d0 = typename
//          //          PSpaceSurface::PSpaceVector{1e-6, 0}; const auto
//          //          [Ds0_psurface, Ds0_psurface_scale]
//          //            = D(m_pspace_surface, par, pspace_d0, m_surface);

//          //          p(1, 0) = Ds0_psurface * Ds0_psurface_scale;
//          p(1, 0) = typename EvaluationResult::ElementType(0);
//        }

//        // D_DvS0 S(S0)
//        if (no_der[1] > 0) {
//          //          const auto pspace_d1 = typename
//          //          PSpaceSurface::PSpaceVector{0, 1e-6}; const auto
//          //          [Ds1_psurface, Ds1_psurface_scale]
//          //            = D(m_pspace_surface, par, pspace_d1, m_surface);

//          //          p(0, 1) = Ds1_psurface * Ds1_psurface_scale;
//          p(0, 1) = typename EvaluationResult::ElementType(0);
//        }

//        // D_DuvS0 S(S0)
//        if (no_der[0] > 0 and no_der[1] > 0)
//          p(1, 1) = typename EvaluationResult::ElementType(0);

//        return p;
//      }

//    private:
//      // Differntial operator
//      static constexpr gm::parametric::approximation::
//        SubObjectDirectionalDerivativeLocal<>
//          D{};
//    };


  }   // namespace mappingkernel



  template <template <typename /*Expands to MappingSpace*/>
            typename MappingSpaceObject_T,
            typename EmbedSpace_T>
  using TPSurfaceInTensorProductSurface = MappingMappingObject<
    mappingkernel::TPSurfaceInTensorProductSurfaceMappingKernel,
    MappingSpaceObject_T, EmbedSpace_T>;




  //  template <template <typename, size_t, typename> typename
  //  SubDomainKernel_T,
  //            typename ParametricObject_T>
  //  using SubSurfaceInSurface
  //    = ParametricSubObject<SubDomainKernel_T, kernel::SubSurfaceInSurface,
  //                          ParametricObject_T>;


//  template <template <gmc::spaces::Space /*EmbedSpace*/>
//            typename MappingKernel_T,
//            gmc::Surface EmbedObject_T>
//  using SurfaceInSurface
//    = MappingMappingObject<MappingKernel_T, mappingkernel::SurfaceInSurface,
//                           EmbedObject_T>;

  //  template <template <typename, size_t, typename> typename MappingKernel_T,
  //            typename EmbedObject_T>
  //  using SurfaceInSurface
  //    = MappingMappingObject<MappingKernel_T, mappingkernel::SurfaceInSurface,
  //                           EmbedObject_T>;


  //  namespace evaluationctrl
  //  {

  //    ////////////////////
  //    /// evaluate(...)
  //    /// SubSurface in Surface
  //    ///
  //    template <typename SubParametricObject_T>
  //    struct SubSurfaceInSurfaceEvalCtrl {

  //      GM2_DEFINE_DEFAULT_PARAMETRIC_SUBOBJECT_EVALUATIONCTRL_TYPES


  //      using PSpaceSurface     = PSpaceObject;
  //      using ParametricSurface = PObject;


  //      static EvaluationResult
  //      evaluate(const PSpaceSurface& pspace_surface,
  //               const ParametricSurface*                       psurface,
  //               const typename PSpaceSurface::PSpacePoint&     par,
  //               const typename PSpaceSurface::PSpaceSizeArray& no_der,
  //               const typename PSpaceSurface::PSpaceBoolArray& /*from_left*/)
  //      {
  //        // Evaluate pspace curve -- result in homogeneous coords ^^,
  //        const auto pspace_eval
  //          = pspace_surface.evaluateParent(par, {{0, 0}}, {{true, true}});

  //        // Make pspace curve result not homeogeneous coords
  //        const auto psurface_par
  //          = blaze::subvector<0UL, ParametricSurface::PSpaceVectorDim>(
  //            pspace_eval(0UL, 0UL));

  //        // Evaluate surface along pspace curve
  //        const auto psurface_eval
  //          = psurface->evaluateLocal(psurface_par, {{0, 0}}, {{true, true}});


  //        // Results
  //        EvaluationResult p(no_der[0] + 1, no_der[1] + 1);

  //        // S(S0)
  //        p(0, 0) = PObjEvalCtrl::toPositionH(psurface_eval);

  //        // D_DuS0 S(S0)
  //        if (no_der[0] > 0) {
  //          const auto pspace_d0 = typename PSpaceSurface::PSpaceVector{1e-6,
  //          0}; const auto [Ds0_psurface, Ds0_psurface_scale]
  //            = D(pspace_surface, par, pspace_d0, *psurface);

  //          p(1, 0) = Ds0_psurface * Ds0_psurface_scale;
  //        }

  //        // D_DvS0 S(S0)
  //        if (no_der[1] > 0) {
  //          const auto pspace_d1 = typename PSpaceSurface::PSpaceVector{0,
  //          1e-6}; const auto [Ds1_psurface, Ds1_psurface_scale]
  //            = D(pspace_surface, par, pspace_d1, *psurface);

  //          p(0, 1) = Ds1_psurface * Ds1_psurface_scale;
  //        }

  //        // D_DuvS0 S(S0)
  //        if (no_der[0] > 0 and no_der[1] > 0)
  //          p(1, 1) = typename EvaluationResult::ElementType(0);

  //        return p;
  //      }

  //    private:
  //      // Differntial operator
  //      static constexpr gm::parametric::approximation::
  //        SubObjectDirectionalDerivativeLocal<>
  //              D{};
  //    };



  //    ////////////////////
  //    /// evaluate(...)
  //    /// SubSurface in Volume
  //    ///
  //    template <typename SubParametricObject_T>
  //    struct SubSurfaceInVolumeEvalCtrl {

  //      GM2_DEFINE_DEFAULT_PARAMETRIC_SUBOBJECT_EVALUATIONCTRL_TYPES


  //      using PSpaceSurface    = PSpaceObject;
  //      using ParametricVolume = PObject;


  //      static EvaluationResult
  //      evaluate(const PSpaceSurface& pspace_surface,
  //               const ParametricVolume*                        pvolume,
  //               const typename PSpaceSurface::PSpacePoint&     par,
  //               const typename PSpaceSurface::PSpaceSizeArray& no_der,
  //               const typename PSpaceSurface::PSpaceBoolArray& /*from_left*/)
  //      {
  //        // Evaluate pspace curve -- result in homogeneous coords ^^,
  //        const auto pspace_eval
  //          = pspace_surface.evaluateParent(par, {{0, 0}}, {{true, true}});

  //        // Make pspace surface result not homeogeneous coords
  //        const auto pvolume_par
  //          = blaze::subvector<0UL, PSpaceSurface::VectorDim>(
  //            pspace_eval(0UL, 0UL));

  //        // Evaluate surface along pspace curve
  //        const auto pvolume_eval = pvolume->evaluateLocal(
  //          pvolume_par, {{0, 0, 0}}, {{true, true, true}});



  //        // Results
  //        EvaluationResult p(no_der[0] + 1, no_der[1] + 1);

  //        // V(S)
  //        p(0, 0) = PObjEvalCtrl::toPositionH(pvolume_eval);

  //        // D_DuS V(S)
  //        if (no_der[0] > 0) {
  //          const auto pspace_d0 = typename PSpaceSurface::PSpaceVector{1e-6,
  //          0}; const auto [Ds0_pvolume, Ds0_pvolume_scale]
  //            = D(pspace_surface, par, pspace_d0, *pvolume);

  //          p(1, 0) = Ds0_pvolume * Ds0_pvolume_scale;
  //        }

  //        // D_DvS V(S)
  //        if (no_der[1] > 0) {
  //          const auto pspace_d1 = typename PSpaceSurface::PSpaceVector{0,
  //          1e-6}; const auto [Ds1_pvolume, Ds1_pvolume_scale]
  //            = D(pspace_surface, par, pspace_d1, *pvolume);

  //          p(0, 1) = Ds1_pvolume * Ds1_pvolume_scale;
  //        }

  //        // D_DuvS V(S)
  //        if (no_der[0] > 0 and no_der[1] > 0)
  //          p(1, 1) = typename EvaluationResult::ElementType(0);

  //        return p;
  //      }

  //    private:
  //      // Differntial operator
  //      static constexpr approximation::SubObjectDirectionalDerivativeLocal<>
  //      D{};
  //    };


  //  }   // namespace evaluationctrl




  //  ////////////////////////////////////
  //  /// Parametric SubSurface in Surface

  //  template <
  //    template <typename, template <typename> typename> typename
  //    PSpaceSurface_T, typename PObject_T, typename BaseSpaceObjectBase_T =
  //    typename PObject_T::BaseSpaceObjectBase, template <typename> typename
  //    PSubObjEvalCtrl_T = evaluationctrl::SubSurfaceInSurfaceEvalCtrl,
  //    template <typename> typename PSpaceObject_PObjEvalCtrl_T
  //    = evaluationctrl::SurfaceEvalCtrl,
  //    template <typename> typename PSubObj_PObjEvalCtrl_T
  //    = evaluationctrl::SurfaceEvalCtrl>
  //  using SubSurfaceInSurface
  //    = ParametricSubObject<PSpaceSurface_T, PObject_T, PSubObjEvalCtrl_T,
  //                          PSpaceObject_PObjEvalCtrl_T,
  //                          PSubObj_PObjEvalCtrl_T, BaseSpaceObjectBase_T>;



  //  ////////////////////////////////////
  //  /// Parametric SubSurface in Volume

  //  template <
  //    template <typename, template <typename> typename> typename
  //    PSpaceSurface_T, typename PObject_T, typename BaseSpaceObjectBase_T =
  //    typename PObject_T::BaseSpaceObjectBase, template <typename> typename
  //    PSubObjEvalCtrl_T = evaluationctrl::SubSurfaceInVolumeEvalCtrl, template
  //    <typename> typename PSpaceObject_PObjEvalCtrl_T =
  //    evaluationctrl::SurfaceEvalCtrl, template <typename> typename
  //    PSubObj_PObjEvalCtrl_T = evaluationctrl::SurfaceEvalCtrl>
  //  using SubSurfaceInVolume
  //    = ParametricSubObject<PSpaceSurface_T, PObject_T, PSubObjEvalCtrl_T,
  //                          PSpaceObject_PObjEvalCtrl_T,
  //                          PSubObj_PObjEvalCtrl_T, BaseSpaceObjectBase_T>;

}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_SUBOBJECT_SUBTENSORPRODUCTSURFACE_H
