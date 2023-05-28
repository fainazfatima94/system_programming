#ifndef GM2_PARAMETRIC_POLYGONSURFACE_H
#define GM2_PARAMETRIC_POLYGONSURFACE_H

#include "mappingobjects.h"

#include "../core/polygonutils.h"
#include "../core/datastructures.h"
#include "utils/mappingobject_utils.h"

#include "../core/gbc/mvc.h"

// gmconcepts
#include <gmconcepts/manifolds.h>

// stl
#include <optional>
#include <vector>

namespace gm::parametric
{

  namespace mappingkernel
  {
    template <template <typename /*Space_T*/> typename GBCParameterization_T,
              typename EmbedSpace_T>
    struct PolygonalSurfaceMappingKernel : SpaceObject<EmbedSpace_T> {

      /***
       * Boiler plate */

      /* Base */
      using Base             = SpaceObject<EmbedSpace_T>;
      using EmbedSpaceObject = Base;
      using EmbedSpace       = typename EmbedSpaceObject::Space;
      using Type             = typename EmbedSpace::Type;

      /* Domain and types */
      using DomainSpace      = spaces::ProjectiveSpace<Type, 2ul>;
//      using Domain           = domain::MeanValueCoordinateDomain<DomainSpace>;
      using Domain           = domain::Domain<GBCParameterization_T<DomainSpace>>;
      using DomainInfo       = typename Domain::DomainInfo;
      using Parameterization = typename Domain::Parameterization;
      GM2_MAPPING_KERNEL_BOILER_PLATE



      /*** Boiler plate END */

      //    using Base::Base;

      using Base::Base;
      virtual ~PolygonalSurfaceMappingKernel() = default;

      virtual DomainInfo const& domain() const = 0;


      virtual PointH
      evaluateAtGBC(typename Domain::Lambda const& gbc) const = 0;

      virtual PointH evaluateAt(DPoint const& xy) const
      {
        return evaluateAtGBC(gbc::mvc(domain().polygon, xy));
      }

      virtual VectorH directionalDerivativeAt(DPoint const&  xy,
                                              DVector const& dir) const
      {
        static constexpr numericdiff::finitediff::CentralFD D;

        auto       fe = [this](DPoint p) { return evaluateAt(p); };
        auto const h  = 1e-6;
        auto const Ds = D(xy, blaze::normalize(dir) * h, fe);

        return Ds;
      }

      virtual std::vector<std::optional<VectorH>> directionalDerivativesAt(
        DPoint const& /*uv*/, std::vector<DVector> const& /*dirs*/
        //          ,
        //        DTraceDirArray const& /*trace_dir*/
        //        = {TraceDirection::LeftToRight, TraceDirection::LeftToRight}
      ) const
      {
        return {};
      }

      virtual std::optional<VectorH> directionalDerivativeAt(
        DPoint const& uv, std::vector<DVector> const& dirs
//          ,
//        DTraceDirArray const& trace_dir
//        = {TraceDirection::LeftToRight, TraceDirection::LeftToRight}
          ) const
      {
        auto const dds = directionalDerivativesAt(uv, dirs/*, trace_dir*/);
        if (dds.size() < dirs.size()) return {};

        auto const dd_no = dirs.size() - 1ul;
        return dds[dd_no];
      }

    };

    template <typename EmbedSpace_T>
    struct MVCPolygonalSurfaceMappingKernel
      : PolygonalSurfaceMappingKernel<
          domain::parameterization::gbc::MVCParameterization,EmbedSpace_T> {
      using Base = PolygonalSurfaceMappingKernel<
        domain::parameterization::gbc::MVCParameterization,EmbedSpace_T>;
      using Base::Base;
    };

  }   // namespace mappingkernel


  template <template <typename /*space*/> typename GBCPolygonalSurfaceKernel_T,
            gmc::spaces::Space EmbedSpace_T>
  using PolygonalSurface = MappingObject<GBCPolygonalSurfaceKernel_T, EmbedSpace_T>;


  //  template <gmc::spaces::Space EmbedSpace_T>
  //  using PolygonalSurface
  //    = MappingObject<mappingkernel::PolygonalSurfaceMappingKernel,
  //                    EmbedSpace_T>;



  namespace polygonsurface
  {

    namespace convex_algorithms
    {

      template <typename Unit_T>
      size_t
      countTriSamplingPSpacePositions(size_t                       rings,
                                      DVectorT<VectorT<Unit_T, 2>> polygon_2d)
      {
        const auto N = int(polygon_2d.size());
        const auto M = int(rings);

       int no_samples = 0;   // M == 0;
        if (M == 1)
          no_samples = 1;
        else {
          no_samples = 1;
          for (int k = 2; k <= M; ++k) no_samples += N * (k - 1);
        }

        return size_t(no_samples);
      }

      template <typename Type_T>
      auto generateTriSamplingPSpacePositions(
        size_t rings, DVectorT<VectorT<Type_T, 2>> polygon_2d)
      {
        using DSamplePoints = std::vector<VectorT<Type_T, 2>>;

        const auto N = int(polygon_2d.size());
        const auto M = int(rings);

        auto no_samples = countTriSamplingPSpacePositions(rings, polygon_2d);

        DSamplePoints pspace_p(no_samples);

        auto par = DVectorT<DVectorT<Type_T>>(
          size_t(N), DVectorT<Type_T>(size_t(N), Type_T(0)));
        for (auto i = 0; i < N; ++i) {
          par[size_t(i)][size_t(i)] = 1.0;
        }

        const auto center          = DVectorT<Type_T>(size_t(N), 1 / Type_T(N));
        const auto center_eval_par = blaze::inner(polygon_2d, center);

        pspace_p[0] = center_eval_par;

        for (int m = 0; m < M; ++m) {

          const int T = m - 1;

          int r_o;
          if (m == 0)
            r_o = 0;
          else if (m == 1)
            r_o = 1;
          else {
            r_o = 1;
            for (int k = 2; k <= m; ++k) {
              r_o += N * (k - 1);
            }
          }

          for (int n = 0; n < (m == 0 ? 1 : N); ++n) {

            const int t_o = r_o + n * T + n;



            for (int t_i = 0; t_i <= T; ++t_i) {

              const int idx = t_o + t_i;

              const double u = 1.0 - ((1.0 / double(M - 1)) * m);
              const double w = (1.0 / double(M - 1)) * t_i;
              const double v = 1.0 - (u + w);

              const auto poly_par
                = blaze::evaluate(center * u + par[size_t(n)] * v
                                  + par[size_t(n == N - 1 ? 0 : n + 1)] * w);

              pspace_p[size_t(idx)] = blaze::inner(polygon_2d, poly_par);
            }
          }
        }

        return pspace_p;
      }


      template <typename Unit_T>
      size_t countTriSamplingFaceIndices(size_t                       rings,
                                       DVectorT<VectorT<Unit_T, 2>> polygon_2d)
      {
        const auto N = polygon_2d.size();
        const auto M = rings;

        if (M == 2UL) return N;

        size_t no_faces = 1ul;
        for (size_t k = 3ul; k <= M; ++k) no_faces += 1 + (k - 2) * 2;
        no_faces *= N;

        return no_faces;
      }

      template <typename Unit_T>
      auto
      generateTriSamplingFaceIndices(size_t                       rings,
                                     DVectorT<VectorT<Unit_T, 2>> polygon_2d)
      {

        const auto no_tris = countTriSamplingFaceIndices(rings, polygon_2d);

        std::vector<std::array<size_t, 3>> tris(no_tris);

        const auto N = int(polygon_2d.size());
        const auto M = int(rings);

        for (int m = 0; m <= M - 1; ++m) {

          int T_prev = m - 2;
          int T      = m - 1;

          int r_o_prev;
          if (m - 1 == 0)
            r_o_prev = 0;
          else if (m - 1 == 1)
            r_o_prev = 1;
          else {
            r_o_prev = 1;
            for (int k = 2; k <= m - 1; ++k) {
              r_o_prev += N * (k - 1);
            }
          }

          int r_o;
          if (m == 0)
            r_o = 0;
          else if (m == 1)
            r_o = 1;
          else {
            r_o = 1;
            for (int k = 2; k <= m; ++k) {
              r_o += N * (k - 1);
            }
          }

          int f_T = 2 * (m - 1);

          int f_o = 0;
          if (m > 0) {
            for (int k = 0; k <= m - 2; ++k) f_o += N * (2 * (k) + 1);
          }


          for (int n = 0; n < (m == 0 ? 1 : N); ++n) {

            int t_o_prev = r_o_prev + n * T_prev + n;
            int t_o      = r_o + n * T + n;

            int f_t_o = f_o + n * f_T + n;


            for (int t_i = 0; t_i <= T; ++t_i) {

              //          const int idx = t_o + t_i;
              const int f_t_i = f_t_o + t_i * 2;


              if (t_i == T and n == N - 1) {

                const int i0_0 = r_o_prev;
                const int i0_1 = t_o + t_i;
                const int i0_2 = r_o;
                tris[size_t(f_t_i)]
                  = {size_t(i0_0), size_t(i0_1), size_t(i0_2)};
              }
              else if (t_i == T) {

                const int i0_0 = t_o_prev + t_i;
                const int i0_1 = t_o + t_i;
                const int i0_2 = t_o + t_i + 1;
                tris[size_t(f_t_i)]
                  = {size_t(i0_0), size_t(i0_1), size_t(i0_2)};
              }
              else {

                const int i0_0 = t_o_prev + t_i;
                const int i0_1 = t_o + t_i;
                const int i0_2 = t_o + t_i + 1;

                const int i1_0 = t_o_prev + t_i;
                const int i1_1 = t_o + t_i + 1;
                const int i1_2 = t_o_prev + t_i + 1;

                tris[size_t(f_t_i)]
                  = {size_t(i0_0), size_t(i0_1), size_t(i0_2)};
                if (n == N - 1 and t_i == T - 1) {
                  tris[size_t(f_t_i + 1)]
                    = {size_t(i1_0), size_t(i1_1), size_t(r_o_prev)};
                }
                else {

                  tris[size_t(f_t_i + 1)]
                    = {size_t(i1_0), size_t(i1_1), size_t(i1_2)};
                }
              }
            }
          }
        }

        return tris;
      }

    }   // namespace convex_algorithms


    template <gmc::PolygonalSurface PolygonalSurface_T>
    typename PolygonalSurface_T::EvaluationResult
    evaluateLocal(PolygonalSurface_T const&                      obj,
                  typename PolygonalSurface_T::DPoint const&     par,
                  typename PolygonalSurface_T::DSizeArray const& no_der,
                  typename PolygonalSurface_T::DBoolArray const& from_left)
    {
      //      return obj.evaluate(par, no_der, from_left);


      using EvaluationResult =
        typename datastructures::parametrics::polygonalsurface::
          EvaluationResult<typename PolygonalSurface_T::VectorH>;

      // DEV HACK -- start (hack reason: unsafe)
      EvaluationResult res(no_der[0] + 1, no_der[1] + 1);
      res(0, 0) = obj.evaluateAt(par);
      if (no_der[0] > 0)
        res(1, 0)
          = obj.derivativeAt(par, typename PolygonalSurface_T::DVector{1, 0});
      if (no_der[1] > 0)
        res(0, 1)
          = obj.derivativeAt(par, typename PolygonalSurface_T::DVector{0, 1});
      if (no_der[0] > 0 and no_der[1] > 0)
        res(1, 1) = typename PolygonalSurface_T::VectorH{0, 0, 0, 0};
      // DEV HACK -- end

      return res;
    }

//    template <typename Object_T>
//    typename Object_T::EvaluationResult
//    evaluateParent(Object_T const&                           obj,
//                   typename Object_T::PSpacePoint const&     par,
//                   typename Object_T::PSpaceSizeArray const& no_derivatives,
//                   typename Object_T::PSpaceBoolArray const& from_left)
//    {
//      const auto eval_res = obj.evaluate(par, no_derivatives, from_left);
//      const auto pframe   = obj.pSpaceFrameParent();

//      typename Object_T::EvaluationResult res
//        = blaze::map(eval_res, [pframe](const auto& ele) {
//            return blaze::evaluate(pframe * ele);
//          });
//      return res;
//    }




    template <gmc::PolygonalSurface PolygonalSurface_T,
              typename SamplingPoints_T>
    datastructures::parametrics::polygonalsurface::SamplingResult<
      typename PolygonalSurface_T::VectorH>
    sample(PolygonalSurface_T const&                      pobj,
           SamplingPoints_T const&                        sample_positions,
           typename PolygonalSurface_T::DSizeArray const& no_derivatives)
    {
      using SamplingResult
        = datastructures::parametrics::polygonalsurface::SamplingResult<
          typename PolygonalSurface_T::VectorH>;

      SamplingResult p;
      p.resize(sample_positions.size());

      std::transform(std::begin(sample_positions), std::end(sample_positions),
                     std::begin(p),
                     [pobj, no_derivatives](const auto& sample_pspace_pos) {
                       return evaluateLocal(pobj, sample_pspace_pos,
                                            no_derivatives, {true, true});
                     });

      return p;
    }






    template <gmc::PolygonalSurface PolygonalSurface_T>
    datastructures::parametrics::polygonalsurface::EvaluationResultDVec<
      typename PolygonalSurface_T::VectorH>
    evaluateLocal(
      PolygonalSurface_T const&                                obj,
      typename PolygonalSurface_T::DPoint const&               par,
      std::vector<typename PolygonalSurface_T::DVector> const& D1_dirs)
    {
      using EvaluationResult =
        typename datastructures::parametrics::polygonalsurface::
          EvaluationResultDVec<typename PolygonalSurface_T::VectorH>;

      EvaluationResult res(D1_dirs.size() + 1);
      res[0] = obj.evaluateAt(par);

      for (auto i = 0ul; i < D1_dirs.size(); ++i)
//        res[i + 1] = obj.derivativeAt(par, D1_dirs[i]);
      res[i + 1] = mappingobject::utils::derivativeAt(obj, par, D1_dirs[i]);

      return res;
    }




    template <gmc::PolygonalSurface PolygonalSurface_T,
              typename SamplingPoints_T>
    datastructures::parametrics::polygonalsurface::SamplingResultDVec<
      typename PolygonalSurface_T::VectorH>
    sample(PolygonalSurface_T const& pobj,
           SamplingPoints_T const&   sample_positions,
           std::vector<typename PolygonalSurface_T::DVector> const& D1_dirs)
    {
      using SamplingResult
        = datastructures::parametrics::polygonalsurface::SamplingResultDVec<
          typename PolygonalSurface_T::VectorH>;

      SamplingResult p;
      p.resize(sample_positions.size());

      std::transform(std::begin(sample_positions), std::end(sample_positions),
                     std::begin(p),
                     [&pobj, D1_dirs](const auto& sample_pspace_pos) {
                       return evaluateLocal(pobj, sample_pspace_pos, D1_dirs);
                     });

      return p;
    }




    template <typename ParametricObject_T>
    auto debugSampleLambdas(
      const ParametricObject_T*                          pobj,
      const typename ParametricObject_T::SamplingPoints& sample_positions)
    {
      using DebugLambdas = typename ParametricObject_T::DebugLambdas;

      DebugLambdas p;
      p.resize(sample_positions.size());

      std::transform(std::begin(sample_positions), std::end(sample_positions),
                     std::begin(p), [pobj](const auto& sample_pspace_pos) {
                       return pobj->debugEvaluateLambda(sample_pspace_pos);
                     });
      return p;
    }

    template <typename ParametricObject_T>
    auto debugSampleSs(
      const ParametricObject_T*                          pobj,
      const typename ParametricObject_T::SamplingPoints& sample_positions)
    {
      using DebugSs = typename ParametricObject_T::DebugSs;

      DebugSs p;
      p.resize(sample_positions.size());

      std::transform(std::begin(sample_positions), std::end(sample_positions),
                     std::begin(p), [pobj](const auto& sample_pspace_pos) {
                       return pobj->debugEvaluateS(sample_pspace_pos);
                     });
      return p;
    }

    template <typename ParametricObject_T>
    auto debugSampleHs(
      const ParametricObject_T*                          pobj,
      const typename ParametricObject_T::SamplingPoints& sample_positions)
    {
      using DebugHs = typename ParametricObject_T::DebugHs;

      DebugHs p;
      p.resize(sample_positions.size());

      std::transform(std::begin(sample_positions), std::end(sample_positions),
                     std::begin(p), [pobj](const auto& sample_pspace_pos) {
                       return pobj->debugEvaluateH(sample_pspace_pos);
                     });
      return p;
    }




  }   // namespace ppolygonsurface

//  namespace evaluationctrl
//  {

//    struct polygonsurface_tag {
//    };

//    template <typename ParametricObject_T>
//    struct PolygonalSurfaceEvalCtrl {
//      GM2_DEFINE_DEFAULT_PARAMETRIC_OBJECT_EVALUATIONCTRL_TYPES


//      using EvaluationResult
//        = datastructures::parametrics::polygonsurface::EvaluationResult<
//          VectorH>;

//      using SamplingResult
//        = datastructures::parametrics::polygonsurface::SamplingResult<VectorH>;

//      using SamplingPoints
//        = datastructures::parametrics::polygonsurface::SamplingPoints<
//          PSpacePoint>;

//      static auto toPositionH(const EvaluationResult& result)
//      {
//        return result(0UL, 0UL);
//      }

//      static auto toPosition(const EvaluationResult& result)
//      {
//        return blaze::evaluate(
//          blaze::subvector<0UL, ParametricObject_T::VectorDim>(
//            toPositionH(result)));
//      }
//    };

//  }   // namespace evaluationctrl




//  template <typename SpaceObjectBase_T = ProjectiveSpaceObject<>,
//            template <typename> typename PolygonalSurfaceEvalCtrl_T
//            = evaluationctrl::PolygonalSurfaceEvalCtrl>
//  class PolygonalSurface
//    : public ParametricObject<spaces::ParameterVSpaceInfo<2>,
//                              evaluationctrl::polygonsurface_tag,
//                              SpaceObjectBase_T, PolygonalSurfaceEvalCtrl_T> {

//    using Base = ParametricObject<spaces::ParameterVSpaceInfo<2>,
//                                  evaluationctrl::polygonsurface_tag,
//                                  SpaceObjectBase_T, PolygonalSurfaceEvalCtrl_T>;

//  public:
//    GM2_DEFINE_DEFAULT_PARAMETRIC_OBJECT_TYPES

//    // Polygon construction types
//    using SamplingPoints = typename PObjEvalCtrl::SamplingPoints;

//  protected:
//    template <typename Element_T>
//    using PolygonContainer = DVectorT<Element_T>;

//  public:
//    using Lambda    = PolygonContainer<PSpaceUnit>;
//    using Lambdas   = PolygonContainer<Lambda>;
//    using Polygon2D = PolygonContainer<PSpacePoint>;


//    const Polygon2D& polygon2D() const { return m_polygon; }
//    size_t           sides() const { return m_polygon.size(); }

//    template <typename... Ts>
//    PolygonalSurface(Polygon2D&& polygon, Ts&&... ts)
//      : Base(std::forward<Ts>(ts)...),
//        m_polygon(std::forward<Polygon2D>(polygon))
//    {
//    }

//    template <typename... Ts>
//    PolygonalSurface(size_t N, Ts&&... ts)
//      : Base(std::forward<Ts>(ts)...),
//        m_polygon(polygonutils::generateRegularPolygon2D<>(N))
//    {
//    }

//    // Member(s)
//  public:
//    Polygon2D m_polygon;


//    using DebugS = std::vector<PSpaceUnit>;
//    using DebugH = std::vector<PSpaceUnit>;

//    using DebugLambdas = std::vector<Lambda>;
//    using DebugSs      = std::vector<DebugS>;
//    using DebugHs      = std::vector<DebugH>;


//    virtual Lambda debugEvaluateLambda(const PSpacePoint&) const
//    {
//      return Lambda();
//    }
//    virtual DebugS debugEvaluateS(const PSpacePoint&) const { return DebugS(); }
//    virtual DebugH debugEvaluateH(const PSpacePoint&) const { return DebugH(); }

//    // Allocator utilities
//  protected:
//    template <typename Element_T>
//    auto allocPolygonContainer(const Element_T& default_value
//                               = Element_T()) const
//    {
//      return PolygonContainer<Element_T>(sides(), default_value);
//    }

//    Polygon2D allocPolygon2D(const PSpacePoint& default_value
//                             = PSpacePoint()) const
//    {
//      return allocPolygonContainer<PSpacePoint>(default_value);
//    }
//    Lambda allocLambda(const PSpaceUnit& default_value = PSpaceUnit()) const
//    {
//      return allocPolygonContainer<PSpaceUnit>(default_value);
//    }
//    Lambdas allocLambdas(const Lambda& default_value) const
//    {
//      return allocPolygonContainer<Lambda>(default_value);
//    }
//  };

}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_POLYGONSURFACE_H
