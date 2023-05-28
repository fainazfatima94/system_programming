#ifndef GM2_PARAMETRIC_UTILS_BLENDINGSPLINEUTILS_H
#define GM2_PARAMETRIC_UTILS_BLENDINGSPLINEUTILS_H


#include "../../core/gm2_blaze.h"


namespace gm::parametric::blendingspline
{


  template <typename Unit_T>
  using BlendingSplineKnotVector = std::vector<Unit_T>;



  template <typename Unit_T>
  auto generateUniformKnotVector(Unit_T const& start, Unit_T const& end,
                                 size_t no_knots, bool is_closed)
  {
    auto const& s = start;
    auto const& e = end;
    auto const& N = no_knots;

    auto const& dt = (e - s) / (N - 1);

    BlendingSplineKnotVector<Unit_T> T(N + 2);

    for (auto i = 0UL; i < N; i++) T[i + 1] = s + i * dt;

    if (is_closed) {
      T[0]     = T[1] - (T[N] - T[N - 1]);
      T[N + 1] = T[N] + (T[2] - T[1]);
    }
    else {
      T[0]     = T[1];
      T[N + 1] = T[N];
    }

    return T;
  }


  namespace curve
  {
    template <typename BezierCurve_T, typename Curve_T>
    auto approximateLocalBezierCurves(const Curve_T* g, size_t N, size_t d)
    {

      using BPSpaceUnit = typename BezierCurve_T::PSpaceUnit;
      using LocalCurves = std::vector<BezierCurve_T*>;
      using KnotVector  = BlendingSplineKnotVector<BPSpaceUnit>;
      using ClosedType  = bool;

      using ReturnType  = std::tuple<LocalCurves, KnotVector, ClosedType>;


      ReturnType ret;

      auto& ret_l = std::get<0>(ret);
      auto& ret_t = std::get<1>(ret);
      auto& ret_c = std::get<2>(ret);
      //      auto& [ret_l,ret_t,ret_c] = ret;

      ret_c = g->isClosed()[0];

      if (ret_c) N++;


      // Generate knot-vector
//      const auto sp = g->startParameters()[0];
//      const auto dt = (g->endParameters()[0] - sp) / (N - 1);

//      ret_t.resize(N + 2);

//      for (auto i = 0UL; i < N; i++) ret_t[i + 1] = sp + i * dt;

//      if (ret_c) {
//        ret_t[0]     = ret_t[1] - (ret_t[N] - ret_t[N - 1]);
//        ret_t[N + 1] = ret_t[N] + (ret_t[2] - ret_t[1]);
//      }
//      else {
//        ret_t[0]     = ret_t[1];
//        ret_t[N + 1] = ret_t[N];
//      }

      ret_t = generateUniformKnotVector(g->startParameters()[0],
                                        g->endParameters()[0], N, ret_c);

      //      qDebug() << "Kont vector: " << ret_t;



      const auto makeLocal
        = [g, d](const auto& s, const auto& t, const auto& e) {
//            using BezControlPoints = std::vector<typename BezierCurve_T::Point>;
            using BezControlPoints = typename BezierCurve_T::ControlPoints;
            using BezPSpaceVector  = typename BezierCurve_T::PSpaceVector;

            const auto eval
              = g->evaluateLocal(typename Curve_T::PSpacePoint{t},
                                 typename Curve_T::PSpaceSizeArray{d});
            DVectorT<typename BezierCurve_T::Point> c(eval.size());
            for (auto i = 0UL; i < eval.size(); ++i)
              c[i] = blaze::evaluate(
                blaze::subvector<0UL, BezierCurve_T::VectorDim>(eval[i]));


            //        std::cout << std::endl;
            //        std::cout << "Make local" << std::endl;
            //        std::cout << "  g->eval:   " << eval << std::endl;
            //        std::cout << "  eval => c: " << c << std::endl;

            const auto scale = BPSpaceUnit(1) / (e - s);
            const auto B
              = blaze::evaluate(blaze::inv(basis::generateBernsteinBasisMatrix(
                int(eval.size() - 1), (t - s) / (e - s), scale)));



            BezControlPoints cps(eval.size());
            for (auto i = 0UL; i < cps.size(); ++i)
              cps[i] = blaze::row(B, i) * c;
            //        cps = B * c;

            //        std::cout << "  B * c => cps: " << cps << std::endl;
            for (auto& cp : cps) cp = blaze::evaluate(cp - c[0]);

            //        std::cout << "  cps - c[0] => cps: " << cps << std::endl;

            auto* local    = new BezierCurve_T(cps);
            local->m_scale = BezPSpaceVector{scale};
            local->translateParent(c[0]);

            return local;
          };

      // Generate local curves
      ret_l.resize(N);
      for (auto i = 1UL; i < N; i++) {
        ret_l[i - 1] = makeLocal(ret_t[i - 1], ret_t[i], ret_t[i + 1]);
      }

      // Handle Open/Closed
      if (ret_c)
        ret_l[N - 1] = ret_l[0];
      else {
        ret_l[N - 1] = makeLocal(ret_t[N - 1], ret_t[N], ret_t[N + 1]);
      }

      return ret;
    }

  }   // namespace curve


  namespace surface
  {


    template <typename BezierSurface_T, typename Surface_T>
    auto approximateLocalBezierPatches(const Surface_T*               g,
                                       std::array<size_t, 2UL>        N,
                                       const std::array<size_t, 2UL>& d)

    {

      using PSpaceUnit    = typename Surface_T::PSpaceUnit;
      using PSpacePoint   = typename Surface_T::PSpacePoint;
      using LocalSurfaces = std::vector<std::vector<BezierSurface_T*>>;
      using KnotVector    = std::vector<PSpaceUnit>;
      using ClosedType    = std::array<bool, 2UL>;

      using ReturnType
        = std::tuple<LocalSurfaces, KnotVector, KnotVector, ClosedType>;




      const auto genKnots = [](const PSpaceUnit& s, const PSpaceUnit& e,
                               const size_t& n, const bool& closed) {
        const auto dt = (e - s) / PSpaceUnit(n - 1);

        KnotVector t;
        t.resize(n + 2);

        for (auto i = 0UL; i < n; i++) t[i + 1] = s + i * dt;

        if (closed) {
          t[0]     = t[1] - (t[n] - t[n - 1]);
          t[n + 1] = t[n] + (t[2] - t[1]);
        }
        else {
          t[0]     = t[1];
          t[n + 1] = t[n];
        }

        return t;
      };

      const auto makeLocal = [g, d](const PSpacePoint& s, const PSpacePoint& t,
                                    const PSpacePoint& e) {
        using BezControlNet   = typename BezierSurface_T::ControlNet;
        using BezPSpaceVector = typename BezierSurface_T::PSpaceVector;

        const auto    eval = g->evaluateLocal(t, d);
        BezControlNet c(eval.rows(), eval.columns());
        for (auto i = 0UL; i < eval.rows(); ++i)
          for (auto j = 0UL; j < eval.columns(); ++j)
            c(i, j) = blaze::evaluate(
              blaze::subvector<0UL, BezierSurface_T::VectorDim>(eval(i, j)));



        const auto scale_u = PSpaceUnit(1) / (e[0] - s[0]);
        const auto scale_v = PSpaceUnit(1) / (e[1] - s[1]);

        const auto Bu
          = blaze::evaluate(blaze::inv(basis::generateBernsteinBasisMatrix(
            int(eval.rows() - 1), (t[0] - s[0]) / (e[0] - s[0]), scale_u)));
        const auto Bv
          = blaze::evaluate(blaze::inv(basis::generateBernsteinBasisMatrix(
            int(eval.columns() - 1), (t[1] - s[1]) / (e[1] - s[1]), scale_v)));


        BezControlNet cps = Bu * (c * blaze::trans(Bv));

        for (auto i = 0UL; i < cps.rows(); ++i)
          for (auto j = 0UL; j < cps.columns(); ++j)
            cps(i, j) = blaze::evaluate(cps(i, j) - c(0, 0));

        auto* local    = new BezierSurface_T(cps);
        local->m_scale = BezPSpaceVector{scale_u, scale_v};
        local->translateParent(c(0, 0));

        return local;
      };





      ReturnType ret;

      auto& ret_c      = std::get<0>(ret);
      auto& ret_u      = std::get<1>(ret);
      auto& ret_v      = std::get<2>(ret);
      auto& ret_closed = std::get<3>(ret);


      ret_closed = {g->isClosed()[0], g->isClosed()[1]};

      if (ret_closed[0]) N[0]++;
      if (ret_closed[1]) N[1]++;


      ret_u = genKnots(g->startParameters()[0], g->endParameters()[0], N[0],
                       ret_closed[0]);
      ret_v = genKnots(g->startParameters()[1], g->endParameters()[1], N[1],
                       ret_closed[1]);

      ret_c.resize(N[0]);
      for (auto& ret_ci : ret_c) ret_ci.resize(N[1]);

      size_t i, j;
      for (i = 0UL; i < N[0] - 1;
           i++) {   // locals should be -1, and then later handle the edges.
        for (j = 0; j < N[1] - 1; j++) {
          ret_c[i][j] = makeLocal(PSpacePoint{ret_u[i], ret_v[j]},
                                  PSpacePoint{ret_u[i + 1], ret_v[j + 1]},
                                  PSpacePoint{ret_u[i + 2], ret_v[j + 2]});
        }

        if (ret_closed[1])
          ret_c[i][j] = ret_c[i][0];
        else {
          ret_c[i][j] = makeLocal(PSpacePoint{ret_u[i], ret_v[j]},
                                  PSpacePoint{ret_u[i + 1], ret_v[j + 1]},
                                  PSpacePoint{ret_u[i + 2], ret_v[j + 2]});
        }
      }

      if (ret_closed[0])
        for (j = 0; j < N[1]; j++) ret_c[i][j] = ret_c[0][j];
      else {
        for (j = 0; j < N[1]; j++) {
          ret_c[i][j] = makeLocal(PSpacePoint{ret_u[i], ret_v[j]},
                                  PSpacePoint{ret_u[i + 1], ret_v[j + 1]},
                                  PSpacePoint{ret_u[i + 2], ret_v[j + 2]});
        }
      }

      return ret;
    }

  }   // namespace surface


}   // namespace gm::parametric::blendingspline




#endif   // GM2_PARAMETRIC_UTILS_BLENDINGSPLINEUTILS_H
