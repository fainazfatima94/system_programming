#ifndef GM2_PARAMETRIC_UTILS_FITTING_H
#define GM2_PARAMETRIC_UTILS_FITTING_H

#include "bsplineutils.h"
#include "../../core/basisgenerators/bernsteinbasisgenerators.h"
#include "../../core/gm2_blaze.h"
#include "../../parametric/curve.h"
#include "../../parametric/surface.h"


namespace gm::parametric::fitting
{

  namespace detail
  {

    template <typename Unit_T>
    auto generateBezierLSqDesignMatrix(size_t no_samples, size_t degree)
    {
      // Fill Design matrix [|no_sampes|x|no_coefs|]
      DMatrixT<Unit_T> A(no_samples, degree + 1, Unit_T(0));
      const auto       dt = 1 / Unit_T(no_samples - 1);

      for (auto i = 0UL; i < no_samples; ++i) {   // For each sample point
        const auto t     = i * dt;
        const auto Bt    = basis::generateBernsteinBasisMatrix(int(degree), t);
        blaze::row(A, i) = blaze::row<0>(Bt);
      }

      return A;
    }

    template <typename Unit_T>
    auto generateBSplineLSqDesignMatrix(size_t no_samples, size_t no_coefs,
                                        size_t degree)
    {
      // Knot Vector
      const auto T
        = bspline::generateUniformKnotVector<Unit_T>(no_coefs, degree);

      // Fill Design matrix [|no_sampes|x|no_coefs|]
      DMatrixT<Unit_T> A(no_samples, no_coefs, Unit_T(0));
      const auto       dt = 1 / Unit_T(no_samples - 1);

      for (auto i = 0UL; i < no_samples; ++i) {   // For each sample point
        const auto t   = i * dt;
        const auto T_i = bspline::findKnotIndex(T, t, degree);
        const auto Bt  = basis::generateBSplineBasisMatrix(T, T_i, t, degree);
        for (auto j = 0UL; j < no_coefs; ++j)   // For each basis cover
          blaze::subvector(blaze::row(A, i), T_i - degree, degree + 1)
            = blaze::row<0>(Bt);
      }

      return A;
    }

  }   // namespace detail





  template <typename Curve_T>
  DVectorT<typename Curve_T::Point>
  fitBezierCurve(const Curve_T*                           pcurve,
                 const typename Curve_T::PSpaceSizeArray& no_samples = {50UL},
                 const typename Curve_T::PSpaceSizeArray& degree     = {3UL}

  )
  {
    // Typedefs
    using Unit = typename   Curve_T::Unit;

    const auto A
      = detail::generateBezierLSqDesignMatrix<Unit>(no_samples[0], degree[0]);

    // Extract sample points
    const auto pcurve_samples = curve::sample(
      pcurve, pcurve->startParameters(), pcurve->endParameters(), no_samples,
      typename Curve_T::PSpaceSizeArray{0UL});

    DVectorT<VectorT<Unit, 3>> P(no_samples[0]);
    std::transform(
      std::begin(pcurve_samples), std::end(pcurve_samples), std::begin(P),
      [](const auto& ele) { return blaze::subvector<0UL, 3UL>(ele[0]); });


    // Factorize QR decomposition
    DMatrixT<Unit, blaze::columnMajor> R;
    DMatrixT<Unit, blaze::rowMajor>    Q;
    blaze::qr(A, Q, R);
    assert(A == Q * R);

    // solve for new coefficients
    return blaze::inv(R) * (blaze::trans(Q) * P);
  }

  template <typename CoefType_T, typename Unit_T = double>
  const DVectorT<CoefType_T>
  fitBSplineCurve(DVectorT<CoefType_T>           P,
                  std::array<size_t, 1ul> const& no_coefs = {20UL},
                  std::array<size_t, 1ul> const& degree   = {3UL})
  {
    const auto A = detail::generateBSplineLSqDesignMatrix<Unit_T>(
      P.size(), no_coefs[0], degree[0]);

    // Factorize QR decomposition
    DMatrixT<Unit_T, blaze::columnMajor> R;
    DMatrixT<Unit_T, blaze::rowMajor>    Q;
    blaze::qr(A, Q, R);
    assert(A == Q * R);

    // solve for new coefficients
    const auto lsq_approx_cps = blaze::inv(R) * (blaze::trans(Q) * P);

    return blaze::evaluate(lsq_approx_cps);
  }


  template <typename   Curve_T>
  auto
  fitBSplineCurve(const   Curve_T*                           pcurve,
                  const typename   Curve_T::PSpaceSizeArray& no_coefs   = {20UL},
                  const typename   Curve_T::PSpaceSizeArray& no_samples = {50UL},
                  const typename   Curve_T::PSpaceSizeArray& degree     = {3UL}

  )
  {
    // Typedefs
    using Unit = typename   Curve_T::Unit;

    // Extract sample points
    const auto pcurve_samples = curve::sample(
      pcurve, pcurve->startParameters(), pcurve->endParameters(), no_samples,
      typename Curve_T::PSpaceSizeArray{0UL});

    DVectorT<VectorT<Unit, 3>> P(no_samples[0]);
    std::transform(
      std::begin(pcurve_samples), std::end(pcurve_samples), std::begin(P),
      [](const auto& ele) { return blaze::subvector<0UL, 3UL>(ele[0]); });

    return fitBSplineCurve(P,no_coefs,degree);
  }


  template <typename CoefType_T, typename Unit_T = double>
  DMatrixT<CoefType_T> fitBezierSurface(const DMatrixT<CoefType_T>&    P,
                                        const std::array<size_t, 2ul>& degree
                                        = {3UL, 3UL})
  {
    // Typedefs
    using Unit = Unit_T;

    // Generate design matrices
    const auto A
      = detail::generateBezierLSqDesignMatrix<Unit>(P.rows(), degree[0]);
    const auto B
      = detail::generateBezierLSqDesignMatrix<Unit>(P.columns(), degree[1]);

    // Factorize using QR decomposition
    DMatrixT<Unit, blaze::columnMajor> Ra, Rb;
    DMatrixT<Unit, blaze::rowMajor>    Qa, Qb;
    blaze::qr(A, Qa, Ra);
    assert(A == Qa * Ra);
    blaze::qr(B, Qb, Rb);
    assert(B == Qb * Rb);

    // QR
    // A x B^T = b --> minimize: || A x B^T - b ||^2
    // =>
    // Ra x Rb^T = Qa^T P Qb

    // solve for new coefficients
    const auto QaPQb          = blaze::trans(Qa) * P * Qb;
    const auto invRa          = blaze::inv(Ra);
    const auto RbT            = blaze::trans(Rb);
    const auto invRbT         = blaze::inv(RbT);
    const auto lsq_approx_cps = invRa * QaPQb * invRbT;

    return lsq_approx_cps;
  }





  template <typename Surface_T>
  DMatrixT<typename Surface_T::Point> fitBezierSurface(
    const Surface_T*                           psurface,
    const typename Surface_T::PSpaceSizeArray& no_samples = {50UL, 50UL},
    const typename Surface_T::PSpaceSizeArray& degree     = {3UL, 3UL})
  {
    // Typedefs
    using Unit = typename Surface_T::Unit;

    // Extract sample points
    const auto psurface_samples = surface::sample(
      psurface, psurface->startParameters(), psurface->endParameters(),
      no_samples, typename Surface_T::PSpaceSizeArray{0UL, 0UL});

    DMatrixT<VectorT<Unit, 3>> P(no_samples[0], no_samples[1]);
    for (auto i = 0UL; i < psurface_samples.rows(); ++i)
      for (auto j = 0UL; j < psurface_samples.columns(); ++j)
        P(i, j) = blaze::subvector<0UL, 3UL>(psurface_samples(i, j)(0, 0));

    return fitBezierSurface(P, degree);
  }







  template <typename CoefType_T, typename Unit_T = double>
  DMatrixT<CoefType_T>
  fitBSplineSurface(const DMatrixT<CoefType_T>&    P,
                    const std::array<size_t, 2ul>& no_coefs,
                    const std::array<size_t, 2ul>& degree = {3UL, 3UL})
  {
    // Generate design matrices
    const auto A = detail::generateBSplineLSqDesignMatrix<Unit_T>(
      P.rows(), no_coefs[0], degree[0]);
    const auto B = detail::generateBSplineLSqDesignMatrix<Unit_T>(
      P.columns(), no_coefs[1], degree[1]);

    // Factorize using QR decomposition
    DMatrixT<Unit_T, blaze::columnMajor> Ra, Rb;
    DMatrixT<Unit_T, blaze::rowMajor>    Qa, Qb;
    blaze::qr(A, Qa, Ra);
    assert(A == Qa * Ra);
    blaze::qr(B, Qb, Rb);
    assert(B == Qb * Rb);

    // QR
    // A x B^T = b --> minimize: || A x B^T - b ||^2
    // =>
    // Ra x Rb^T = Qa^T P Qb

    // solve for new coefficients
    const auto QaPQb          = blaze::trans(Qa) * P * Qb;
    const auto invRa          = blaze::inv(Ra);
    const auto RbT            = blaze::trans(Rb);
    const auto invRbT         = blaze::inv(RbT);
    const auto lsq_approx_cps = invRa * QaPQb * invRbT;

    return lsq_approx_cps;
  }


  template <typename Surface_T>
  DMatrixT<typename Surface_T::Point> fitBSplineSurface(
    const Surface_T*                           psurface,
    const typename Surface_T::PSpaceSizeArray& no_coefs   = {20UL, 20UL},
    const typename Surface_T::PSpaceSizeArray& no_samples = {50UL, 50UL},
    const typename Surface_T::PSpaceSizeArray& degree     = {3UL, 3UL})
  {
    // Typedefs
    using Unit = typename Surface_T::Unit;


    // Extract sample points
    const auto psurface_samples = surface::sample(
      psurface, psurface->startParameters(), psurface->endParameters(),
      no_samples, typename Surface_T::PSpaceSizeArray{0UL, 0UL});

    DMatrixT<typename Surface_T::Point> P(no_samples[0], no_samples[1]);

    for (auto i = 0UL; i < psurface_samples.rows(); ++i)
      for (auto j = 0UL; j < psurface_samples.columns(); ++j)
        P(i, j) = blaze::subvector<0UL, 3UL>(psurface_samples(i, j)(0, 0));


    return fitBSplineSurface(P, no_coefs, degree);
  }

}   // namespace gm::parametric::fitting

#endif   // GM2_PARAMETRIC_UTILS_FITTING_H
