#ifndef GM2_CORE_GBC_MVC_H
#define GM2_CORE_GBC_MVC_H

#include "../gm2_blaze.h"

namespace gm::gbc {

  /**!
   * Mean value coordiantes for Arbitrary Planar Polygons, Hormann and Floater
   * -- Only for simple polygons (does not contain other polygons) --
   *
   * \tparam UnitType_T UnitType - real type: (float,double)
   * \tparam Point_T Polygon Point type - euclidean type
   * \tparam VD_T Dimension of the Vector-space of the planar polygon
   *
   * \param[in] V An arbitrary M-sided Planar Polygon of
   * \param[in] x A vertex point inside the polygon
   * \param[in] F Per vertex; v in V, associated data points
   *
   * \returns the mean value weights lambda_i, i=0,...,n-1
   */
  template <typename Unit_Type_T, size_t VectorDim_T, typename FElem_T>
  auto mvc(const DVectorT<VectorT<Unit_Type_T, VectorDim_T>>& V,
                  const VectorT<Unit_Type_T, VectorDim_T>&           x,
                  const DVectorT<FElem_T>&                           F)
  {

    // Dimensions must be at least 3
    assert(V.size() >= 3);
    const auto M = V.size();

    // Dimensions must match
    assert(M == F.size());



    using UnitType = Unit_Type_T;
    using Vector   = VectorT<UnitType, VectorDim_T>;

    const auto N = int(M);   // PVectorDim
    //      using PSpacePoint = VectorT<UnitType, VectorDimF_T>;
    using ReturnType = const FElem_T;


    auto s = std::vector<Vector>(size_t(N));
    std::transform(std::begin(V), std::end(V), std::begin(s),
                   [&x](const auto& v_i) { return v_i - x; });

    auto r = std::vector<UnitType>(size_t(N));
    auto A = std::vector<UnitType>(size_t(N));
    auto D = std::vector<UnitType>(size_t(N));
    for (int i = 0; i < N; ++i) {

      const auto ip1 = (i + N + 1) % N;
      r[size_t(i)]   = blaze::length(s[size_t(i)]);

      MatrixT<UnitType, 2, 2> A_mat;
      blaze::column(A_mat, 0UL) = s[size_t(i)];
      blaze::column(A_mat, 1UL) = s[size_t(ip1)];

      A[size_t(i)] = blaze::det(A_mat) / UnitType(2);
      D[size_t(i)] = blaze::inner(s[size_t(i)], s[size_t(ip1)]);

      // x == v_i
      if (std::abs(r[size_t(i)]) < 1e-7) return ReturnType(F[size_t(i)]);

      // x \in e_i (v_i,v_{i+1})
      if (std::abs(A[size_t(i)]) < 1e-7 and D[size_t(i)] < 0) {

        r[size_t(ip1)] = blaze::length(s[size_t(ip1)]);

        return ReturnType(
          ((r[size_t(ip1)] * F[size_t(i)]) + (r[size_t(i)] * F[size_t(ip1)]))
          / (r[size_t(i)] + r[size_t(ip1)]));
      }
    }

    FElem_T  f(M, 0);
    UnitType W = 0;
    for (int i = 0; i < N; ++i) {

      const auto ip1 = (i + N + 1) % N;
      const auto im1 = (i + N - 1) % N;

      UnitType w = 0;

      if (std::abs(A[size_t(im1)]) >= 1e-7)
        w += (r[size_t(im1)] - (D[size_t(im1)] / r[size_t(i)]))
             / A[size_t(im1)];

      if (std::abs(A[size_t(i)]) >= 1e-7)
        w += (r[size_t(ip1)] - (D[size_t(i)] / r[size_t(i)])) / A[size_t(i)];

      f += w * F[size_t(i)];
      W += w;
    }

    const auto f_over_W = f / W;

    return ReturnType(f_over_W);
  }


  template <typename Unit_Type_T, size_t VectorDim_T>
  auto mvc(const DVectorT<VectorT<Unit_Type_T, VectorDim_T>>& V,
                  const VectorT<Unit_Type_T, VectorDim_T>&           x)
  {

    // Dimensions must be at least 3
    assert(V.size() >= 3);
    const auto M = V.size();


    using PSpacePoint = DVectorT<Unit_Type_T>;

    auto F = DVectorT<PSpacePoint>(M, PSpacePoint(M, Unit_Type_T{0}));
    for (size_t i = 0; i < M; ++i) F[i][i] = Unit_Type_T{1};

    return mvc(V, x, F);
  }

}   // namespace gm::gbc

#endif // GM2_CORE_GBC_MVC_H
