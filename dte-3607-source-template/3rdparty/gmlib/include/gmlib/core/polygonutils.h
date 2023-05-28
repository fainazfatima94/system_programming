#ifndef GM2_CORE_POLYGONUTILS_H
#define GM2_CORE_POLYGONUTILS_H

#include "gm2_blaze.h"

namespace gm::polygonutils
{

  template <typename Unit_Type_T       = double,
            typename PSpaceUnit_Type_T = Unit_Type_T>
  auto generateRegularPolygon2D(size_t N);

  template <typename Unit_Type_T       = double,
            typename PSpaceUnit_Type_T = Unit_Type_T>
  auto generateRegularPolygon2DXZ(size_t N);




  template <typename Unit_T>
  auto projectOnto2DPlane(const VectorT<Unit_T, 3>& Op,
                          const VectorT<Unit_T, 3>& u_axis3D,
                          const VectorT<Unit_T, 3>& v_axis3D,
                          const VectorT<Unit_T, 2>& u_axis2D,
                          const VectorT<Unit_T, 2>& v_axis2D);

  // LSQ approx
  template <typename Unit_T>
  auto
  approximateBestFitPlaneLsq(const DVectorT<VectorT<Unit_T, 3UL>>& polygon3D);

  // Construct plane normal by CCW polygon-boundary traversal
  template <typename Unit_T>
  auto
  approximateBestFitPlaneN(const DVectorT<VectorT<Unit_T, 3UL>>& polygon3D);

  // Construct plane normal and primary [u,v]-axis by CCW polygon-boundary
  // traversal
  // [u,v]-axis constructed as follows:
  //   u = ||linInd(n) ^ n||
  //   v = ||n ^ u||
  template <typename Unit_T>
  auto
  approximateBestFitPlaneNUV(const DVectorT<VectorT<Unit_T, 3UL>>& polygon3D);

  /*!
   * Approximates a best fitting polygon2D given a polygon3D
   * in a best fitting plane2D with normal out of the positive up.
   *
   * The polygon3D is expected to be simple and
   * to have a best fitting polygon 2d.
   *
   * \return Tuple of Polygon, Plane-Normal, U-axis and V-axis (best fit)
   */
  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedPlaneNUV(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D);

  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedPlane(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D);

  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedPlaneCenteredInNUV(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D,
    const VectorT<Unit_T, 3>&           p3D);

  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedPlaneCenteredIn(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D,
    const VectorT<Unit_T, 3>&           p3D);


  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedZXPlaneNormalized(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D);



}   // namespace gm::polygonutils





namespace gm::polygonutils
{

  template <typename Unit_Type_T, typename PSpaceUnit_Type_T>
  auto generateRegularPolygon2D(size_t N)
  {
    using Point2D   = VectorT<Unit_Type_T, 2>;
    using Polygon2D = DVectorT<Point2D>;

    // Construct PSpace mapping polygon
    const auto dt = (PSpaceUnit_Type_T(1) / PSpaceUnit_Type_T(N))
                    * PSpaceUnit_Type_T(2 * M_PI);

    auto circleEval2D = [](const auto t) {
      return std::forward<Point2D>(Point2D{std::cos(t), std::sin(t)});
    };

    Polygon2D P(N);
    for (size_t i = 0; i < N; ++i)
      P[i] = circleEval2D(PSpaceUnit_Type_T(i) * dt);

    return P;
  }

  template <typename Unit_Type_T, typename PSpaceUnit_Type_T>
  auto generateRegularPolygon2DXZ(size_t N)
  {
    using Point3D   = VectorT<Unit_Type_T, 3>;
    using Polygon3D = DVectorT<Point3D>;

    const auto p2d
      = generateRegularPolygon2D<Unit_Type_T, PSpaceUnit_Type_T>(N);

    Polygon3D P(N);
    for (size_t i = 0; i < N; ++i)
      P[i] = Point3D{p2d[i][0], Unit_Type_T{0}, p2d[i][1]};


    return P;
  }



  template <typename Unit_T>
  auto projectOnto2DPlane(const VectorT<Unit_T, 3>& Op,
                          const VectorT<Unit_T, 3>& u_axis3D,
                          const VectorT<Unit_T, 3>& v_axis3D,
                          const VectorT<Unit_T, 2>& u_axis2D,
                          const VectorT<Unit_T, 2>& v_axis2D)
  {
    return blaze::evaluate(blaze::inner(Op, u_axis3D) * u_axis2D
                           + blaze::inner(Op, v_axis3D) * v_axis2D);
  }


  template <typename Unit_T>
  auto
  approximateBestFitPlaneLsq(const DVectorT<VectorT<Unit_T, 3UL>>& polygon3D)
  {
    auto A  = DMatrixT<Unit_T>(polygon3D.size(), 3UL);
    auto P  = DVectorT<Unit_T>(polygon3D.size());
    auto pc = VectorT<Unit_T, 3>{0.0, 0.0, 0.0};
    for (auto i = 0UL; i < polygon3D.size(); ++i) pc += polygon3D[i];
    pc /= Unit_T(polygon3D.size());

    for (auto i = 0UL; i < polygon3D.size(); ++i) {

      const auto pd = blaze::evaluate(polygon3D[i] - pc);
      A(i, 0)       = pd[2];   // x
      A(i, 1)       = pd[0];   // z
      A(i, 2)       = 1.0;     // y

      P[i] = -pd[1];
    }

    DMatrixT<Unit_T, blaze::columnMajor> R;
    DMatrixT<Unit_T, blaze::rowMajor>    Q;
    blaze::qr(A, Q, R);
    assert(A == Q * R);

    const auto lsq_approx_plane_n
      = DVectorT<Unit_T>(blaze::inv(R) * (blaze::trans(Q) * P));
    return lsq_approx_plane_n;
  }

  template <typename Unit_T>
  auto approximateBestFitPlaneN(const DVectorT<VectorT<Unit_T, 3UL>>& polygon3D)
  {
    auto n_sum = VectorT<Unit_T, 3UL>(0);
    for (auto i = 0UL; i < polygon3D.size(); ++i) {

      const auto im = (i + polygon3D.size() - 1) % polygon3D.size();
      const auto ip = (i + polygon3D.size() + 1) % polygon3D.size();

      const auto vp = polygon3D[ip] - polygon3D[i];
      const auto vm = polygon3D[im] - polygon3D[i];
      const auto n  = blaze::cross(vp, vm);

      n_sum += n;
    }

    return blaze::evaluate(blaze::normalize(n_sum));
  }


  template <typename Unit_T>
  auto
  approximateBestFitPlaneNUV(const DVectorT<VectorT<Unit_T, 3UL>>& polygon3D)
  {
    // approx best fitint plane normal
    const auto n = approximateBestFitPlaneN(polygon3D);

    // construct plane -- choose X or Z as starting "X-axis"
    const auto linIndAxis = gm::algorithms::linearIndependentVector(n);

    // best fit plane as primary axis
    const auto axisU = blaze::normalize(blaze::cross(linIndAxis, n));
    const auto axisV = blaze::normalize(blaze::cross(n, axisU));

    return std::tuple(n, blaze::evaluate(axisU), blaze::evaluate(axisV));
  }



  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedPlaneNUV(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D)
  {

    using Polygon2D = DVectorT<VectorT<Unit_T, 2>>;


    // approx best fitint plane normal and primary axis
    const auto [n, axisX, axisY] = approximateBestFitPlaneNUV(polygon3D);

    // projected polygon
    Polygon2D polygon2D(polygon3D.size());

    for (auto i = 0UL; i < polygon3D.size(); ++i) {
      const auto p3d = polygon3D[i];
      polygon2D[i]
        = projectOnto2DPlane(p3d, axisX, axisY, VectorT<Unit_T, 2>{1., 0.},
                             VectorT<Unit_T, 2>{0., 1.});
    }

    return std::tuple(polygon2D, n, axisX, axisY);
  }


  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedPlane(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D)
  {
    const auto [P2d, n, U, V]
      = projectPolygonOntoBestFitApproximatedPlaneNUV(polygon3D);
    return P2d;
  }



  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedPlaneCenteredInNUV(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D,
    const VectorT<Unit_T, 3>&           ip3D)
  {

    using Polygon2D = DVectorT<VectorT<Unit_T, 2>>;


    // approx best fitint polygon
    const auto [P2D, n, axisX, axisY]
      = projectPolygonOntoBestFitApproximatedPlaneNUV(polygon3D);

    // projected polygon
    Polygon2D polygon2D(P2D);

    // Project interpolation point onto plane
    const auto ip2D
      = projectOnto2DPlane(ip3D, axisX, axisY, VectorT<Unit_T, 2>{1., 0.},
                           VectorT<Unit_T, 2>{0., 1.});

    // Transform around center
    for (auto& p2d_i : polygon2D) p2d_i -= ip2D;

    return std::tuple(polygon2D, n, axisX, axisY);
  }

  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedPlaneCenteredIn(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D,
    const VectorT<Unit_T, 3>&           ip3D)
  {
    const auto [P2D, n, axisX, axisY]
      = projectPolygonOntoBestFitApproximatedPlaneCenteredInNUV(polygon3D,
                                                                ip3D);

    return P2D;
  }


  template <typename Unit_T>
  auto projectPolygonOntoBestFitApproximatedZXPlaneNormalized(
    const DVectorT<VectorT<Unit_T, 3>>& polygon3D)
  {

    using Point2D   = VectorT<Unit_T, 2>;
    using Polygon2D = DVectorT<Point2D>;

    // Extract ZX-plane
    Polygon2D polygon2D(polygon3D.size());
    for (auto i = 0UL; i < polygon3D.size(); ++i)
      polygon2D[i] = VectorT<Unit_T, 2>{polygon3D[i][2], polygon3D[i][0]};

    // Find barycenter of ZX-plane polygon and move polygon to ZX-origin
    Point2D p2d_c
      = std::accumulate(std::begin(polygon2D), std::end(polygon2D), Point2D(0));
    p2d_c /= Unit_T(polygon2D.size());
    for (auto& p2d_i : polygon2D) p2d_i -= p2d_c;

    // "Normalize" by scaling by longest "arm"
    Unit_T max_len = 1;
    for (const auto& p2d_i : polygon2D)
      max_len = std::max(max_len, blaze::length(p2d_i));
    Unit_T scale = 1 / max_len;
    for (auto& p2d_i : polygon2D) p2d_i *= scale;

    return polygon2D;
  }

}   // namespace gm::polygonutils

#endif   // GM2_CORE_POLYGONUTILS_H
