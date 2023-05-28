#ifndef GM2_CORE_BLAZE_H
#define GM2_CORE_BLAZE_H


#include "coreutils.h"

// blaze
#include <blaze/Math.h>

// stl
#include <chrono>
#include <cmath>
#include <iostream>







//////////////////
// Data structures

namespace gm
{

  // Point/Vector types
  template <typename Type_T, size_t N_T, bool TF_T = blaze::columnVector>
  using VectorT = blaze::StaticVector<Type_T, N_T, TF_T>;

  template <typename Type_T, size_t N_T>
  using ColVectorT = VectorT<Type_T, N_T>;

  template <typename Type_T, size_t N_T>
  using RowVectorT = VectorT<Type_T, N_T, blaze::rowVector>;




  // Matrix
  template <typename Type_T, size_t M_T, size_t N_T, bool SO_T = blaze::rowMajor>
  using MatrixT = blaze::StaticMatrix<Type_T, M_T, N_T, SO_T>;

  template <typename Type_T, size_t N_T, bool SO_T = blaze::rowMajor>
  using SqMatrixT = MatrixT<Type_T, N_T, N_T, SO_T>;



  // Extended arithmetic containers
  template <typename Element_T, bool TF_T = blaze::columnVector>
  using DVectorT = blaze::DynamicVector<Element_T, TF_T>;

  template <typename Element_T>
  using DColVectorT = DVectorT<Element_T>;

  template <typename Element_T>
  using DRowVectorT = DVectorT<Element_T, blaze::rowVector>;

  template <typename Element_T, bool SO_T = blaze::rowMajor>
  using DMatrixT = blaze::DynamicMatrix<Element_T, SO_T>;







}   // END namespace gm





////////////////////
// Specialized utils
namespace gm::utils
{
  template <typename Value_T, size_t size_T, bool TF_T>
  constexpr auto
  extendStaticContainer(const blaze::StaticVector<Value_T, size_T, TF_T>& C,
                        const Value_T&                                    val)
  {

    return extendStaticContainer<blaze::StaticVector<Value_T, size_T, TF_T>,
                                 blaze::StaticVector<Value_T, size_T + 1, TF_T>,
                                 size_T, 1, Value_T>(C, val);
  }
}   // namespace gm::utils






/////////////
// Algorithms

namespace gm::algorithms
{

  template <typename Type_T, size_t n>
  VectorT<Type_T, n> linearIndependentVector(const VectorT<Type_T, n>& v)
  {
    if (n == 1 || std::abs(length(v)) < 10e-6)
      return VectorT<Type_T, n>(Type_T(0));
    else {
      size_t i, j = 0;
      for (i = 1; i < n; i++)
        if ((v[i] * v[i]) > (v[j] * v[j])) j = i;
      if (j == 0)
        i = 1;
      else
        i = j - 1;

      VectorT<Type_T, n> r(v);

      Type_T tmp = -r[j];
      r[j]       = r[i];
      r[i]       = tmp;
      r          = r - (dot(r, v) / dot(v, v)) * v;
      return r;
    }
  }

  template <typename Type_T>
  DVectorT<Type_T> linearIndependentVector(const DVectorT<Type_T>& v)
  {
    const auto n = v.size();

    if (n == 1 || std::abs(length(v)) < 10e-6)
      return DVectorT<Type_T>(n, Type_T(0));
    else {
      size_t i, j = 0;
      for (i = 1; i < n; i++)
        if ((v[i] * v[i]) > (v[j] * v[j])) j = i;
      if (j == 0)
        i = 1;
      else
        i = j - 1;

      DVectorT<Type_T> r(v);

      Type_T tmp = -r[j];
      r[j]       = r[i];
      r[i]       = tmp;
      r          = r - (dot(r, v) / dot(v, v)) * v;
      return r;
    }
  }


  template <typename Type_T, size_t n = 3>
  DMatrixT<Type_T> linearIndependetFrameTo(const DVectorT<Type_T>& t0)
  {
    static_assert(n == 3, "Neds to be defined in R3");

    auto liv_t0
      = blaze::evaluate(blaze::normalize(linearIndependentVector(t0)));
    auto t2 = blaze::evaluate(blaze::normalize(blaze::cross(liv_t0, t0)));
    auto t1 = blaze::evaluate(blaze::normalize(blaze::cross(t0, t2)));

    DMatrixT<Type_T> U(n, n);
    blaze::column(U, 0UL) = blaze::normalize(t0);
    blaze::column(U, 1UL) = blaze::normalize(t1);
    blaze::column(U, 2UL) = blaze::normalize(t2);
    return U;
  }

  // rotaion minimization frame - Microsoft - double reflection
  // ToG v.27, n.1, March 2008
  // returns; rhs, [dir,side,up], ([ti,si,ri]), 3x3
  template <typename Type_T, size_t n>
  DMatrixT<Type_T> rotationMinimizingFrameMSDR(const DVectorT<Type_T>& ri,
                                               const DVectorT<Type_T>& xi,
                                               const DVectorT<Type_T>& xip1,
                                               const DVectorT<Type_T>& ti,
                                               const DVectorT<Type_T>& tip1)
  {
    static_assert(n == 3, "Neds to be defined in R3");

    //    auto ti = blaze::column(Ui,0UL);
    //    auto ri = blaze::column(Ui,2UL);
    //    std::cout << "t[i]: " << std::endl << ti << std::endl;
    //    std::cout << "r[i]: " << std::endl << ri << std::endl;

    auto v1 = xip1 - xi;
    auto c1 = blaze::inner(v1, v1);
    //    std::cout << "v1: " << std::endl << v1 << std::endl;
    //    std::cout << "c1: " << c1 << std::endl;

    auto rLi = ri - ((2.0 / c1) * blaze::inner(v1, ri) * v1);
    auto tLi = ti - ((2.0 / c1) * blaze::inner(v1, ti) * v1);
    //    std::cout << "rL[i]: " << std::endl << rLi << std::endl;
    //    std::cout << "tL[i]: " << std::endl << tLi << std::endl;

    auto v2 = tip1 - tLi;
    auto c2 = blaze::inner(v2, v2);
    //    std::cout << "v2: " << std::endl << v2 << std::endl;
    //    std::cout << "c2: " << c2 << std::endl;

    auto rip1 = rLi - ((2.0 / c2) * blaze::inner(v2, rLi) * v2);
    auto sip1 = blaze::cross(tip1, rip1);
    //    std::cout << "t[i-1]: " << std::endl << tip1 << std::endl;
    //    std::cout << "s[i-1]: " << std::endl << sip1 << std::endl;
    //    std::cout << "r[i-1]: " << std::endl << rip1 << std::endl;

    DMatrixT<Type_T> Uip1(n, n);
    blaze::column(Uip1, 0UL) = blaze::normalize(tip1);
    blaze::column(Uip1, 1UL) = blaze::normalize(sip1);
    blaze::column(Uip1, 2UL) = blaze::normalize(rip1);
    return Uip1;
  }

  template <typename Type_T>
  auto ortoNormal(const VectorT<Type_T, 3>& b, const VectorT<Type_T, 3>& c)
  {
    VectorT<Type_T, 3> a = normalize(b - blaze::dot(b, c) * c);
    return a;
  }

  template <typename Type_T>
  size_t maxAbsIndex(const VectorT<Type_T, 3>& v)
  {
    size_t j = 0;
    for (size_t i = 1; i < 3; i++)
      if (std::fabs(v[i]) > std::fabs(v[j])) j = i;
    return j;
  }

  /*! SqMatrixT<Type_T, 3> orthogonalMatrix(const VectorT<Type_T, 3>& u, const
   * VectorT<Type_T, 3>& v) \brief To make an orthonormal set of basis-vectors
   * using vector u and vector v as a start.
   *
   *  To make an orthonormal set of basis-vectors using vector u and vector v as
   * a start.
   */
  template <typename Type_T>
  auto orthogonalMatrix(const VectorT<Type_T, 3>& u,
                        const VectorT<Type_T, 3>& v)
  {
    const size_t         n = 3;
    SqMatrixT<Type_T, n> x;

    VectorT<Type_T, 3> uu = normalize(u);
    blaze::column<0UL>(x) = uu;
    blaze::column<1UL>(x) = ortoNormal(v, uu);

    size_t ku = maxAbsIndex(u);
    size_t kv = maxAbsIndex(VectorT<Type_T, 3>{blaze::column<1UL>(x)});

    size_t k = 0, i = 2;
    for (; i < n; i++, k++) {
      if (k == ku) k++;
      if (k == kv) {
        k++;
        if (k == ku) k++;
      }
      blaze::column(x, i)[k] = Type_T{1};
    }

    for (i = 2; i < n; i++) {
      for (size_t j = 0; j < i; j++) {
        Type_T tmp = blaze::dot(blaze::column(x, i), blaze::column(x, j));
        blaze::column(x, i) = blaze::column(x, i) - tmp * blaze::column(x, j);
      }
      blaze::column(x, i) = blaze::normalize(blaze::column(x, i));
    }

    using ReturnType = const SqMatrixT<Type_T, 3>;
    return ReturnType{x};
  }

  template <typename Type_T, size_t n>
  auto orthogonalMatrixOfAxis(const VectorT<Type_T, n>& w)
  {
    VectorT<Type_T, n> lu = linearIndependentVector(w);
    VectorT<Type_T, n> u  = blaze::cross(lu, w);
    VectorT<Type_T, n> v  = blaze::cross(w, u);
    //    std::cout << "w:\n" << w << std::endl;
    //    std::cout << "lu:\n" << lu << std::endl;
    //    std::cout << "u:\n" << u << std::endl;
    //    std::cout << "v:\n" << v << std::endl;
    //    const auto axis_check = blaze::cross(u,v);
    //    std::cout << "w(verify):\n" << axis_check << std::endl;

    return orthogonalMatrix(u, v);
  }

  //  template <typename Type_T, size_t n>
  //  auto basisChange(const SqMatrixT<Type_T, n>& x, const SqMatrixT<Type_T,
  //  n>& y)
  //  {
  //    using ReturnType = const SqMatrixT<Type_T,n>;
  //    return ReturnType{blaze::trans(x) * y};
  //  }

  //  template <typename Type_T, size_t n>
  //  auto basisChangeInverse(const SqMatrixT<Type_T, n>& x, const
  //  SqMatrixT<Type_T, n>& y)
  //  {
  //    using ReturnType = const SqMatrixT<Type_T,n>;
  //    return ReturnType{y *  x};
  //  }

  // Non-dimensional prototype
  template <typename Type_T, size_t n, typename = std::enable_if_t<n >= 2UL>>
  auto xyRotationMatrix(Type_T ang)
  {
    const auto sa = std::sin(ang);
    const auto ca = std::cos(ang);

    using MatType = SqMatrixT<Type_T, n>;

    MatType M(
      0);   // still rewriting 4 + (N-2UL) values... but who cares -- for now!
    M(0UL, 0UL) = ca;
    M(0UL, 1UL) = -sa;
    M(1UL, 0UL) = sa;
    M(1UL, 1UL) = ca;
    blaze::subvector<2UL, n - 2UL>(blaze::diagonal(M))
      = VectorT<Type_T, n - 2UL>(1);

    using ReturnType = const MatType;
    return ReturnType{M};
  }



  template <typename Type_T, size_t n>
  auto rotationMatrix(Type_T ang, const VectorT<Type_T, n>& rot_axis)
  {
    // Represent plane of rotation axis
    const auto x = orthogonalMatrixOfAxis(rot_axis);

    // Construct XY rotation matrix
    const auto r = xyRotationMatrix<Type_T, n>(ang);

    // Rotation matrix is: (r * x)^T * x
    using ReturnType = const SqMatrixT<Type_T, n>;
    return ReturnType{(x * r) * blaze::trans(x)};
  }

}   // namespace gm::algorithms


#endif   // GM2_CORE_BLAZE_H
