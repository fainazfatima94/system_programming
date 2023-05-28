#ifndef GM2_CORE_BASISGENERATORS_HERMITEBASISGENERATORS_H
#define GM2_CORE_BASISGENERATORS_HERMITEBASISGENERATORS_H

// blaze
#include <blaze/Math.h>

// stl
#include <chrono>
#include <cmath>
#include <iostream>



//////////////////
namespace gm::basis
{






  /*!
   * Generates a vector of Hermite-type basis-function values at t
   * for a given DERIVATIVE:
   * \tparam Nth_DER Derivative to generate for; 0 -> function value
   * \tparam TF Blaze spesific transpose flag; return row or column vector.
   * \param [in] t Hermite function paramter value t = [0,1]
   */
  template <typename Unit_Type, size_t Nth_DER, bool TF = blaze::rowVector>
  constexpr auto generateHermiteBasisVectorF2D2(const Unit_Type& t)
  {

    const auto t2 = t * t;
    const auto t3 = t * t2;

    using ReturnType = VectorT<Unit_Type, 4, TF>;
    if constexpr (Nth_DER == 0) {
      // clang-format off
      return ReturnType{
        1 -     3*t2 + 2*t3,
                3*t2 - 2*t3,
          + t - 2*t2 +   t3,
              -   t2 +   t3
      };
      // clang-format on
    }
    if constexpr (Nth_DER == 1) {
      // clang-format off
      return ReturnType{
          - 6*t + 6*t2,
            6*t - 6*t2,
        1 - 4*t + 3*t2,
          - 2*t + 3*t2
      };
      // clang-format on
    }
    if constexpr (Nth_DER == 2) {
      // clang-format off
      return ReturnType{
         - 6 + 12*t,
           6 - 12*t,
           4 +  6*t,
         - 2 +  6*t
      };
      // clang-format off
    }
    if constexpr( Nth_DER == 3) {
      // clang-format off
      return ReturnType{
          12,
        - 12,
           6,
           6
      };
      // clang-format on
    }
    else
      return ReturnType{0, 0, 0, 0};
  }

  template <typename Unit_Type, size_t... i>
  constexpr auto generateHermiteBasisMatrixF2D2(std::index_sequence<i...>,
                                                const Unit_Type& t)
  {
    using ReturnType = MatrixT<Unit_Type, sizeof...(i), 4>;
    ReturnType M;
    [](...) {
    }(((blaze::row(M, i)
        = generateHermiteBasisVectorF2D2<Unit_Type, i, blaze::rowVector>(t)),
       0)...);
    return M;
  }

  /*!
   * Generates a matrix of Hermite-type basis-function values at t
   * for a given numer of DERIVATIVES:
   * \tparam NO_DERIVATIVES Derivative to generate for; 0 -> only function value
   * \param [in] t Hermite function paramter value t = [0,1]
   */
  template <typename Unit_Type, size_t NO_DERIVATIVES>
  constexpr auto generateHermiteBasisMatrixF2D2(const Unit_Type& t)
  {
    return generateHermiteBasisMatrixF2D2<Unit_Type>(
      std::make_index_sequence<NO_DERIVATIVES + 1>{}, t);
  }




  template <typename = void>
  struct HermiteBasisVectorF2D2Kernel {
    template <typename Unit_T>
    constexpr auto operator()(const Unit_T& t) const
    {
      return basis::generateHermiteBasisMatrixF2D2<Unit_T, 2>(t);
    }

    constexpr static auto Degree = 4UL;
  };










  /*!
   * Generates a vector of Hermite-type basis-function values at t
   * for a given DERIVATIVE:
   * \tparam Nth_DER Derivative to generate for; 0 -> function value
   * \tparam TF Blaze spesific transpose flag; return row or column vector.
   * \param [in] t Hermite function paramter value t = [0,1]
   */
  template <typename Unit_Type, size_t Nth_DER, bool TF = blaze::rowVector>
  constexpr auto generateHermiteBasisVectorF3D2(const Unit_Type& t)
  {

    const auto t2 = t * t;
    const auto t3 = t * t2;
    const auto t4 = t * t3;

    using ReturnType = VectorT<Unit_Type, 5, TF>;
    if constexpr (Nth_DER == 0) {
      // clang-format off
      return ReturnType{
        1     - 11*t2 + 18*t3 -  8*t4,
                16*t2 - 32*t3 + 16*t4,
              -  5*t2 + 14*t3 -  8*t4,
            t -  4*t2 +  5*t3 -  2*t4,
                   t2 -  3*t3 +  2*t4
      };
      // clang-format on
    }
    if constexpr (Nth_DER == 1) {
      // clang-format off
      return ReturnType{
          - 22*t + 54*t2 - 32*t3,
            32*t - 96*t2 + 64*t3,
          - 10*t + 42*t2 - 32*t3,
        1 -  8*t + 15*t2 -  8*t3,
             2*t -  9*t2 +  8*t3
      };
      // clang-format on
    }
    if constexpr (Nth_DER == 2) {
      // clang-format off
      return ReturnType{
        - 22 + 108*t -  96*t2,
          32 - 192*t + 192*t2,
        - 10 +  84*t -  96*t2,
        -  8 +  30*t -  24*t2,
           2 -  18*t +  24*t2
      };
      // clang-format off
    }
    if constexpr( Nth_DER == 3) {
      // clang-format off
      return ReturnType{
          108 - 192*t,
        - 192 + 384*t,
           84 - 192*t,
           30 -  48*t,
        -  18 +  48*t
      };
      // clang-format on
    }
    if constexpr( Nth_DER == 4) {
      // clang-format off
      return ReturnType{
        -192,
         384,
        -192,
        - 48,
          48
      };
      // clang-format on
    }
    else
      return ReturnType{0, 0, 0, 0};
  }

  template <typename Unit_Type, size_t... i>
  constexpr auto generateHermiteBasisMatrixF3D2(std::index_sequence<i...>,
                                                const Unit_Type& t)
  {
    using ReturnType = MatrixT<Unit_Type, sizeof...(i), 5>;
    ReturnType M;
    [](...) {
    }(((blaze::row(M, i)
        = generateHermiteBasisVectorF3D2<Unit_Type, i, blaze::rowVector>(t)),
       0)...);
    return M;
  }

  /*!
   * Generates a matrix of Hermite-type basis-function values at t
   * for a given numer of DERIVATIVES:
   * \tparam NO_DERIVATIVES Derivative to generate for; 0 -> only function value
   * \param [in] t Hermite function paramter value t = [0,1]
   */
  template <typename Unit_Type, size_t NO_DERIVATIVES>
  constexpr auto generateHermiteBasisMatrixF3D2(const Unit_Type& t)
  {
    return generateHermiteBasisMatrixF3D2<Unit_Type>(
      std::make_index_sequence<NO_DERIVATIVES + 1>{}, t);
  }




  template <typename = void>
  struct HermiteBasisVectorF3D2Kernel {
    template <typename Unit_T>
    constexpr auto operator()(const Unit_T& t) const
    {
      return basis::generateHermiteBasisMatrixF3D2<Unit_T, 2>(t);
    }

    constexpr static auto Degree = 5UL;
  };
















  /*!
   * Generates a vector of Hermite-type basis-function values at t
   * for a given DERIVATIVE:
   * \tparam Nth_DER Derivative to generate for; 0 -> function value
   * \tparam TF Blaze spesific transpose flag; return row or column vector.
   * \param [in] t Hermite function paramter value t = [0,1]
   */
  template <typename Unit_Type, size_t Nth_DER, bool TF = blaze::rowVector>
  constexpr auto generateHermiteBasisVectorF3D3(const Unit_Type& t)
  {

    const auto t2 = t * t;
    const auto t3 = t * t2;
    const auto t4 = t * t3;
    const auto t5 = t * t4;

    using ReturnType = VectorT<Unit_Type, 6, TF>;
    if constexpr (Nth_DER == 0) {
      // clang-format off
      return ReturnType{
        1     - 23*t2 + 66*t3 - 68*t4 + 24*t5,
                16*t2 - 32*t3 + 16*t4,
                 7*t2 - 34*t3 + 52*t4 - 24*t5,
            t -  6*t2 + 13*t3 - 12*t4 +  4*t5,
              -  8*t2 + 32*t3 - 40*t4 + 16*t5,
              -    t2 +  5*t3 -  8*t4 +  4*t5
      };
      // clang-format on
    }
    if constexpr (Nth_DER == 1) {
      // clang-format off
      return ReturnType{
          - 46*t + 198*t2 - 272*t3 + 120*t4,
            32*t -  96*t2 +  64*t3,
            14*t - 102*t2 + 208*t3 - 120*t4,
        1 - 12*t +  39*t2 -  48*t3 +  20*t4,
          - 16*t +  96*t2 - 160*t3 +  80*t4,
          -  2*t +  15*t2 -  32*t3 +  20*t4
      };
      // clang-format on
    }
    if constexpr (Nth_DER == 2) {
      // clang-format off
      return ReturnType{
        - 46 + 396*t - 816*t2 + 480*t3,
          32 - 192*t + 192*t2,
          14 - 204*t + 624*t2 - 480*t3,
        - 12 +  78*t - 144*t2 +  80*t3,
        - 16 + 192*t - 480*t2 + 320*t3,
        -  2 +  30*t -  96*t2 +  80*t3
      };
      // clang-format off
    }
    if constexpr( Nth_DER == 3) {
      // clang-format off
      return ReturnType{
          396 - 1632*t + 1440*t2,
        - 192 +  384*t,
        - 204 + 1248*t - 1440*t2,
           78 -  288*t +  240*t2,
          192 -  960*t +  960*t2,
           30 -  192*t +  240*t2
      };
      // clang-format on
    }
    if constexpr( Nth_DER == 4) {
      // clang-format off
      return ReturnType{
        -1632 + 2880*t,
          384,
         1248 - 2880*t,
        - 288 +  480*t,
        - 960 + 1920*t,
        - 192 +  480*t
      };
      // clang-format on
    }
    if constexpr( Nth_DER == 5) {
      // clang-format off
      return ReturnType{
         2880,
            0,
        -2880,
          480,
         1920,
          480
      };
      // clang-format on
    }
    else
      return ReturnType{0, 0, 0, 0};
  }

  template <typename Unit_Type, size_t... i>
  constexpr auto generateHermiteBasisMatrixF3D3(std::index_sequence<i...>,
                                                const Unit_Type& t)
  {
    using ReturnType = MatrixT<Unit_Type, sizeof...(i), 6>;
    ReturnType M;
    [](...) {
    }(((blaze::row(M, i)
        = generateHermiteBasisVectorF3D3<Unit_Type, i, blaze::rowVector>(t)),
       0)...);
    return M;
  }

  /*!
   * Generates a matrix of Hermite-type basis-function values at t
   * for a given numer of DERIVATIVES:
   * \tparam NO_DERIVATIVES Derivative to generate for; 0 -> only function value
   * \param [in] t Hermite function paramter value t = [0,1]
   */
  template <typename Unit_Type, size_t NO_DERIVATIVES>
  constexpr auto generateHermiteBasisMatrixF3D3(const Unit_Type& t)
  {
    return generateHermiteBasisMatrixF3D3<Unit_Type>(
      std::make_index_sequence<NO_DERIVATIVES + 1>{}, t);
  }




  template <typename = void>
  struct HermiteBasisVectorF3D3Kernel {
    template <typename Unit_T>
    constexpr auto operator()(const Unit_T& t) const
    {
      return basis::generateHermiteBasisMatrixF3D3<Unit_T, 2>(t);
    }

    constexpr static auto Degree = 6UL;
  };


}   // END namespace gm



#endif   // GM2_CORE_BASISGENERATORS_HERMITEBASISGENERATORS_H
