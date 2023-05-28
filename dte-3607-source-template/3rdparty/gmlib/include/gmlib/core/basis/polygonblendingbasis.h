#ifndef GM2_BASIS_POLYGONBLENDINGBASIS_H
#define GM2_BASIS_POLYGONBLENDINGBASIS_H

#include "../gm2_blaze.h"

// stl
#include <tuple>

namespace gm::basis::polygonblendingbasis
{


  namespace types
  {
    template <typename Unit_T>
    using Lambda = gm::DVectorT<Unit_T>;

    template <typename Unit_T>
    using Lambdas = gm::DVectorT<Lambda<Unit_T>>;

  }   // namespace types


  template <typename... dxN_B_Ts>
  struct PolygonBlendingBasis {

    template <typename Unit_T, size_t xN_T = 0>
    constexpr auto operator()(const types::Lambda<Unit_T>& l) const
    {
      static_assert(std::tuple_size<Tuple_dxN_Bs>::value > xN_T,
                    "PolygonBlendingBasis derivative index out of bounce.");
      return std::get<xN_T>(dxN_Bs)(l);
    }

  private:
    using Tuple_dxN_Bs = std::tuple<dxN_B_Ts...>;
    static constexpr Tuple_dxN_Bs dxN_Bs{};
  };


  namespace detail
  {
    template <typename Unit_T>
    auto generateSideBasedHParameterizations(const types::Lambdas<Unit_T>& L)
    {
      auto H = types::Lambda<Unit_T>(L.size());
      for (size_t i = 0; i < L.size(); ++i) H[i] = gbc::gbcToHMapping(L[i]);
      return H;
    }

    template <typename Unit_T, typename BFunc_T, size_t M_T>
    auto computeDh(const types::Lambda<Unit_T>&   H,
                   const std::array<size_t, M_T>& is, const BFunc_T BFunc)
    {
      const auto N  = H.size();
      Unit_T     dh = 1;
      for (size_t i = 0; i < N; ++i)
        if (std::find(std::begin(is), std::end(is), i) == std::end(is))
          dh *= BFunc(H[i]);
      return dh;
    }


    template <typename Unit_T, typename BFunc_T>
    auto evaluateVertexWiseBasis(const types::Lambda<Unit_T>& H,
                                 const BFunc_T&               BFunc)
    {
      const auto            N = H.size();
      types::Lambda<Unit_T> dH(N);
      for (size_t i = 0; i < N; ++i) dH[i] = computeDh(H, std::array{i}, BFunc);
      return dH;
    }

    template <typename Unit_T, typename BFunc_T>
    auto evaluateEdgeWiseBasis(const types::Lambda<Unit_T>& H,
                               const BFunc_T&               BFunc)
    {
      const auto            N = H.size();
      types::Lambda<Unit_T> dH(N);
      for (size_t i = 0; i < N; ++i)
        dH[i] = computeDh(H, std::array{(i + N - 1) % N, i}, BFunc);
      return dH;
    }
  }   // namespace detail



  namespace corner
  {

    template <typename BFunc_T>
    struct CornerBlendingBasisKernel {


      template <typename Unit_T>
      constexpr auto operator()(const types::Lambda<Unit_T>& l) const
      {
        types::Lambda<Unit_T> B(l.size());
        std::transform(std::begin(l), std::end(l), std::begin(B),
                       [](const auto& u) { return m_BFunc(u); });

        Unit_T Bsum = 0;
        for (const auto& b : B) Bsum += b;
        for (auto& b : B) b /= Bsum;

        return B;
      }

    private:
      static constexpr BFunc_T m_BFunc{};
    };

  }   // namespace corner



  namespace side
  {

    template <typename BFunc_T>
    struct SideBlendingBasisKernel {

      template <typename Unit_T>
      constexpr auto operator()(const types::Lambda<Unit_T>& l) const
      {
        const auto L  = gbc::generateLeftRotatedLambdaSet(l);
        const auto H  = detail::generateSideBasedHParameterizations(L);
        auto       dH = detail::evaluateEdgeWiseBasis(H, m_BFunc);

        types::Lambda<Unit_T> B(l.size());
        std::transform(std::begin(dH), std::end(dH), std::begin(B),
                       [](const auto& u) { return m_BFunc(u); });

        Unit_T Bsum = 0;
        for (const auto& b : B) Bsum += b;
        for (auto& b : B) b /= Bsum;

        return B;
      }

    private:
      static constexpr BFunc_T m_BFunc{};
    };

  }   // namespace side




  namespace hungariancorner
  {

    template <typename BFunc_T>
    struct HungarianCornerBlendingBasisKernel {

      template <typename Unit_T>
      constexpr auto operator()(const types::Lambda<Unit_T>& l) const
      {
        const auto N = l.size();

        const auto L  = gbc::generateLeftRotatedLambdaSet(l);
        const auto H  = detail::generateSideBasedHParameterizations(L);
        auto       dH = detail::evaluateEdgeWiseBasis(H, m_BFunc);
        const auto dH_sum
          = std::accumulate(std::begin(dH), std::end(dH), Unit_T(0));

        types::Lambda<Unit_T> B(N);
        for (size_t i = 0; i < N; ++i) B[i] = dH[i] / dH_sum;
        return B;
      }

    private:
      static constexpr BFunc_T m_BFunc{};
    };

  }   // namespace hungariancorner


  namespace hungarianside
  {
    template <typename BFunc_T>
    struct HungarianSideBlendingBasisKernel {

      template <typename Unit_T>
      constexpr auto operator()(const types::Lambda<Unit_T>& l) const
      {
        const auto N = l.size();

        const auto L  = gbc::generateLeftRotatedLambdaSet(l);
        const auto H  = detail::generateSideBasedHParameterizations(L);
        const auto dH = detail::evaluateEdgeWiseBasis(H, m_BFunc);

        const auto dH_sum
          = std::accumulate(std::begin(dH), std::end(dH), Unit_T(0));

        types::Lambda<Unit_T> B(N);
        for (size_t i = 0; i < N; ++i)
          B[i] = (dH[i] + dH[(i + 1) % N]) / dH_sum;

        return B;
      }

    private:
      static constexpr BFunc_T m_BFunc{};
    };

  }   // namespace hungarianside

  namespace hungarianspecialside
  {

    /**!
     * cite:
     *   "Ribbon-based Transfinite Surfaces", Salvi and Varady and Rockwood,
     *   2014
     */
    template <typename BFunc_T>
    struct HungarianSpecialSideBlendingBasisKernel {


      template <typename Unit_T>
      constexpr auto operator()(const types::Lambda<Unit_T>& l) const
      {
        const auto N = l.size();

        const auto L  = gbc::generateLeftRotatedLambdaSet(l);
        const auto H  = detail::generateSideBasedHParameterizations(L);
        const auto dH = detail::evaluateVertexWiseBasis(H, m_BFunc);

        const auto dH_sum
          = std::accumulate(std::begin(dH), std::end(dH), Unit_T(0));

        const auto epsilon = 1e-4;

        types::Lambda<Unit_T> B(N);
        for (size_t i = 0; i < N; ++i) {
          const auto im = (i + N - 1) % N;
          if (H[im] <= epsilon and H[i] <= epsilon) {
            for (auto& B_i : B) B_i = 0;
            B[i] = 1.0;
            break;
          }
          else {
            B[i] = dH[i] / dH_sum;
          }
        }

        return B;
      }

    private:
      static constexpr BFunc_T m_BFunc{};
    };

  }   // namespace hungarianspecialside






  template <typename BFunction_T>
  using CornerBlendingBasis
    = PolygonBlendingBasis<corner::CornerBlendingBasisKernel<BFunction_T>>;

  template <typename BFunc_T>
  using SideBlendingBasis
    = PolygonBlendingBasis<side::SideBlendingBasisKernel<BFunc_T>>;

  template <typename BFunc_T>
  using HungarianCornerBlendingBasis = PolygonBlendingBasis<
    hungariancorner::HungarianCornerBlendingBasisKernel<BFunc_T>>;

  template <typename BFunc_T>
  using HungarianSideBlendingBasis = PolygonBlendingBasis<
    hungarianside::HungarianSideBlendingBasisKernel<BFunc_T>>;

  template <typename BFunc_T>
  using HungarianSpecialSideBlendingBasis = PolygonBlendingBasis<
    hungarianspecialside::HungarianSpecialSideBlendingBasisKernel<BFunc_T>>;



}   // namespace gm::basis::polygonblendingbasis

#endif   // GM2_BASIS_POLYGONBLENDINGBASIS_H
