#ifndef GM2_BASIS_TRIANGLEBLENDINGBASIS_H
#define GM2_BASIS_TRIANGLEBLENDINGBASIS_H


namespace gm::basis::triangleblendingbasis
{

  template <typename... dxN_B_Ts>
  struct TriangleBlendingBasis {

    template <typename Unit_T, size_t xN_T = 0>
    static constexpr auto evaluate(const Unit_T& u, const Unit_T& v,
                                   const Unit_T& w)
    {
      static_assert(std::tuple_size<Tuple_dxN_Bs>::value > xN_T,
                    "BasisTriangle derivative index out of bounce.");
      return std::get<xN_T>(dxN_Bs)(u, v, w);
    }

    template <typename Unit_T, size_t xN_T = 0>
    constexpr Unit_T operator()(const Unit_T& u, const Unit_T& v,
                                const Unit_T& w) const
    {
      return evaluate<Unit_T, xN_T>(u, v, w);
    }

  private:
    using Tuple_dxN_Bs = std::tuple<dxN_B_Ts...>;
    static constexpr Tuple_dxN_Bs dxN_Bs{};
  };



  namespace rational
  {
    template <typename BFunction_T>
    struct TriangleBlendingBasisKernel {

      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& u, const Unit_T& v,
                                const Unit_T& w) const
      {
        const auto Bu = m_B(u);
        const auto Bv = m_B(v);
        const auto Bw = m_B(w);

        const auto sum = Bu + Bv + Bw;

        return VectorT<Unit_T, 3ul>{Bu / sum, Bv / sum, Bw / sum};
      }

    private:
      BFunction_T m_B{};
    };


    template <typename BFunction_T>
    struct TriangleBlendingBasisD1Kernel {

      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& u, const Unit_T& v,
                                const Unit_T& w) const
      {
        const auto Bu = m_B(u);
        const auto Bv = m_B(v);
        const auto Bw = m_B(w);

        const auto sum  = Bu + Bv + Bw;
        const auto sum2 = std::pow(sum, 2);

        const auto DBu = m_B.template operator()<Unit_T, 1>(u);
        const auto DBv = m_B.template operator()<Unit_T, 1>(v);
        const auto DBw = m_B.template operator()<Unit_T, 1>(w);

        const auto aa_u = DBu / sum2;
        const auto aa_v = DBv / sum2;
        const auto aa_w = DBw / sum2;

        // clang-format off
        return MatrixT<Unit_T, 3ul, 3ul>{
          {
             aa_u * (sum - Bu), // du Bu
            -aa_u * Bv,         //    Bv
            -aa_u * Bw          //    Bw
          },
          {
            -aa_v * Bu,         // dv Bu
             aa_v * (sum - Bv), //    Bv
            -aa_v * Bw          //    Bw
          },
          {
            -aa_w * Bu,         // dw Bu
            -aa_w * Bv,         //    Bv
            aa_w * (sum - Bw)   //    Bw
          }};
        // clang-format on
      }

    private:
      BFunction_T m_B{};
    };

    template <typename BFunction_T>
    struct TriangleBlendingBasisD2Kernel {

      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& u, const Unit_T& v,
                                const Unit_T& w) const
      {
        const auto Bu = m_B(u);
        const auto Bv = m_B(v);
        const auto Bw = m_B(w);

        const auto sum  = Bu + Bv + Bw;
        const auto sum2 = sum * sum;
        const auto sum3 = sum2 * sum;

        const auto DBu = m_B.template operator()<Unit_T, 1>(u);
        const auto DBv = m_B.template operator()<Unit_T, 1>(v);
        const auto DBw = m_B.template operator()<Unit_T, 1>(w);

        const auto D2Bu = m_B.template operator()<Unit_T, 2>(u);
        const auto D2Bv = m_B.template operator()<Unit_T, 2>(v);
        const auto D2Bw = m_B.template operator()<Unit_T, 2>(w);

        const auto aa_uu = ((2 * DBu * DBu / sum) - D2Bu) / sum2;
        const auto aa_uv = DBu * DBv / sum3;
        const auto aa_uw = DBu * DBw / sum3;
        const auto aa_vv = ((2 * DBv * DBv / sum) - D2Bv) / sum2;
        const auto aa_vw = DBv * DBw / sum3;
        const auto aa_ww = ((2 * DBw * DBw / sum) - D2Bw) / sum2;

        // clang-format off
        return MatrixT<Unit_T, 6ul, 3ul>{
          {
            aa_uu * (Bu-sum),           // duu Bu
            aa_uu * Bv,                 //     Bv
            aa_uu * Bw                  //     Bw
          },
          {
            aa_uv * ((2 * Bu) - sum),   // duv Bu
            aa_uv * ((2 * Bu) - sum),   //     Bv
            aa_uv *   2 * Bw            //     Bw
          },
          {
            aa_uw * ((2 * Bu) - sum),   // duw Bu
            aa_uw *   2 * Bv,           //     Bv
            aa_uw * ((2 * Bw) - sum)    //     Bw
          },
          {
            aa_vv *  Bu,                // dvv Bu
            aa_vv * (Bv - sum),         //     Bv
            aa_vv *  Bw                 //     Bw
          },
          {
            aa_vw *  2 * Bu,            // dvw Bu
            aa_vw * (2 * Bv - sum),     //     Bv
            aa_vw * (2 * Bw - sum)      //     Bw
          },
          {
            aa_ww *  Bu,                // dww Bu
            aa_ww *  Bv,                //     Bv
            aa_ww * (Bw - sum)          //     Bw
          }
        };
        // clang-format on
      }

    private:
      BFunction_T m_B{};
    };


  }   // namespace rational




  template <typename BFunction_T>
  using RationalTriangleBlendingBasis = TriangleBlendingBasis<
    rational::TriangleBlendingBasisKernel<BFunction_T>,
    rational::TriangleBlendingBasisD1Kernel<BFunction_T>>;



}   // namespace gm::basis::triangleblendingbasis



#endif   // GM2_BASIS_TRIANGLEBLENDINGBASIS_H
