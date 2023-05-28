#ifndef GM2_PARAMETRIC_POLYGONALSURFACE_GENERALIZEDBEZIERPATCH_H
#define GM2_PARAMETRIC_POLYGONALSURFACE_GENERALIZEDBEZIERPATCH_H

// gm

#include "../polygonalsurface.h"

#include "../../core/divideddifference/finitedifference.h"
#include "../../core/basisgenerators/bernsteinbasisgenerators.h"
#include "../../core/gbc/mvc.h"
#include "../../core/gbc/mapping.h"
#include "../../core/gbc/gbcutils.h"

namespace gm::parametric
{

  namespace generalizedbezierpatch
  {

    template <typename PGBP_T>
    auto constructControlNet(const DVectorT<typename PGBP_T::Point>& P,
                             const int& d, const int& n)
    {


      const auto N = P.size();
      //      using Unit       = typename PGBP_T::Unit;
      using PSpaceUnit = typename PGBP_T::Type;
      using ControlNet = typename PGBP_T::ControlNet;

      //      const auto N   = int(P.size());
      const int nm2 = (n + int(N) - 2) % int(N);
      const int nm1 = (n + int(N) - 1) % int(N);
      const int np1 = (n + int(N) + 1) % int(N);

      DColVectorT<PSpaceUnit> anm2(N, 0);
      anm2[size_t(nm2)] = 1.0;
      DColVectorT<PSpaceUnit> anm1(N, 0);
      anm1[size_t(nm1)] = 1.0;
      DColVectorT<PSpaceUnit> an(N, 0);
      an[size_t(n)] = 1.0;
      DColVectorT<PSpaceUnit> anp1(N, 0);
      anp1[size_t(np1)] = 1.0;

      const auto D  = (d + 2) / 2;
      const auto L  = (d + 1) / 2;
      auto       CN = ControlNet(size_t(L), size_t(D));

      const PSpaceUnit dt = PSpaceUnit(1) / PSpaceUnit(d);


      for (int r = 0; r < L; ++r) {
        for (int c = 0; c < D; ++c) {

          const auto ir = r * dt;   // i-1 side with respect to side i
          const auto ic = c * dt;   // i+1 side with respect to side i

          const auto gamma_im1_alpha = anm1 + ic * (anm2 - anm1);
          const auto gamma_ip1_alpha = an + ic * (anp1 - an);

          const auto alpha_regular = DColVectorT<PSpaceUnit>(
            gamma_ip1_alpha + ir * (gamma_im1_alpha - gamma_ip1_alpha));

          const auto p_rc = blaze::inner(P, alpha_regular);

          CN(size_t(r), size_t(c)) = p_rc;
        }
      }

      using ReturnType = const ControlNet;
      return std::forward<ReturnType>(blaze::eval(CN));
    }

    template <typename PGBP_T>
    auto constructControlNets(const DVectorT<typename PGBP_T::Point>& P,
                              const int& d, const int& is = 0)
    {
      auto const N      = int(P.size());
      using ControlNets = typename PGBP_T::ControlNets;

      auto cns = ControlNets(size_t(N));
      for (int i = is; i < N; ++i)
        cns[size_t(i)] = constructControlNet<PGBP_T>(P, d, (N + i) % N);

      using ReturnType = const ControlNets;
      return std::forward<ReturnType>(cns);
    }


    template <typename PGBP_T>
    auto
    constructRibbonsFromControlNets(const typename PGBP_T::ControlNets& CNs)
    {

      using Ribbons = typename PGBP_T::Ribbons;

      Ribbons    C;
      auto const N = int(CNs.size());
      C.resize(size_t(N));
      for (int j = 0; j < N; ++j) {

        auto i  = j;
        auto im = (j + N - 1) % N;
        //        auto im = (j + N + 1) % N;

        auto& CNi  = CNs.at(size_t(i));
        auto& CNim = CNs.at(size_t(im));

        auto& Ci = C.at(size_t(i));

        // resize C
        Ci.resize(CNim.rows(), CNim.columns() + CNi.rows());

        // fill C[i] from CN[i-1]
        for (auto c_i = 0UL; c_i < CNim.rows(); ++c_i)
          for (auto c_j = 0UL; c_j < CNim.columns(); ++c_j)
            Ci.at(c_i, c_j) = CNim.at(c_i, c_j);

        // fill C[i] from CN[i]
        for (auto c_i = 0UL; c_i < CNim.rows(); ++c_i)
          for (auto c_j = 0UL; c_j < CNi.rows(); ++c_j)
            Ci.at(c_i, (CNim.columns() + c_j))
              = CNi.at(CNi.rows() - 1 - c_j, c_i);
      }

      using ReturnType = const Ribbons;
      return std::forward<ReturnType>(C);
    }

    // Computes the central control point
    template <typename PGBP_T>
    inline auto computeC0(const typename PGBP_T::ControlNets& CNs)
    {
      using Point = typename PGBP_T::Point;
      using Type  = typename PGBP_T::Type;
      //      using ReturnType = const Point;

      const auto N = CNs.size();
      const auto d = CNs.at(0).rows() + CNs.at(0).columns() - 1;
      const auto L = (d + 1) / 2;
      const auto D = (d + 2) / 2;

      Point P0_sum(0);

      for (auto i = 0UL; i < N; ++i) P0_sum += CNs.at(i)(L - 1, D - 1);

      return blaze::evaluate((Type(1) / Type(N)) * P0_sum);
    }

    // Expects a ribbon of degree d as input
    // Outputs a control net of degree d+1
    template <typename PGBP_T>
    auto degreeElevate(const typename PGBP_T::ControlNets& CNs)
    {
      using ControlNets = typename PGBP_T::ControlNets;
      using T           = typename PGBP_T::PSpaceUnit;
      using Point       = typename PGBP_T::Point;

      const auto N = CNs.size();

      const auto d  = CNs.at(0).rows() + CNs.at(0).columns() - 1;
      const auto Ld = (d + 1) / 2;
      const auto Dd = (d + 2) / 2;

      // Computing the central control point
      auto P0 = computeC0<PGBP_T>(CNs);

      ControlNets C(N);

      for (size_t i = 0; i < N; ++i) {

        auto  im    = (i + N - 1) % N;
        auto  im2   = (i + N - 2) % N;
        auto& CNi   = CNs.at(size_t(i));
        auto& CNim  = CNs.at(size_t(im));
        auto& CNim2 = CNs.at(size_t(im2));

        auto& Ci = C.at(size_t(i));

        const auto dp1 = CNi.rows() + CNi.columns();   // d + 1
        const auto D   = (dp1 + 2) / 2;
        const auto L   = (dp1 + 1) / 2;

        Ci.resize(L, D);

        // retain the corner control point
        Ci(0, 0) = CNim(0, 0);

        // degree elevate the boundaries
        for (auto c_j = 1UL; c_j < L; ++c_j) {
          const auto nj = T(c_j) / T(dp1);
          const auto w0 = nj;
          const auto w1 = 1 - nj;

          auto C0 = CNim(c_j - 1, 0);
          auto C1 = (c_j < Ld) ? CNim(c_j, 0)                     // base case
                               : CNim2(0, CNim2.columns() - 1);   // rotate left

          Ci(c_j, 0) = w0 * C0 + w1 * C1;
        }

        for (auto c_k = 1UL; c_k < D; ++c_k) {
          const auto vk = T(c_k) / T(dp1);
          const auto w0 = vk;
          const auto w1 = 1 - vk;

          auto C0 = CNim(0, c_k - 1);
          auto C1 = (c_k < Dd) ? CNim(0, c_k)              // base case
                               : CNi(CNi.rows() - 1, 0);   // rotate right

          Ci(0, c_k) = w0 * C0 + w1 * C1;
        }

        // degree elevate the interior
        for (auto c_j = 1UL; c_j < L; ++c_j) {
          for (auto c_k = 1UL; c_k < D; ++c_k) {

            const auto nj = T(c_j) / T(dp1);
            const auto vk = T(c_k) / T(dp1);

            // clang-format off
              // compute weights
              std::array<T, 4> w{ nj * vk,
                                 (1 - nj) * vk,
                                  nj * (1 - vk),
                                 (1 - nj) * (1 - vk) };
            // clang-format on

            auto& Cjk = Ci(c_j, c_k);

            // SW
            auto C0 = CNim(c_j - 1, c_k - 1);

            // NW
            auto C1 = (c_j < Ld)
                        ? CNim(c_j, c_k - 1)                     // base case
                        : CNim2(c_k - 1, CNim2.columns() - 1);   // rotate left

            // SE
            auto C2 = (c_k < Dd)
                        ? CNim(c_j - 1, c_k)              // base case
                        : CNi(CNi.rows() - 1, c_j - 1);   // rotate right

            // NE
            Point C3(0);
            if (c_j < Ld) {
              // row within CNim
              C3 = (c_k < Dd) ? CNim(c_j, c_k)              // base case
                              : CNi(CNi.rows() - 1, c_j);   // rotate right
            }
            else if (c_k == Ld and Dd != Ld) {
              // Special case
              C3 = P0;   // central control point as NE corner in the
                         // quadrilateral
            }
            else {
              // row above CNim (i.e. c_j == Ld)
              C3 = (c_k < Ld) ? CNim2(c_k, CNim2.columns() - 1)   // rotate left
                              : CNi(CNi.rows() - 1,
                                    CNi.columns() - 1);   // rotate right
            }

            Cjk = w.at(0) * C0;
            Cjk += w.at(1) * C1;
            Cjk += w.at(2) * C2;
            Cjk += w.at(3) * C3;
          }
        }
      }

      using ReturnType = const ControlNets;
      return std::forward<ReturnType>(C);
    }

  }   // namespace generalizedbezierpatch


  template <gmc::spaces::EucledianSpace EmbedSpace_T
            = spaces::ProjectiveSpace<double, 3ul>>
  struct GeneralizedBezierPatch
    : PolygonalSurface<mappingkernel::MVCPolygonalSurfaceMappingKernel,
                     EmbedSpace_T> {


    /***
     * Boiler plate */

    /* Base */
    using Base = PolygonalSurface<mappingkernel::MVCPolygonalSurfaceMappingKernel,
                                EmbedSpace_T>;
    using EmbedSpaceObject = typename Base::EmbedSpaceObject;
    using EmbedSpace       = typename Base::EmbedSpace;
    using Type             = typename Base::Type;


    /* Domain space */
    using DomainSpace      = typename Base::DomainSpace;
    using Domain           = typename Base::Domain;
    using DomainInfo       = typename Base::DomainInfo;
    using Parameterization = typename Base::Parameterization;
    GM2_MAPPING_KERNEL_BOILER_PLATE



    // Polygon construction types
    template <typename Element_T>
    using PolygonContainer = DVectorT<Element_T>;

    using Lambda    = PolygonContainer<Type>;
    using Lambdas   = PolygonContainer<Lambda>;
    using Polygon2D = PolygonContainer<DPoint>;
    //      using Vertices  = DVectorT<Point>;

    using ControlNet  = DMatrixT<Point>;
    using ControlNets = DVectorT<ControlNet>;
    using Ribbon      = DMatrixT<Point>;
    using Ribbons     = DVectorT<Ribbon>;

    // Constructor(s)
    template <typename... Ts>
    GeneralizedBezierPatch(ControlNets const& cns, Ts&&... ts)
      : Base(std::forward<Ts>(ts)...), m_CNs(cns), m_l{int(m_CNs.at(0).rows())},
        m_d{int(m_CNs.at(0).rows() + m_CNs.at(0).columns()) - 1}
    {
      m_domain.polygon = polygonutils::generateRegularPolygon2D<>(cns.size());
      if (cns.size() < 3)
        throw std::runtime_error("Wrong dimension: Control Nets");

      initConstructRibbons();
    }


    ControlNets m_CNs;
    Ribbons     m_C;
    DomainInfo  m_domain;
//    Polygon2D   m_polygon2d;

    int m_i{0};
    int m_l{0};   // Equals to CNs_i rows; Layers
    int m_d{0};   // Equals to CNs_i colums + CNs_i rows - 1; Degree


    DomainInfo const& domain() const { return m_domain; }












    /**!
     * Derivative of order dirs.size()
     */
    //      VectorH derivativeAt(DPoint const&            par,
    //                           DVectorT<DVector> const& dirs) const
    //      {
    //      }

    PointH evaluateAtGBC(Lambda const& gbc) const
    {
//      const auto  M = m_polygon2d.size();
//      const auto& V = m_polygon2d;
      const auto  M = m_domain.polygon.size();
      const auto& V = m_domain.polygon;
//      const auto& v = par;

      //      std::cout << "Evaluating GBpatch at v = " << v << std::endl;

      // Find MVC
//      const auto gbc = gbc::mvc(V, v);


      // Lambdas
      const auto L = gbc::generateLeftRotatedLambdaSet(gbc);

      // Local coords: s/h-parameterization
      std::vector<Type> Sis(M);
      std::vector<Type> His(M);
      std::transform(std::begin(L), std::end(L), std::begin(Sis),
                     [V](const auto& l) { return gbc::gbcToSMapping(l); });
      std::transform(std::begin(L), std::end(L), std::begin(His),
                     [](const auto& l) { return gbc::gbcToHMapping(l); });

      // Pre-computation - Basis function
      std::vector<DMatrixT<Type>> BssT(M);
      std::vector<DMatrixT<Type>> Bhs(M);
      std::transform(std::begin(Sis), std::end(Sis), std::begin(BssT),
                     [d = this->m_d](const auto& Si) {
                       return blaze::evaluate(blaze::trans(
                         basis::generateBernsteinBasisMatrix(d, Si)));
                     });
      std::transform(
        std::begin(His), std::end(His), std::begin(Bhs),
        [d = this->m_d, l = size_t(this->m_l)](const auto& Hi) {
          return blaze::evaluate(blaze::submatrix(
            basis::generateBernsteinBasisMatrix(d, Hi), 0UL, 0UL, l, l));
        });

      // Pre-compute deficiency weights
      std::vector<DMatrixT<Type>> Mus(M);
      for (size_t i = 0; i < M; ++i) {
        const size_t im = (M + i - 1) % M;
        const size_t ip = (M + i + 1) % M;

        Mus[i] = generateMuMatrix(His[im], His[i], His[ip]);
      }





      const auto C0
        = generalizedbezierpatch::computeC0<GeneralizedBezierPatch>(m_CNs);


      // Position
      Point Si_sum = C0;

      // Per-side evaluation
      for (size_t i = 0; i < M; ++i) {

        const auto& Bhs_i  = Bhs[i];
        const auto& BssT_i = BssT[i];
        const auto& Mus_i  = Mus[i];
        const auto& C_i    = m_C[i];

        // Edge-ribbon evaluation
        Si_sum += (Bhs_i * ((Mus_i % C_i) * BssT_i))(0, 0);

        // Center point correction
        Si_sum -= C0 * (Bhs_i * (Mus_i * BssT_i))(0, 0);
      }

      return blaze::evaluate(
        utils::extendStaticContainer<Point, PointH, VectorDim, 1UL>(Si_sum,
                                                                    Type(1)));
    }


  private:
    void initConstructRibbons()
    {
      // Check control net sizes
      int CN_col_size = int(m_CNs.at(0).columns());
      int CN_row_size = int(m_CNs.at(0).rows());

      // col size > 2
      const auto COL_SIZE_CHECK = CN_col_size >= 2;
      assert(COL_SIZE_CHECK);
      if (not COL_SIZE_CHECK)
        throw std::runtime_error{"Needs degree 3; |CN col| < 2!"};

      // row size == col size || row size == (col_size - 1)
      const auto ROW_COL_SIZE_CHECK
        = (CN_row_size == CN_col_size or CN_row_size == (CN_col_size - 1));
      assert(ROW_COL_SIZE_CHECK);
      if (not ROW_COL_SIZE_CHECK)
        throw std::runtime_error{"|CN row| != {|CN col|,|CN col -1|}!"};

      // Equivalent sized rows and cols;
      for (const auto& CN : m_CNs) {
        const auto EQUIVE_SIZE_CHECK
          = int(CN.columns()) == CN_col_size and int(CN.rows()) == CN_row_size;
        assert(EQUIVE_SIZE_CHECK);
        if (not EQUIVE_SIZE_CHECK)
          throw std::runtime_error{
            "|CN_j row| != |CN_0 row| or |CN_j col| != |CN_0 col|!"};
      }


      // Construct ribbons
      m_C = generalizedbezierpatch::constructRibbonsFromControlNets<
        GeneralizedBezierPatch>(m_CNs);
    }


    Type compute_mu(int j, int k, const Type& him, const Type& hi,
                    const Type& hip) const
    {
      if (k < 2) {
        if (2 <= j and j <= (m_d - 2))
          return Type(1);
        else if (j < 2) {
          if (std::abs(hi + him) < 1e-7) {
            return Type(0.5);
          }
          return him / (him + hi);
        }
        else {   // j > d - 2
          if (std::abs(hi + hip) < 1e-7) {
            return Type(0.5);
          }
          return hip / (hip + hi);
        }
      }
      else {   //( k >= 3)
        if (j < m_l) {
          if (j < k)
            return Type(0);
          else if (j == k)
            return Type(0.5);
          else   // (j>k)
            return Type(1);
        }
        else {   // j >= l
          if (j > m_d - k)
            return Type(0);
          else if (j == m_d - k)
            return Type(0.5);
          else
            return Type(1);   // (j< d-k)
        }
      }
    }

    DMatrixT<Type> generateMuMatrix(const Type& him, const Type& hi,
                                    const Type& hip) const
    {
      DMatrixT<Type> Mu(size_t(m_l), size_t(m_d + 1));
      for (size_t k = 0; k < Mu.rows(); ++k)
        for (size_t j = 0; j < Mu.columns(); ++j)
          Mu(k, j) = compute_mu(int(j), int(k), him, hi, hip);
      return Mu;
    }

    static constexpr divideddifference::fd::CentralDifference D{};
  };

}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_POLYGONALSURFACE_GENERALIZEDBEZIERPATCH_H
