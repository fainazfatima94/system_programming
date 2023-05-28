// gtest
#include <gtest/gtest.h>   // googletest header file

// gmlib2
#include <core/gm2_blaze.h>
using namespace gmlib2;
using namespace gmlib2::algorithms;

#include <random>

namespace unittest_detail
{

  SqMatrixT<double, 3> genGoldRMatrixD3(double th, const VectorT<double, 3>& u);

  SqMatrixT<double, 3> genGoldRMatrixD3(double th, const VectorT<double, 3>& u)
  {

    auto ux = u[0];
    auto uy = u[1];
    auto uz = u[2];

    auto uxy = ux * uy;
    auto uxz = ux * uz;
    auto uyz = uy * uz;

    auto u2x = ux * ux;
    auto u2y = uy * uy;
    auto u2z = uz * uz;

    auto cth = std::cos(th);
    auto sth = std::sin(th);

    // Row x Col | [Row,Col]
    SqMatrixT<double, 3> R;
    R(0, 0) = cth + u2x * (1.0 - cth);
    R(0, 1) = uxy * (1.0 - cth) - uz * sth;
    R(0, 2) = uxz * (1.0 - cth) + uy * sth;

    R(1, 0) = uxy * (1.0 - cth) + uz * sth;
    R(1, 1) = cth + u2y * (1.0 - cth);
    R(1, 2) = uyz * (1.0 - cth) - ux * sth;

    R(2, 0) = uxz * (1.0 - cth) - uy * sth;
    R(2, 1) = uyz * (1.0 - cth) + ux * sth;
    R(2, 2) = cth + u2z * (1.0 - cth);

    return R;
  }

  template <typename RMat>
  void blazeRotMatrixChecks(const RMat& R, const RMat& Gold)
  {


    for (size_t ri = 0; ri < R.rows(); ++ri)
      for (size_t ci = 0; ci < R.columns(); ++ci)
        EXPECT_FLOAT_EQ(Gold(ri, ci), R(ri, ci));
  }

}   // namespace unittestutils

TEST(Core_GM2_Blaze, rotationMatrix)
{
  std::random_device rd;
  std::mt19937 rng(rd());

  std::uniform_real_distribution<double> pi_dist(0.0, M_PI);
  std::uniform_real_distribution<double> vec_dist(0.0, 1.0);

  // Test rotation around the axes
  std::array<VectorT<double, 3>, 3> axes{VectorT<double, 3>{1.0, 0.0, 0.0},
                                         VectorT<double, 3>{0.0, 1.0, 0.0},
                                         VectorT<double, 3>{0.0, 0.0, 1.0}};
  for (auto axis : axes) {
    auto ang_rad = 0.2 * M_PI;
    auto R       = rotationMatrix(ang_rad, axis);
    auto Rgold   = unittest_detail::genGoldRMatrixD3(ang_rad, axis);
    unittest_detail::blazeRotMatrixChecks(R, Rgold);
  }

  // Test 100 random angles and directions
  for (auto i=0; i < 100; ++i) {
    auto ang_rad = 2.0 * pi_dist(rng);

    auto v1 = vec_dist(rng);
    auto v2 = vec_dist(rng);
    auto v3 = 1.0 - v2;

    auto               axis2 = VectorT<double, 3>{v1, v2, v3};
    VectorT<double, 3> axis  = normalize(axis2);

    auto R     = rotationMatrix(ang_rad, axis);
    auto Rgold = unittest_detail::genGoldRMatrixD3(ang_rad, axis);

    //    std::cout << "R: " << std::endl << R << std::endl;
    //    std::cout << "Rgold: " << std::endl << Rgold << std::endl;

    unittest_detail::blazeRotMatrixChecks(R, Rgold);
  }
}

TEST(Core_GM2_Blaze, geometricMatrixVectorMultCompileTests)
{

  using EvalType      = VectorT<double, 4>;
  using CVecType      = ColVectorT<double, 2>;
  using RVecType      = RowVectorT<double, 2>;
  using MatOfEvalType = MatrixT<EvalType, 2, 2>;


  const auto u = 0.5;
  const auto v = 0.3;

  CVecType Bu{1 - u, u};
  RVecType Bv{1 - v, v};
  MatOfEvalType M{{EvalType{0.0, 0.0, 0.0, 1.0}, EvalType{1.0, 0.0, 0.0, 1.0}},
                  {EvalType{0.0, 1.0, 0.0, 1.0}, EvalType{1.0, 1.0, 0.0, 1.0}}};

  [[maybe_unused]] auto res = Bv * M * Bu;
  //  std::cout << "res: " << res << std::endl;
  EXPECT_TRUE(true);
}
