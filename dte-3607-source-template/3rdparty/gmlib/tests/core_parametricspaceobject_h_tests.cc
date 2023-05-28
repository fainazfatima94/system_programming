// gtest
#include <gtest/gtest.h>   // googletest header file

// gmlib2
#include <parametric/parametricobject.h>
#include <parametric/ppoint.h>
#include <parametric/classic_objects/pcircle.h>
#include <parametric/classic_objects/pline.h>
#include <parametric/classic_objects/pplane.h>
#include <parametric/classic_objects/psphere.h>
#include <parametric/classic_objects/ptorus.h>
#include <parametric/classic_constructions/phermitecurve.h>
#include <parametric/classic_constructions/pbilinearcoonspatch.h>
#include <parametric/classic_constructions/pbicubiccoonspatch.h>

using namespace gmlib2;
using namespace gmlib2::parametric;






namespace unittest_detail
{



  template <typename SpaceObjectEmbedBase_T = ProjectiveSpaceObject<>>
  class ZeroDerCurve : public PCurve<SpaceObjectEmbedBase_T> {
    using Base = PCurve<SpaceObjectEmbedBase_T>;

  public:
    GM2_DEFINE_DEFAULT_PARAMETRIC_OBJECT_TYPES

    // Constructor
    using Base::Base;

    // PCurve interface
  public:
    PSpaceBoolArray isClosed() const override { return {{false}}; }
    PSpacePoint     startParameters() const override { return PSpacePoint{0}; }
    PSpacePoint     endParameters() const override { return PSpacePoint{1}; }

  protected:
    EvaluationResult evaluate(const PSpacePoint& /*par*/,
                              const PSpaceSizeArray& /*no_der*/,
                              const PSpaceBoolArray& /*from_left*/
                              ) const override
    {
      return VectorH{0};
    }
  };


  using EmbedSpaceInfo = spaces::D3R3SpaceInfo<>;
  using SO             = ProjectiveSpaceObject<EmbedSpaceInfo>;
  using TestHC         = PHermiteCurveP2V2<SO>;
  using TestZDC        = unittest_detail::ZeroDerCurve<SO>;
  using TestCP         = PBicubicCoonsPatch<SO>;

  auto constructBicubicCoonsValidBaseNonValidZeroDer = []() {
    auto c0 = std::make_unique<TestHC>(
      TestHC::Point{-2.0, -2.0, 0.0}, TestHC::Point{-2.0, 2.0, 0.0},
      //    TestHC::Vector{0.0, 4.0, -10.0},
      TestHC::Vector{0.0, 4.0, 0.0}, TestHC::Vector{0.0, 4.0, 0.0});

    auto c1 = std::make_unique<TestHC>(
      TestHC::Point{2.0, -2.0, 0.0}, TestHC::Point{2.0, 2.0, 0.0},
      TestHC::Vector{0.0, 4.0, 40.0}, TestHC::Vector{0.0, 4.0, 4.0}
      //                 TestHC::Vector{0.0, 4.0, 0.0}, TestHC::Vector{0.0, 4.0,
      //                 0.0}
    );

    auto g0 = std::make_unique<TestHC>(
      TestHC::Point{-2.0, -2.0, 0.0}, TestHC::Point{2.0, -2.0, 0.0},
      TestHC::Vector{4.0, 0.0, 0.0}, TestHC::Vector{4.0, 0.0, 0.0});

    auto g1 = std::make_unique<TestHC>(
      TestHC::Point{-2.0, 2.0, 0.0}, TestHC::Point{2.0, 2.0, 0.0},
      TestHC::Vector{4.0, 0.0, 4.0}, TestHC::Vector{4.0, 0.0, 4.0});


    auto c2 = std::make_unique<TestZDC>();
    auto c3 = std::make_unique<TestZDC>();
    auto g2 = std::make_unique<TestZDC>();
    auto g3 = std::make_unique<TestZDC>();

    // Bicubic Coons patch
    auto bccoons
      = std::make_unique<TestCP>(c0.get(), c1.get(), c2.get(), c3.get(),
                                 g0.get(), g1.get(), g2.get(), g3.get());

    return std::make_tuple(std::move(bccoons), std::move(c0), std::move(c1),
                           std::move(c2), std::move(c3), std::move(g0),
                           std::move(g1), std::move(g2), std::move(g3));
  };

  auto test_bicubic_coons_boundary = [](auto* patch, auto* bcurve, auto fix_v,
                                        auto fix_other_at_start,
                                        auto resolution) {
    auto pi    = fix_v ? 0UL : 1UL;
    auto p_s   = patch->startParameters();
    auto p_e   = patch->endParameters();
    auto pe_dt = (p_e[pi] - p_s[pi]) / double(resolution - 1);

    auto bc_s  = bcurve->startParameters();
    auto bc_e  = bcurve->endParameters();
    auto bc_dt = (bc_e - bc_s) / double(resolution - 1);

    for (size_t i{0}; i < resolution; ++i) {

      SCOPED_TRACE("i: " + std::to_string(i));
      auto eval_p_par = TestCP::PSpacePoint{
        fix_v ? i * pe_dt : (fix_other_at_start ? p_s[0] : p_e[0]),
        fix_v ? (fix_other_at_start ? p_s[1] : p_e[1]) : i * pe_dt};
      auto eval_b_par = i * bc_dt;
      auto p_eval     = patch->evaluateLocal(eval_p_par, {{0, 0}})(0, 0);
      auto b_eval     = bcurve->evaluateLocal(eval_b_par, {{0}})[0];
      std::cout << "################### i: " << i << std::endl;
      std::cout << "     #### SURFACE; edge dt: " << pe_dt << ", par(u,v): ("
                << eval_p_par[0] << "," << eval_p_par[1] << ")" << std::endl;
      std::cout << "     ---- p_eval: " << std::endl << p_eval << std::endl;
      std::cout << "     #### CURVE; dt: " << bc_dt[0] << ", par(t): ("
                << eval_b_par[0] << ")" << std::endl;
      std::cout << "     ---- b_eval: " << std::endl << b_eval << std::endl;

      EXPECT_TRUE(blaze::length(p_eval - b_eval) < 1e-7);
      //                EXPECT_DOUBLE_EQ(blaze::length(p_eval -
      //                b_eval)); // gives false positives/negatives
    }
  };
}   // namespace unittest_detail










TEST(Core_ParametricSpaceObject, Stack_Compile_Test)
{
  using EmbedSpaceInfo = spaces::D3R3SpaceInfo<>;
  using SO             = ProjectiveSpaceObject<EmbedSpaceInfo>;

  using CurvePSpaceInfo = spaces::ParameterFVSpaceInfo<1, 1>;
  using CurvePSO
    = ParametricObject<CurvePSpaceInfo, evaluationctrl::pcurve_tag, SO,evaluationctrl::PCurveEvalCtrl>;

  using SurfacePSpaceInfo = spaces::ParameterFVSpaceInfo<2, 2>;
  using SurfacePSO
    = ParametricObject<SurfacePSpaceInfo, evaluationctrl::psurface_tag, SO,evaluationctrl::PSurfaceEvalCtrl>;

  // Space object
  [[maybe_unused]] SO so;

  //
  [[maybe_unused]] PPoint<SO> ppoint;

  // Curves
  [[maybe_unused]] PCircle<SO>           pcircle;
  [[maybe_unused]] PLine<SO>             pline;
  [[maybe_unused]] PHermiteCurveP2V2<SO> phermitecurve_p2v2;
  phermitecurve_p2v2.evaluateLocal(PHermiteCurveP2V2<SO>::PSpacePoint{0.0},
                                   {{2}});
  phermitecurve_p2v2.evaluateLocal(PHermiteCurveP2V2<SO>::PSpacePoint{0.5},
                                   {{2}});
  phermitecurve_p2v2.evaluateLocal(PHermiteCurveP2V2<SO>::PSpacePoint{1.0},
                                   {{2}});

  // Curves - "transitive parametric space objects"
  [[maybe_unused]] PLine<CurvePSO>   pline_pso;
  [[maybe_unused]] PCircle<CurvePSO> pcircle_pso;

  // Surfaces
  [[maybe_unused]] PPlane<SO>  tpplane;
  [[maybe_unused]] PSphere<SO> tpsphere;
  [[maybe_unused]] PTorus<SO>  tptorus;

  // Surfaces - "transitive parametric space objects"
  [[maybe_unused]] PPlane<SurfacePSO>  tpplane_pso;
  [[maybe_unused]] PSphere<SurfacePSO> tpsphere_pso;
  [[maybe_unused]] PTorus<SurfacePSO>  tptorus_pso;

  EXPECT_TRUE(true);
}

TEST(Core_ParametricSapceObject,
     DISABLED_HermiteCurveP2V2_InputEvalExpectations)
{

  auto test_start_end_expectations = [&](auto p0, auto p1, auto v0, auto v1) {
    auto curve = PHermiteCurveP2V2<>(p0, p1, v0, v1);

    auto evalLocal_start
      = curve.evaluateLocal(curve.startParameters(), {{1}}, {{true}});
    auto evalLocal_end
      = curve.evaluateLocal(curve.endParameters(), {{1}}, {{true}});

    auto eval_p0 = blaze::subvector(evalLocal_start[0], 0UL,
                                    PHermiteCurveP2V2<>::VectorDim);
    auto eval_v0 = blaze::subvector(evalLocal_start[1], 0UL,
                                    PHermiteCurveP2V2<>::VectorDim);
    auto eval_p1
      = blaze::subvector(evalLocal_end[0], 0UL, PHermiteCurveP2V2<>::VectorDim);
    auto eval_v1
      = blaze::subvector(evalLocal_end[1], 0UL, PHermiteCurveP2V2<>::VectorDim);

    auto evalParent_start
      = curve.evaluateParent(curve.startParameters(), {{1}}, {{true}});
    auto evalParent_end
      = curve.evaluateParent(curve.endParameters(), {{1}}, {{true}});

    auto eval_parent_p0 = blaze::subvector(evalParent_start[0], 0UL,
                                           PHermiteCurveP2V2<>::VectorDim);
    auto eval_parent_v0 = blaze::subvector(evalParent_start[1], 0UL,
                                           PHermiteCurveP2V2<>::VectorDim);
    auto eval_parent_p1 = blaze::subvector(evalParent_end[0], 0UL,
                                           PHermiteCurveP2V2<>::VectorDim);
    auto eval_parent_v1 = blaze::subvector(evalParent_end[1], 0UL,
                                           PHermiteCurveP2V2<>::VectorDim);

    // pure evaluate local tests
    EXPECT_DOUBLE_EQ(blaze::length(eval_p0 - p0), 0);
    EXPECT_DOUBLE_EQ(blaze::length(eval_p1 - p1), 0);
    EXPECT_DOUBLE_EQ(blaze::length(eval_v0 - v0), 0);
    EXPECT_DOUBLE_EQ(blaze::length(eval_v1 - v1), 0);

    // pure evaluate parent tests
    EXPECT_DOUBLE_EQ(blaze::length(eval_parent_p0 - p0), 0);
    EXPECT_DOUBLE_EQ(blaze::length(eval_parent_p1 - p1), 0);
    EXPECT_DOUBLE_EQ(blaze::length(eval_parent_v0 - v0), 0);
    EXPECT_DOUBLE_EQ(blaze::length(eval_parent_v1 - v1), 0);
  };

  {
    SCOPED_TRACE("Curve01");
    test_start_end_expectations(PHermiteCurveP2V2<>::Point{-2.0, -2.0, 0.0},
                                PHermiteCurveP2V2<>::Point{2.0, -2.0, 0.0},
                                PHermiteCurveP2V2<>::Vector{4.0, 0.0, 0.0},
                                PHermiteCurveP2V2<>::Vector{4.0, 0.0, 0.0});
  }

  {
    SCOPED_TRACE("Curve02");
    test_start_end_expectations(PHermiteCurveP2V2<>::Point{-2.0, 2.0, 0.0},
                                PHermiteCurveP2V2<>::Point{2.0, 2.0, 4.0},
                                PHermiteCurveP2V2<>::Vector{4.0, 0.0, 4.0},
                                PHermiteCurveP2V2<>::Vector{4.0, 0.0, 4.0});
  }

  {
    SCOPED_TRACE("Curve03");
    test_start_end_expectations(PHermiteCurveP2V2<>::Point{-2.0, -2.0, 0.0},
                                PHermiteCurveP2V2<>::Point{-2.0, 2.0, 0.0},
                                PHermiteCurveP2V2<>::Vector{0.0, 4.0, -10.0},
                                PHermiteCurveP2V2<>::Vector{0.0, 4.0, 0.0});
  }

  {
    SCOPED_TRACE("Curve04");
    test_start_end_expectations(PHermiteCurveP2V2<>::Point{2.0, -2.0, 0.0},
                                PHermiteCurveP2V2<>::Point{2.0, 2.0, 4.0},
                                PHermiteCurveP2V2<>::Vector{0.0, 4.0, 4.0},
                                PHermiteCurveP2V2<>::Vector{0.0, 40.0, 4.0});
  }
}

TEST(Core_ParametricSpaceObject, BilinearCoonsPatchValidateConstruction)
{

  using EmbedSpaceInfo = spaces::D3R3SpaceInfo<>;
  using SO             = ProjectiveSpaceObject<EmbedSpaceInfo>;

  // clang-format off
  auto constructBilinearCoonsPatch = [](
      PLine<SO>::Point c0p, PLine<SO>::Vector c0v,
      PLine<SO>::Point c1p, PLine<SO>::Vector c1v,
      PLine<SO>::Point g0p, PLine<SO>::Vector g0v,
      PLine<SO>::Point g1p, PLine<SO>::Vector g1v ) {

        auto c0       = std::make_unique<PLine<SO>>(c0p, c0v);
        auto c1       = std::make_unique<PLine<SO>>(c1p, c1v);
        auto g0       = std::make_unique<PLine<SO>>(g0p, g0v);
        auto g1       = std::make_unique<PLine<SO>>(g1p, g1v);
        auto bl_coons = std::make_unique<PBilinearCoonsPatch<SO>>(
          c0.get(), c1.get(), g0.get(), g1.get());

        return std::make_tuple(std::move(bl_coons), std::move(c0),
                               std::move(c1), std::move(g0), std::move(g1));

      };
  // clang-format on


  auto coons01 = constructBilinearCoonsPatch(
    PLine<SO>::Point{0.0, 0.0, 0.0}, PLine<SO>::Vector{2.0, 0.0, 0.0},
    PLine<SO>::Point{0.0, 2.0, 0.0}, PLine<SO>::Vector{2.0, 0.0, 0.0},
    PLine<SO>::Point{0.0, 0.0, 0.0}, PLine<SO>::Vector{0.0, 2.0, 0.0},
    PLine<SO>::Point{2.0, 0.0, 0.0}, PLine<SO>::Vector{0.0, 2.0, 0.0});
  EXPECT_TRUE(std::get<0>(coons01)->validateConstruction());

  auto coons02 = constructBilinearCoonsPatch(
    PLine<SO>::Point{0.0, 0.0, 0.0}, PLine<SO>::Vector{2.0, 0.0, 0.0},
    PLine<SO>::Point{0.0, 2.0, 0.0}, PLine<SO>::Vector{2.0, 0.0, 0.0},
    PLine<SO>::Point{0.0, 0.0, 0.0}, PLine<SO>::Vector{0.0, 2.0, 0.0},
    PLine<SO>::Point{2.0, 0.0, 0.0}, PLine<SO>::Vector{0.0, 4.0, 0.0});
  EXPECT_FALSE(std::get<0>(coons02)->validateConstruction());

  auto coons03 = constructBilinearCoonsPatch(
    PLine<SO>::Point{0.0, 0.0, 0.0}, PLine<SO>::Vector{2.0, 0.0, -40.0},
    PLine<SO>::Point{0.0, 2.0, 0.0}, PLine<SO>::Vector{2.0, 0.0, 0.0},
    PLine<SO>::Point{0.0, 0.0, 0.0}, PLine<SO>::Vector{0.0, 2.0, 0.0},
    PLine<SO>::Point{2.0, 0.0, 0.0}, PLine<SO>::Vector{0.0, -4.0, 0.0});
  EXPECT_FALSE(std::get<0>(coons03)->validateConstruction());
}


TEST(Core_ParametricSpaceObject, BicubicCoonsPatchValidateConstructions)
{
  auto bc_tuple
    = unittest_detail::constructBicubicCoonsValidBaseNonValidZeroDer();
  EXPECT_FALSE(std::get<0>(bc_tuple)->validateConstruction());
}

TEST(Core_ParametricSpaceObject, BicubicCoonsPatchValidateCorners)
{

  //  using EmbedSpaceInfo = spaces::D3R3SpaceInfo<>;
  //  using SO             = SpaceObject<EmbedSpaceInfo>;
  //  using TestCP         = PBicubicCoonsPatch<SO>;

  auto evalCEnds = [](auto* c, auto nd, auto at_start, auto component) {
    if (at_start)
      return c->evaluateParent(c->startParameters(), {{nd}})[component];
    else
      return c->evaluateParent(c->endParameters(), {{nd}})[component];
  };

  auto evalSCorners = [](auto* s, auto u_at_start, auto v_at_start) {
    if (u_at_start and v_at_start)
      return s->evaluateParent(s->startParameters(), {{0, 0}})(0, 0);
    else if ((not u_at_start) and v_at_start)
      return s->evaluateParent(
        unittest_detail::TestCP::PSpacePoint{
          {s->endParameters()[0], s->startParameters()[1]}},
        {{0, 0}})(0, 0);
    else if (u_at_start and (not v_at_start))
      return s->evaluateParent(
        unittest_detail::TestCP::PSpacePoint{
          {s->startParameters()[0], s->endParameters()[1]}},
        {{0, 0}})(0, 0);
    else
      return s->evaluateParent(s->endParameters(), {{0, 0}})(0, 0);
  };



  auto bc_tuple
    = unittest_detail::constructBicubicCoonsValidBaseNonValidZeroDer();

  auto c0_0 = evalCEnds(std::get<1>(bc_tuple).get(), 0UL, true, 0UL);
  auto c0_1 = evalCEnds(std::get<1>(bc_tuple).get(), 0UL, false, 0UL);

  auto c1_0 = evalCEnds(std::get<2>(bc_tuple).get(), 0UL, true, 0UL);
  auto c1_1 = evalCEnds(std::get<2>(bc_tuple).get(), 0UL, false, 0UL);

  auto g0_0 = evalCEnds(std::get<5>(bc_tuple).get(), 0UL, true, 0UL);
  auto g0_1 = evalCEnds(std::get<5>(bc_tuple).get(), 0UL, false, 0UL);

  auto g1_0 = evalCEnds(std::get<6>(bc_tuple).get(), 0UL, true, 0UL);
  auto g1_1 = evalCEnds(std::get<6>(bc_tuple).get(), 0UL, false, 0UL);

  //////////
  // Corners
  auto bc_00 = evalSCorners(std::get<0>(bc_tuple).get(), true, true);
  auto bc_10 = evalSCorners(std::get<0>(bc_tuple).get(), false, true);
  auto bc_01 = evalSCorners(std::get<0>(bc_tuple).get(), true, false);
  auto bc_11 = evalSCorners(std::get<0>(bc_tuple).get(), false, false);

  // "lower left" - u/v:0/0
  EXPECT_DOUBLE_EQ(blaze::length(c0_0 - bc_00), 0);
  EXPECT_DOUBLE_EQ(blaze::length(g0_0 - bc_00), 0);

  // "lower right" - u/v:1/0
  EXPECT_DOUBLE_EQ(blaze::length(c1_0 - bc_10), 0);
  EXPECT_DOUBLE_EQ(blaze::length(g0_1 - bc_10), 0);

  // "upper left" - u/v:0/1
  EXPECT_DOUBLE_EQ(blaze::length(c0_1 - bc_01), 0);
  EXPECT_DOUBLE_EQ(blaze::length(g1_0 - bc_01), 0);

  // "lower right" - u/v:1/1
  EXPECT_DOUBLE_EQ(blaze::length(c1_1 - bc_11), 0);
  EXPECT_DOUBLE_EQ(blaze::length(g1_1 - bc_11), 0);
}

TEST(Core_ParametricSpaceObject, BicubicCoonsPatchValidateEdgeUfreeVis0)
{
  auto bc_tuple
    = unittest_detail::constructBicubicCoonsValidBaseNonValidZeroDer();

  // [u,v=0] vs. g0
  SCOPED_TRACE("u,v=0");
  unittest_detail::test_bicubic_coons_boundary(
    std::get<0>(bc_tuple).get(), std::get<5>(bc_tuple).get(), true, true, 10UL);
}

TEST(Core_ParametricSpaceObject, BicubicCoonsPatchValidateEdgeUfreeVis1)
{
  auto bc_tuple
    = unittest_detail::constructBicubicCoonsValidBaseNonValidZeroDer();

  // [u,v=1] vs. g1
  SCOPED_TRACE("u,v=1");
  unittest_detail::test_bicubic_coons_boundary(std::get<0>(bc_tuple).get(),
                                               std::get<6>(bc_tuple).get(),
                                               true, false, 10UL);
}

TEST(Core_ParametricSpaceObject, BicubicCoonsPatchValidateEdgeUis0Vfree)
{
  auto bc_tuple
    = unittest_detail::constructBicubicCoonsValidBaseNonValidZeroDer();

  // [u=0,v] vs. c0
  SCOPED_TRACE("u=0,v");
  unittest_detail::test_bicubic_coons_boundary(std::get<0>(bc_tuple).get(),
                                               std::get<1>(bc_tuple).get(),
                                               false, true, 10UL);
}

TEST(Core_ParametricSpaceObject, BicubicCoonsPatchValidateEdgeUis1Vfree)
{
  auto bc_tuple
    = unittest_detail::constructBicubicCoonsValidBaseNonValidZeroDer();

  // [u=1,v] vs. c1
  SCOPED_TRACE("u=1,v");
  unittest_detail::test_bicubic_coons_boundary(std::get<0>(bc_tuple).get(),
                                               std::get<2>(bc_tuple).get(),
                                               false, false, 10UL);
}









TEST(Core_ParametricSpaceObject, DISABLED_PSurfSubCurve_DUMMY) {}
