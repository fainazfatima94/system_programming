// gtest
#include <gtest/gtest.h>   // googletest header file


// gmlib2
#include <parametric/ppoint.h>
#include <parametric/curve.h>
#include <parametric/surface.h>
#include <parametric/polygonsurface.h>


TEST(Parametric_ParametricObject_EvaluationCtrl_Traits,
     Point_Default_ObjEvalCtrl_isSEvaluationCtrl)
{
  using PPoint = gmlib2::parametric::PPoint<>;
  EXPECT_TRUE(gmlib2::traits::Is_PObjEvalCtrl<PPoint::PObjEvalCtrl>::value);
}

TEST(Parametric_ParametricObject_EvaluationCtrl_Traits,
     evaluationctrl_PPointEvaluationCtrl_isSEvaluationCtrl)
{
  using PPoint = gmlib2::parametric::PPoint<
    gmlib2::ProjectiveSpaceObject<>,
    gmlib2::parametric::evaluationctrl::PPointEvalCtrl>;
  EXPECT_TRUE(gmlib2::traits::Is_PObjEvalCtrl<PPoint::PObjEvalCtrl>::value);
}

TEST(Parametric_ParametricObject_EvaluationCtrl_Traits,
     Curve_Default_ObjEvalCtrl_isSEvaluationCtrl)
{
  using Curve = gmlib2::parametric::Curve<>;
  EXPECT_TRUE(gmlib2::traits::Is_PObjEvalCtrl<Curve::PObjEvalCtrl>::value);
}

TEST(Parametric_ParametricObject_EvaluationCtrl_Traits,
     evaluationctrl_CurveEvaluationCtrl_isSEvaluationCtrl)
{
  using Curve = gmlib2::parametric::Curve<
    gmlib2::ProjectiveSpaceObject<>,
    gmlib2::parametric::evaluationctrl::CurveEvalCtrl>;
  EXPECT_TRUE(gmlib2::traits::Is_PObjEvalCtrl<Curve::PObjEvalCtrl>::value);
}


TEST(Parametric_ParametricObject_EvaluationCtrl_Traits,
     PSurface_Default_PObjEvalCtrl_isSEvaluationCtrl)
{
  using Surface = gmlib2::parametric::Surface<>;
  EXPECT_TRUE(gmlib2::traits::Is_PObjEvalCtrl<Surface::PObjEvalCtrl>::value);
}

TEST(Parametric_ParametricObject_EvaluationCtrl_Traits,
     evaluationctrl_SurfaceEvaluationCtrl_isSEvaluationCtrl)
{
  using Surface = gmlib2::parametric::Surface<
    gmlib2::ProjectiveSpaceObject<>,
    gmlib2::parametric::evaluationctrl::SurfaceEvalCtrl>;
  EXPECT_TRUE(gmlib2::traits::Is_PObjEvalCtrl<Surface::PObjEvalCtrl>::value);
}

TEST(Parametric_ParametricObject_EvaluationCtrl_Traits,
     PolygonSurfaceConstruction_Default_ObjEvalCtrl_isSEvaluationCtrl)
{
  using PolygonSurface = gmlib2::parametric::PolygonSurface<>;
  EXPECT_TRUE(
    gmlib2::traits::Is_PObjEvalCtrl<PolygonSurface::PObjEvalCtrl>::value);
}

TEST(Parametric_ParametricObject_EvaluationCtrl_Traits,
     evaluationctrl_PolyConstrEvalCtrl_isSEvaluationCtrl)
{
  using PolygonSurface = gmlib2::parametric::PolygonSurface<
    gmlib2::ProjectiveSpaceObject<>,
    gmlib2::parametric::evaluationctrl::PolygonSurfaceEvalCtrl>;
  EXPECT_TRUE(
    gmlib2::traits::Is_PObjEvalCtrl<PolygonSurface::PObjEvalCtrl>::value);
}
