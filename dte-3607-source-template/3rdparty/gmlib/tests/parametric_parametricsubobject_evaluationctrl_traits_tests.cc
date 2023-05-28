// gtest
#include <gtest/gtest.h>   // googletest header file


// gmlib2
#include <parametric/classic_objects/pline.h>
#include <parametric/classic_objects/pcircle.h>
#include <parametric/subobject_constructions/psubcurve.h>

#include <parametric/classic_objects/pplane.h>
#include <parametric/classic_objects/ptorus.h>
#include <parametric/subobject_constructions/psubsurface.h>



TEST(Parametric_ParametricSubObject_EvaluationCtrl_Traits,
     PSubCurveOnCurve_Default_PObjEvalCtrl_isSEvaluationCtrl)
{
  namespace gm2p = gmlib2::parametric;

  using PCurveOnCurveType
    = gm2p::PSubCurveOnCurve<gm2p::PLine, gm2p::PCircle<>>;
  EXPECT_TRUE(gmlib2::traits::Is_PSubObjEvalCtrl<
              PCurveOnCurveType::PSubObjEvalCtrl>::value);
}

TEST(Parametric_ParametricSubObject_EvaluationCtrl_Traits,
     evaluationctrl_PSubCurveOnCurveEvalCtrl_isSEvaluationCtrl)
{
  namespace gm2p = gmlib2::parametric;

  using PCurveOnCurveType
    = gm2p::PSubCurveOnCurve<gm2p::PLine, gm2p::PCircle<>,
                             gm2p::PCircle<>::BaseSpaceObjectBase,
                             gm2p::evaluationctrl::PSubCurveOnCurveEvalCtrl>;
  EXPECT_TRUE(gmlib2::traits::Is_PSubObjEvalCtrl<
              PCurveOnCurveType::PSubObjEvalCtrl>::value);
}

TEST(Parametric_ParametricSubObject_EvaluationCtrl_Traits,
     PSubCurveOnSurface_Default_PObjEvalCtrl_isSEvaluationCtrl)
{
  namespace gm2p = gmlib2::parametric;

  using PCurveOnSurfaceType
    = gm2p::PSubCurveOnSurface<gm2p::PLine, gm2p::PTorus<>>;
  EXPECT_TRUE(gmlib2::traits::Is_PSubObjEvalCtrl<
              PCurveOnSurfaceType::PSubObjEvalCtrl>::value);
}

TEST(Parametric_ParametricSubObject_EvaluationCtrl_Traits,
     evaluationctrl_PSubCurveOnSurfaceEvalCtrl_isSEvaluationCtrl)
{
  namespace gm2p = gmlib2::parametric;

  using PCurveOnSurfaceType = gm2p::PSubCurveOnSurface<
    gm2p::PLine, gm2p::PTorus<>, gm2p::PTorus<>::BaseSpaceObjectBase,
    gm2p::evaluationctrl::PSubCurveOnSurfaceEvalCtrl>;
  EXPECT_TRUE(gmlib2::traits::Is_PSubObjEvalCtrl<
              PCurveOnSurfaceType::PSubObjEvalCtrl>::value);
}

TEST(Parametric_ParametricSubObject_EvaluationCtrl_Traits,
     PSubSurfaceOnSurface_Default_PObjEvalCtrl_isSEvaluationCtrl)
{
  namespace gm2p = gmlib2::parametric;

  using PSurfaceOnSurfaceType
    = gm2p::PSubSurfaceOnSurface<gm2p::PPlane, gm2p::PTorus<>>;
  EXPECT_TRUE(gmlib2::traits::Is_PSubObjEvalCtrl<
              PSurfaceOnSurfaceType::PSubObjEvalCtrl>::value);
}

TEST(Parametric_ParametricSubObject_EvaluationCtrl_Traits,
     evaluationctrl_PSubSurfaceOnSurfaceEvalCtrl_isSEvaluationCtrl)
{
  namespace gm2p = gmlib2::parametric;

  using PSurfaceOnSurfaceType = gm2p::PSubSurfaceOnSurface<
    gm2p::PPlane, gm2p::PTorus<>, gm2p::PTorus<>::BaseSpaceObjectBase,
    gm2p::evaluationctrl::PSubSurfaceOnSurfaceEvalCtrl>;
  EXPECT_TRUE(gmlib2::traits::Is_PSubObjEvalCtrl<
              PSurfaceOnSurfaceType::PSubObjEvalCtrl>::value);
}
