// gtest
#include <gtest/gtest.h>   // googletest header file


#include "parametricobject_tests.h"

// gmlib2
#include <parametric/classic_objects/pcircle.h>
#include <parametric/classic_objects/pline.h>
#include <parametric/subobject_constructions/psubcurve.h>

#include <parametric/classic_objects/pplane.h>
#include <parametric/classic_objects/ptorus.h>
#include <parametric/subobject_constructions/psubsurface.h>



TEST(Parametric_SubSpaceObject_Traits,
     PLineSubCurveOnPCircle_isParametricObject)
{
  SCOPED_TRACE("");
  using PSubLineOnPCircle
    = gmlib2::parametric::PSubCurveOnCurve<gmlib2::parametric::PLine,
                                           gmlib2::parametric::PCircle<>>;
  gmlib2::testing::isParametricObject<PSubLineOnPCircle>();
}


TEST(Parametric_SubSpaceObject_Traits, PLineSubCurveOnPTorus_isParametricObject)
{
  SCOPED_TRACE("");
  using PSubLineOnPTorus
    = gmlib2::parametric::PSubCurveOnCurve<gmlib2::parametric::PLine,
                                           gmlib2::parametric::PTorus<>>;
  gmlib2::testing::isParametricObject<PSubLineOnPTorus>();
}

TEST(Parametric_SubSpaceObject_Traits,
     PPlaneSubSurfaceOnPTorus_isParametricObject)
{
  SCOPED_TRACE("");
  using PSubPlaneOnPTorus
    = gmlib2::parametric::PSubSurfaceOnSurface<gmlib2::parametric::PPlane,
                                               gmlib2::parametric::PTorus<>>;
  gmlib2::testing::isParametricObject<PSubPlaneOnPTorus>();
}

TEST(Parametric_SubSpaceObject_Traits,
     PLineSubCurveOnPPlaneSubSurfaceOnPTorus_isParametricObject)
{
  SCOPED_TRACE("");
  using PSubPlaneOnPTorus
    = gmlib2::parametric::PSubSurfaceOnSurface<gmlib2::parametric::PPlane,
                                               gmlib2::parametric::PTorus<>>;
  using PSubLineOnPSubPlaneOnPTorus
    = gmlib2::parametric::PSubCurveOnSurface<gmlib2::parametric::PLine,
                                             PSubPlaneOnPTorus>;
  gmlib2::testing::isParametricObject<PSubLineOnPSubPlaneOnPTorus>();
}
