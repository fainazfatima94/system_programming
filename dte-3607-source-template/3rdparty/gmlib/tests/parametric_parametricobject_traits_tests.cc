// gtest
#include <gtest/gtest.h>   // googletest header file


#include "parametricobject_tests.h"

// gmlib2
#include <parametric/pcurve.h>
#include <parametric/psurface.h>
#include <parametric/ppolygonsurface.h>



TEST(Parametric_ParametricObject_Traits, ParameterVSpaceInfo_isSpaceInfo) {

  using ParameterVSpaceInfo = gmlib2::spaces::ParameterVSpaceInfo<2>;
  EXPECT_TRUE(gmlib2::traits::Is_SpaceInfo<ParameterVSpaceInfo>::value);
}

TEST(Parametric_ParametricObject_Traits, PCurve_isParametricObject) {

  SCOPED_TRACE("");
  using PCurve = gmlib2::parametric::PCurve<>;
  gmlib2::testing::isParametricObject<PCurve>();
}

TEST(Parametric_ParametricObject_Traits, PSurface_isParametricObject) {

  SCOPED_TRACE("");
  using PSurface = gmlib2::parametric::PSurface<>;
  gmlib2::testing::isParametricObject<PSurface>();
}


TEST(Parametric_ParametricObject_Traits, PPolygonSurfaceConstruction_isParametricObject) {

  SCOPED_TRACE("");
  using PPolygonSurface = gmlib2::parametric::PPolygonSurface<>;
  gmlib2::testing::isParametricObject<PPolygonSurface>();
}
