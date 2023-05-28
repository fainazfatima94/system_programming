// gtest
#include <gtest/gtest.h>   // googletest header file


#include "parametricobject_tests.h"

// gmlib2
#include <parametric/classic_constructions/pbeziercurve.h>
#include <parametric/classic_constructions/pbeziersurface.h>
#include <parametric/classic_constructions/pbilinearcoonspatch.h>
#include <parametric/classic_constructions/pbicubiccoonspatch.h>
#include <parametric/classic_constructions/pbsplinecurve.h>
#include <parametric/classic_constructions/pbsplinesurface.h>
#include <parametric/classic_constructions/phermitecurve.h>
#include <parametric/classic_constructions/pnurbscurve.h>
#include <parametric/classic_constructions/pnurbssurface.h>



TEST(Parametric_ClassicConstructions_Traits, PBezierCurve_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PBezierCurve<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PBezierSurface_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PBezierSurface<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PBilinearCoonsPatch_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PBilinearCoonsPatch<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PBicubicCoonsPatch_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PBicubicCoonsPatch<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PBSplineCurve_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PBSplineCurve<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PBSplineSurface_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PBSplineSurface<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PHermiteCurveP2V2_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PHermiteCurveP2V2<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PHermiteCurveP3V2_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PHermiteCurveP3V2<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PHermiteCurveP3V3_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PHermiteCurveP3V3<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PNURBSCurve_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PNURBSCurve<>>();
}

TEST(Parametric_ClassicConstructions_Traits, PNURBSSurface_isParametricObject) {

  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PNURBSSurface<>>();
}



