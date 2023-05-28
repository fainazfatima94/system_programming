// gtest
#include <gtest/gtest.h>   // googletest header file


#include "parametricobject_tests.h"

// gmlib2
#include <parametric/classic_objects/pcircle.h>
#include <parametric/classic_objects/pline.h>
#include <parametric/classic_objects/pplane.h>
#include <parametric/classic_objects/psphere.h>
#include <parametric/classic_objects/ptorus.h>



TEST(Parametric_ClassicShapes_Traits, PCircle_isParametricObject)
{
  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PCircle<>>();
}

TEST(Parametric_ClassicShapes_Traits, PLine_isParametricObject)
{
  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PLine<>>();
}

TEST(Parametric_ClassicShapes_Traits, PPlane_isParametricObject)
{
  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PPlane<>>();
}

TEST(Parametric_ClassicShapes_Traits, PSphere_isParametricObject)
{
  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PSphere<>>();
}

TEST(Parametric_ClassicShapes_Traits, PTorus_isParametricObject)
{
  SCOPED_TRACE("");
  gmlib2::testing::isParametricObject<gmlib2::parametric::PTorus<>>();
}
