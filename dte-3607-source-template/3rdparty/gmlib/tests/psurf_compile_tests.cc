// gtest
#include <gtest/gtest.h>   // googletest header file

// gmlib2
#include <parametric/classic_objects/plane.h>
#include <parametric/classic_objects/sphere.h>
#include <parametric/classic_objects/torus.h>
using namespace gmlib2::parametric;

// stl
#include <chrono>
using namespace std::chrono_literals;


namespace unittest_detail
{

  template <typename Surface_T>
  void invokeDefaultSurfaceMethods([[maybe_unused]]const Surface_T& surface)
  {
    static_assert(std::is_base_of<Surface<>, Surface_T>::value,
                  "Surface_T must be a base of Surface<>.");

    [[maybe_unused]] auto is_c    = surface.isClosed();
    [[maybe_unused]] auto param_s = surface.startParameters();
    [[maybe_unused]] auto param_e = surface.endParameters();
    [[maybe_unused]] auto local_space_eval_result = surface.evaluateLocal(
      param_s + (param_e - param_s) * 0.5, {1, 1}, {false, false});
    [[maybe_unused]] auto parent_space_eval_result = surface.evaluateParent(
      param_s + (param_e - param_s) * 0.5, {1, 1}, {false, false});
  }

}   // namespace unittest_detail

TEST(PlaneForceCompile, CompileTest)
{
  Plane<> plane;
  unittest_detail::invokeDefaultSurfaceMethods(plane);
  EXPECT_TRUE(true);
}

TEST(SphereForceCompile, CompileTest)
{
  Sphere<> sphere;
  unittest_detail::invokeDefaultSurfaceMethods(sphere);
  EXPECT_TRUE(true);
}

TEST(TorusForceCompile, CompileTest)
{
  Torus<> torus;
  unittest_detail::invokeDefaultSurfaceMethods(torus);
  EXPECT_TRUE(true);
}
