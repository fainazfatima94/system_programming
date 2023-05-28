#include <physengine/mechanics/sphere_vs_sphere_detection.h>
#include <physengine/bits/types.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <chrono>


// Safely enable namespaces
using namespace std::chrono_literals;
using namespace dte3607::physengine;

/////////////////
/// \brief TEST
///
///





struct Mechanics_Sphere_vs_Sphere_CollisionDetection
    : ::testing::Test {

  using ::testing::Test::Test;
  ~Mechanics_Sphere_vs_Sphere_CollisionDetection() override {}

  // Start time point
  types::HighResolutionTP m_t_0;

  // The sphere
  types::HighResolutionTP m_sphere_tc;
  types::Point3    m_sphere_p;
  types::ValueType m_sphere_r;
  types::Vector3   m_sphere_v;

  // The sphere
  types::HighResolutionTP m_other_tc;
  types::Point3    m_other_p;
  types::ValueType m_other_r;
  types::Vector3   m_other_v;

  // No gravity
  types::Vector3 m_external_forces;
};





struct Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0
    : Mechanics_Sphere_vs_Sphere_CollisionDetection {

  void SetUp() final
  {
    m_t_0       = types::HighResolutionClock::now();
    m_sphere_tc = m_t_0;
    m_other_tc  = m_t_0;

    // Sphere
    m_sphere_p = types::Point3{-5.0, 0.0, 0.0};
    m_sphere_r = types::ValueType{1.0};
    m_sphere_v = types::Vector3{10.0, 0.0, 0.0};

    // Other
    m_other_p = types::Point3{5.0, 0.0, 0.0};
    m_other_r = types::ValueType{1.0};
    m_other_v = types::Vector3{-10.0, 0.0, 0.0};

    // No gravity
    m_external_forces = types::Vector3{0, 0, 0};
  }

};



TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0, DT0ms)
{

  // Sim 0ms
  auto const res_0ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 0ms);

  EXPECT_FALSE(res_0ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0, DT200ms)
{
  // Sim 200ms
  auto const res_200ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 200ms);

  EXPECT_FALSE(res_200ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0, DT399ms)
{
  // Sim 399ms
  auto const res_399ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 399ms);

  EXPECT_FALSE(res_399ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0, DT400ms)
{
  // Sim 400ms
  auto const res_400ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 400ms);

  EXPECT_TRUE(res_400ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0, DT1s)
{
  // Sim 1s
  auto const res_1s = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 1s);

  EXPECT_TRUE(res_1s);
}







struct Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P200ms
    : Mechanics_Sphere_vs_Sphere_CollisionDetection {

  void SetUp() final
  {
    m_t_0       = types::HighResolutionClock::now();
    m_sphere_tc = m_t_0 + 200ms;
    m_other_tc  = m_t_0 + 200ms;

    // Sphere
    m_sphere_p = types::Point3{-5.0, 0.0, 0.0};
    m_sphere_r = types::ValueType{1.0};
    m_sphere_v = types::Vector3{10.0, 0.0, 0.0};

    // Other
    m_other_p = types::Point3{5.0, 0.0, 0.0};
    m_other_r = types::ValueType{1.0};
    m_other_v = types::Vector3{-10.0, 0.0, 0.0};

    // No gravity
    m_external_forces = types::Vector3{0, 0, 0};
  }

};



TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P200ms, DT0ms)
{
  // Sim 0ms
  auto const res_0ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 0ms);

  EXPECT_FALSE(res_0ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P200ms, DT16ms)
{
  // Sim 16ms
  auto const res_16ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 16ms);

  EXPECT_FALSE(res_16ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P200ms, DT599ms)
{
  // Sim 599ms
  auto const res_599ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 599ms);

  EXPECT_FALSE(res_599ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P200ms, DT600ms)
{
  // Sim 600ms
  auto const res_600ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 600ms);

  EXPECT_TRUE(res_600ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P200ms, DT1s)
{
  // Sim 1s
  auto const res_1s = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 1s);

  EXPECT_TRUE(res_1s);
}







struct Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P1s
    : Mechanics_Sphere_vs_Sphere_CollisionDetection {

  void SetUp() final
  {
    m_t_0       = types::HighResolutionClock::now();
    m_sphere_tc = m_t_0 + 1s;
    m_other_tc  = m_t_0 + 1s;

    // Sphere
    m_sphere_p = types::Point3{-5.0, 0.0, 0.0};
    m_sphere_r = types::ValueType{1.0};
    m_sphere_v = types::Vector3{10.0, 0.0, 0.0};

    // Other
    m_other_p = types::Point3{5.0, 0.0, 0.0};
    m_other_r = types::ValueType{1.0};
    m_other_v = types::Vector3{-10.0, 0.0, 0.0};

    // No gravity
    m_external_forces = types::Vector3{0, 0, 0};
  }

};


TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P1s, DT0ms)
{
  // Sim 0ms
  auto const res_0ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 0ms);

  EXPECT_FALSE(res_0ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P1s, DT16ms)
{
  // Sim 16ms
  auto const res_16ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 16ms);

  EXPECT_FALSE(res_16ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P1s, DT599ms)
{
  // Sim 599ms
  auto const res_599ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 599ms);

  EXPECT_FALSE(res_599ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P1s, DT600ms)
{
  // Sim 600ms
  auto const res_600ms = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 600ms);

  EXPECT_FALSE(res_600ms);
}

TEST_F(Mechanics_Sp_vs_Sp_CD_Basic_SphereAtT0P1s, DT1s)
{
  // Sim 1s
  auto const res_1s = mechanics::detectCollisionSphereSphere(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_other_tc, m_other_p,
    m_other_r, m_other_v, m_external_forces, m_t_0, 1s);

  EXPECT_FALSE(res_1s);
}
