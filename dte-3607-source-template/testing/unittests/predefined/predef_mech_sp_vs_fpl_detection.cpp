  #include <physengine/mechanics/sphere_vs_fixed_plane_detection.h>
#include <physengine/bits/types.h>


// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <chrono>


// Safely enable namespaces
using namespace dte3607::physengine;
using namespace std::chrono_literals;





/////////////////
/// \brief TEST
///
///
///





struct Mechanics_Sphere_vs_FixedPlane_CollisionDetection
    : ::testing::Test {

  using ::testing::Test::Test;
  ~Mechanics_Sphere_vs_FixedPlane_CollisionDetection() override {}

  // Start time point
  types::HighResolutionTP m_t_0;

  // The sphere
  types::HighResolutionTP m_sphere_tc;
  types::Point3    m_sphere_p;
  types::ValueType m_sphere_r;
  types::Vector3   m_sphere_v;

  // The plane
  types::Point3  m_fplane_q;
  types::Vector3 m_fplane_n;

  // No gravity
  types::Vector3 m_external_forces;

};



struct Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0
    : Mechanics_Sphere_vs_FixedPlane_CollisionDetection {

  void SetUp() final
  {
    m_t_0       = types::HighResolutionClock::now();
    m_sphere_tc = m_t_0;

    // The sphere
    m_sphere_p = types::Point3{0.0, 10.0, 0};
    m_sphere_r = types::ValueType{1.0};
    m_sphere_v = types::Vector3{10.0, 0, 0};

    // The plane
    m_fplane_q = types::Point3{10., 0, 0};
    m_fplane_n = types::Vector3{-1, 0, 0};

    // No gravity
    m_external_forces = types::Vector3{0, 0, 0};
  }

};





TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0, DT16MS)
{
  // Sim 16 [ms]
  auto const res_16ms = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 16ms);
  EXPECT_FALSE(res_16ms);
}

TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0, DT1S)
{
  // Sim 1 [s]
  auto const res_1s = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 1s);
  EXPECT_TRUE(res_1s);
//  if(res_1s) {
//    auto const res_1s_value = res_1s.value();
//    EXPECT_DOUBLE_EQ(res_1s_value, 0.9);
//  }
}

TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0, DT2S)
{
  // Sim 1 [ms]
  auto const res_2s = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 2s);
  EXPECT_TRUE(res_2s);
}







struct Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0P500ms
    : Mechanics_Sphere_vs_FixedPlane_CollisionDetection {

  void SetUp() final
  {
    m_t_0       = types::HighResolutionClock::now();
    m_sphere_tc = m_t_0 + 500ms;

    // The sphere
    m_sphere_p = types::Point3{0.0, 10.0, 0};
    m_sphere_r = types::ValueType{1.0};
    m_sphere_v = types::Vector3{10.0, 0, 0};

    // The plane
    m_fplane_q = types::Point3{10., 0, 0};
    m_fplane_n = types::Vector3{-1, 0, 0};

    // No gravity
    m_external_forces = types::Vector3{0, 0, 0};
  }

};



TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0P500ms, dt200ms)
{
  // Sim 200 [ms]
  auto const res_200ms = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 200ms);

  EXPECT_FALSE(res_200ms);
}

TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0P500ms, dt1s)
{
  // Sim 1 [s]
  auto const res_1s = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 1s);

  EXPECT_TRUE(res_1s);
//  if(res_1s) {
//    auto const res_1s_value = res_1s.value();
//    EXPECT_DOUBLE_EQ(res_1s_value, 0.9);
//  }
}

TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0P500ms, dt2s)
{
  // Sim 1 [ms]
  auto const res_2s = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 2s);

  EXPECT_TRUE(res_2s);
}







struct Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0P2s
    : Mechanics_Sphere_vs_FixedPlane_CollisionDetection {

  void SetUp() final
  {
    m_t_0       = types::HighResolutionClock::now();
    m_sphere_tc = m_t_0 + 2s;

    // The sphere
    m_sphere_p = types::Point3{0.0, 10.0, 0};
    m_sphere_r = types::ValueType{1.0};
    m_sphere_v = types::Vector3{10.0, 0, 0};

    // The plane
    m_fplane_q = types::Point3{10., 0, 0};
    m_fplane_n = types::Vector3{-1, 0, 0};

    // No gravity
    m_external_forces = types::Vector3{0, 0, 0};
  }

};


TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0P2s, dt200ms)
{

  // Sim 200 [ms]
  auto const res_200ms = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 200ms);

  EXPECT_FALSE(res_200ms);
}

TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0P2s, dt1s)
{
  // Sim 1 [s]
  auto const res_1s = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 1s);

  EXPECT_FALSE(res_1s);
}

TEST_F(Mechanics_Sp_vs_FPl_CD_Basic_SphereAtT0P2s, dt2s)
{
  // Sim 1 [ms]
  auto const res_2s = mechanics::detectCollisionSphereFixedPlane(
    m_sphere_tc, m_sphere_p, m_sphere_r, m_sphere_v, m_fplane_q, m_fplane_n,
    m_external_forces, m_t_0, 2s);

  EXPECT_FALSE(res_2s);
}
