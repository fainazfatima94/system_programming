#include <physengine/solvers/solver_dev_level2.h>
#include <physengine/bits/fixtures.h>
#include <physengine/bits/types.h>


// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>
#include <memory>


// Safely enable namespaces
using namespace dte3607::physengine;
using namespace std::chrono_literals;




/////////////////
/// \brief TEST
///
///

struct SolverDevLevel2Step1_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

    // make sphere
    m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
  }
  void TearDown() final { m_scenario.release(); }
};



TEST_F(SolverDevLevel2Step1_Fixture001, Test001)
{
  solver_dev::level2::solve(*m_scenario, 1s);

  auto no_rbs = m_scenario->noRigidBodies();
  for( auto rid = 1; rid < no_rbs; ++rid )
  {
    // Ask for global frame position of object nr. i
    auto const pos = m_scenario->globalFramePosition(rid);
    EXPECT_LT(pos[0], 10);
  }
}





struct SolverDevLevel2Step1_Fixture002 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane(s)
    // yz+ plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

    // yz- plane
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});

    // make sphere
    m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step1_Fixture002, Test001)
{
  solver_dev::level2::solve(*m_scenario, 1s);

  // Expect to be inbetween the planes
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    for (auto const& p_rid : m_scenario->fixedInfPlaneRBs()) {

      auto const pn = m_scenario->rbPlaneNormal(p_rid);
      auto const pp = m_scenario->globalFramePosition(p_rid);
      auto const sp = m_scenario->globalFramePosition(s_rid);
      auto const sr = m_scenario->rbSphereRadius(s_rid);
      auto const d  = blaze::evaluate(sp - (pp + blaze::normalize(pn)*sr));
      EXPECT_GE(blaze::inner(pn,d),0);
    }
  }
}


struct SolverDevLevel2Step2_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});

    // make sphere
    m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step2_Fixture001, Test001)
{
  solver_dev::level2::solve(*m_scenario, 1s);

  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    auto const sp = m_scenario->globalFramePosition(s_rid);
    EXPECT_GT(sp[0], -10);
    EXPECT_LT(sp[0], 10);
  }
}




struct SolverDevLevel2Step2_Fixture002 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();


    // Set external forces
    m_scenario->setGravity({0,0,0});


    // make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});

    m_scenario->createFixedInfPlane({0, 0, -1}, {0, 0, 10});

    m_scenario->createFixedInfPlane({0, 0, 1}, {0, 0, -10});


    // make sphere
    m_scenario->createSphere(1.0, {100, 0, 100}, {0, 0, 0});
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step2_Fixture002, Test001)
{
  solver_dev::level2::solve(*m_scenario, 1s);

  // Expect to be inbetween the planes
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    for (auto const& p_rid : m_scenario->fixedInfPlaneRBs()) {

      auto const pn = m_scenario->rbPlaneNormal(p_rid);
      auto const pp = m_scenario->globalFramePosition(p_rid);
      auto const sp = m_scenario->globalFramePosition(s_rid);
      auto const sr = m_scenario->rbSphereRadius(s_rid);
      auto const& d = sp - pp;
      auto const pnd = blaze::inner(pn,d) - sr;
      EXPECT_TRUE(pnd > 0);
    }
  }
}



struct SolverDevLevel2Step2_Fixture003 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel2;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();


    // Set external forces
    m_scenario->setGravity({0, 0, 0});


    // make plane
    m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});
    m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});
    m_scenario->createFixedInfPlane({0, 0, -1}, {0, 0, 10});
    m_scenario->createFixedInfPlane({0, 0, 1}, {0, 0, -10});
    m_scenario->createFixedInfPlane({0, -1, 0}, {0, 10, 0});
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, -10, 0});


    // Set up spheres
    auto const p = types::Vector3{-2, -2, -2};

    // Randomize zelection of normal
    auto const v_min = types::Vector3{-10, 5, -20};
    auto const v_max = types::Vector3{ 20, 10, 10};

    std::random_device                     r;
    std::default_random_engine             g(r());
    std::uniform_real_distribution<types::ValueType> x_dist(v_min[0], v_max[0]);
    std::uniform_real_distribution<types::ValueType> y_dist(v_min[1], v_max[1]);
    std::uniform_real_distribution<types::ValueType> z_dist(v_min[2], v_max[2]);

    std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;
    sphere_data.reserve(3 * 3 * 3);
    for (auto x = 0; x < 3; ++x) {
      for (auto y = 0; y < 3; ++y) {
        for (auto z = 0; z < 3; ++z) {

          auto const v = types::Vector3{x_dist(g), y_dist(g), z_dist(g)};
          sphere_data.emplace_back(
            v, p + types::Vector3{2. * x, 2. * y, 2. * z});
        }
      }
    }

    for (auto const& sd : sphere_data)
      m_scenario->createSphere(1.0, std::get<0>(sd), std::get<1>(sd));
  }
  void TearDown() final { m_scenario.release(); }
};


TEST_F(SolverDevLevel2Step2_Fixture003, Test001)
{
  solver_dev::level2::solve(*m_scenario, 1s);

  // Expect to be inbetween the planes
  for (auto const& s_rid : m_scenario->nonFixedSphereRBs()) {
    for (auto const& p_rid : m_scenario->fixedInfPlaneRBs()) {

      auto const pn = m_scenario->rbPlaneNormal(p_rid);
      auto const pp = m_scenario->globalFramePosition(p_rid);
      auto const sp = m_scenario->globalFramePosition(s_rid);
      auto const sr = m_scenario->rbSphereRadius(s_rid);
      auto const& d = sp - pp;
      auto const pnd = blaze::inner(pn,d) - sr;
      EXPECT_GT(pnd, 0);
    }
  }
}



//struct SolverDevLevel2Step3_TestFixture : ::testing::Test {

//  using ::testing::Test::Test;
//  ~SolverDevLevel2Step3_TestFixture() override {}

//  void SetUp() final
//  {
////    m_scenario_fixture = ...;
//  }
//  void TearDown() final {}

//  fixtures::FixtureLevel2 m_scenario;
//};



//TEST_F(SolverDevLevel2Step3_TestFixture, Test001)
//{
//  solver_dev::level2::solve(m_scenario, 16ms);

//  EXPECT_TRUE(false);
//}

