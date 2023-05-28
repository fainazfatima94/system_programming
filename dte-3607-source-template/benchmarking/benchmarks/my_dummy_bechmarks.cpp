// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <random>


namespace dte3607::benchmarking::predef {

    struct GoldDummyBenchmarkF : benchmark::Fixture {

      using benchmark::Fixture::Fixture;
      ~GoldDummyBenchmarkF() override {}

      void SetUp(benchmark::State const&) final
      {
        int no_ints = 1'000'000;
        my_ints.reserve(no_ints);

        std::random_device rd;
        std::mt19937                    gen(rd());
        std::uniform_int_distribution<> distrib(0, 10);

        for (int n = 0; n < no_ints; ++n) {
          my_ints.emplace_back(distrib(gen));
        }
      }
      void TearDown(benchmark::State const&) override { my_ints.clear(); }

      std::vector<int> my_ints;
    };


    struct SilverDummyBenchmarkF : benchmark::Fixture {

      using benchmark::Fixture::Fixture;
      ~SilverDummyBenchmarkF() override {}

      void SetUp(benchmark::State const&) final
      {
        int no_ints = 1'000'000;
        my_ints.reserve(no_ints);

        std::random_device rd;
        std::mt19937                    gen(rd());
        std::uniform_int_distribution<> distrib(0, 10);

        for (int n = 0; n < no_ints; ++n) {
          my_ints.push_back(std::move(std::make_unique<int>(distrib(gen))));
        }
      }
      void TearDown(benchmark::State const&) override { my_ints.clear(); }

      std::vector<std::unique_ptr<int>> my_ints;
    };

}   // namespace dte3607::benchmarking::predef




// Qualify predefined fixtures
using namespace dte3607::benchmarking::predef;

// Dummy benchmarks
BENCHMARK_DEFINE_F(GoldDummyBenchmarkF, dummy_test)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    auto const ans [[maybe_unused]]
    = std::accumulate(std::begin(my_ints), std::end(my_ints), 0);
}

// Dummy benchmarks
BENCHMARK_DEFINE_F(SilverDummyBenchmarkF, dummy_test)
(benchmark::State& st)
{
  for ([[maybe_unused]]auto const& _ : st)
    auto const ans [[maybe_unused]]
    = std::accumulate(std::begin(my_ints), std::end(my_ints), 0,
                      [](auto const& sub_sum, auto const& ele_ptr) {
                        return sub_sum + *ele_ptr;
                      });
}


BENCHMARK_REGISTER_F(GoldDummyBenchmarkF, dummy_test);

BENCHMARK_REGISTER_F(SilverDummyBenchmarkF, dummy_test);

BENCHMARK_MAIN();
