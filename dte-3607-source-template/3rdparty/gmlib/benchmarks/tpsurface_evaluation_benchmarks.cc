// google benchmark
#include <benchmark/benchmark.h>

// gmlib2
#include <parametric/classic_objects/pplane.h>
//#include <core/tpsphere.h>
//#include <core/tptorus.h>


//template <typename TPSurface_T>
//int sampleBenchmarkTPSurface(benchmark::State& state)
//{
//  static_assert(std::is_base_of<gmlib2::TPSurface<>, TPSurface_T>::value,
//                "TPSurface_T must be a base of TPSurface<>.");

//  TPSurface_T tpplane;

//  while (state.KeepRunning()) {
//    auto sample_result
//      = tpplane.sample(size_t(state.range(0)), size_t(state.range(0)), 0, 0);
//    benchmark::DoNotOptimize(sample_result);
//  }
//}
//BENCHMARK_TEMPLATE(sampleBenchmarkTPSurface,gmlib2::TPPlane<>)
//  ->Unit(benchmark::kMillisecond)
//  ->RangeMultiplier(2)
//  ->Range(2, 8 << 7);

static void sampleBenchmarkTPPlane(benchmark::State& state)
{
  using PPlaneType = gmlib2::parametric::PPlane<>;
  using PSpaceSize = PPlaneType::PSpaceSizeArray;
  PPlaneType pplane;

  while (state.KeepRunning()) {
    auto sample_result
      = pplane.sample(PSpaceSize{size_t(state.range(0)),size_t(state.range(0))},PSpaceSize{0UL,0UL});
    benchmark::DoNotOptimize(sample_result);
  }
}
BENCHMARK(sampleBenchmarkTPPlane)
  ->Unit(benchmark::kMillisecond)
  ->RangeMultiplier(2)
  ->Range(2, 8 << 7);


BENCHMARK_MAIN();
