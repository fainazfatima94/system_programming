#ifndef GM2_CORE_GBC_MAPPING_H
#define GM2_CORE_GBC_MAPPING_H

#include "../gm2_blaze.h"

namespace gm::gbc
{

  template <typename Unit_Type_T>
  inline auto gbcToSMapping(const DVectorT<Unit_Type_T>& lambda)
  {
    const auto N    = lambda.size();
    auto rotate_idx = [N](int k) { return size_t((int(N) + k) % int(N)); };

    [[maybe_unused]] const auto im2 = rotate_idx(-2);
    const auto                  im  = rotate_idx(-1);
    const auto                  i   = 0;
    [[maybe_unused]] const auto ip  = rotate_idx(1);

    [[maybe_unused]] const auto lim2 = lambda[im2];
    const auto                  lim  = lambda[im];
    const auto                  li   = lambda[i];
    [[maybe_unused]] const auto lip  = lambda[ip];
    const auto                  sum  = lim + li;

    Unit_Type_T eps{1e-2};
    // if (std::abs(lim) < eps && std::abs(li) < eps && std::abs(lip) < eps) {
    if (std::abs(sum) < eps) {
#if 0
        // Use the identity s_i = 1 - h_{i+1}
        const auto h_ip = Unit_Type_T(1) - li - lip;
        const auto ret  = Unit_Type_T(1) - h_ip;
#else
      // Use the identity s_i = h_{i-1}
      const auto h_im = Unit_Type_T(1) - lim2 - lim;
      const auto ret  = h_im;
#endif
      //          if (ret > PSpaceUnit(1))
      //            return PSpaceUnit(1);
      //          if (ret < PSpaceUnit(0))
      //            return PSpaceUnit(0);
      return ret;
    }

    const auto l = lambda[i] / sum;

    //        if (l > PSpaceUnit(1)){
    //          std::cout << "l > 1: " << l << "\t  lambda: " << lambda <<
    //          std::endl; return PSpaceUnit(1);
    //        }
    //        if (l < PSpaceUnit(0)) {
    //          std::cout << "l < 1: " << l << "\t  lambda: " << lambda <<
    //          std::endl; return PSpaceUnit(0);
    //        }

    return Unit_Type_T(l);
  }



  template <typename Unit_Type_T>
  inline auto gbcToHMapping(const DVectorT<Unit_Type_T>& lambda)
  {
    // Polygon sides
    const auto N = lambda.size();

    // Rotate polygon index
    auto rotate_idx = [N](int k) { return size_t((int(N) + k) % int(N)); };

    // Indices
    const auto im = rotate_idx(-1);
    const auto i  = rotate_idx(0);

    // compute distance-coordinate (h or v)
    const auto h = Unit_Type_T(1) - lambda[im] - lambda[i];

    //    std::cout << " gbcToHMapping:";
    //    std::cout << ", lambda: ";
    //    for( const auto& l : lambda ) std::cout << l << " ";
    //    std::cout << ", idxm: " << im << ", idx: " << i;
    //    std::cout << ", h: " << h;
    //    std::cout << std::endl;

    return h;
  }

}   // namespace gm::gbc


#endif   // GM2_CORE_GBC_MAPPING_H
