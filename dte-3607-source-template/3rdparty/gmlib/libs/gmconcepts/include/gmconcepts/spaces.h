#ifndef GMC_SPACES_H
#define GMC_SPACES_H


#include "numbers.h"

namespace gmc::spaces
{

  template <typename Space_T>
  concept Space = requires
  {
    // Dimension
    requires std::unsigned_integral<decltype(Space_T::Dimension)>;

    // Type
    typename Space_T::Type;
  };

  template <typename Space_T>
  concept OneDimensionalSpace = Space<Space_T>and requires
  {
    // Dimension
    requires Space_T::Dimension == 1ul;
  };

  template <typename Space_T>
  concept TwoDimensionalSpace = Space<Space_T>and requires
  {
    // Dimension
    requires Space_T::Dimension == 2ul;
  };

  template <typename Space_T>
  concept ThreeDimensionalSpace = Space<Space_T>and requires
  {
    // Dimension
    requires Space_T::Dimension == 3ul;
  };

  template <typename Space_T>
  concept FourDimensionalSpace = Space<Space_T>and requires
  {
    // Dimension
    requires Space_T::Dimension == 4ul;
  };


  template <typename Space_T>
  concept VectorSpace = Space<Space_T>and requires(typename Space_T::Type   s,
                                                   typename Space_T::Vector v)
  {
    // V + V -> V
    { v + v } -> std::convertible_to<typename Space_T::Vector>;

    // s * V -> V
    { s * v } -> std::convertible_to<typename Space_T::Vector>;
  };

  template <typename Space_T>
  concept AffineSpace
    = VectorSpace<Space_T>and requires(typename Space_T::Point  p,
                                       typename Space_T::Vector v)
  {
    // P + V -> P
    { p + v } -> std::convertible_to<typename Space_T::Point>;
  };


  // Eucledian space
  template <typename Space_T>
  concept EucledianSpace = AffineSpace<Space_T>and
    numbers::Z<decltype(Space_T::Dimension)>and requires
  {
    typename Space_T::ASFrame;
  };


  template <typename Space_T>
  concept R = EucledianSpace<Space_T>and OneDimensionalSpace<Space_T>;

  template <typename Space_T>
  concept R2 = EucledianSpace<Space_T>and TwoDimensionalSpace<Space_T>;

  template <typename Space_T>
  concept R3 = EucledianSpace<Space_T>and ThreeDimensionalSpace<Space_T>;


  // Projective space
  template <typename Space_T>
  concept ProjectiveSpace
    = EucledianSpace<Space_T>and requires(typename Space_T::PointH   p,
                                          typename Space_T::VectorH  v,
                                          typename Space_T::ASFrameH m)
  {
    // M * P -> P
    { m * p } ->std::convertible_to<typename Space_T::PointH>;

    // M * V -> V
    { m * v } ->std::convertible_to<typename Space_T::VectorH>;
  };

  template <typename Space_T>
  concept P = ProjectiveSpace<Space_T>and OneDimensionalSpace<Space_T>;

  template <typename Space_T>
  concept P2 = ProjectiveSpace<Space_T>and TwoDimensionalSpace<Space_T>;

  template <typename Space_T>
  concept P3 = ProjectiveSpace<Space_T>and ThreeDimensionalSpace<Space_T>;

}   // namespace gmc::spaces



#endif   // GMC_SPACES_H
