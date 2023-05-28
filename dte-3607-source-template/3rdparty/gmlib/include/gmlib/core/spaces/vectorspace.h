#ifndef GM2_CORE_SPACES_VECTORSPACE_H
#define GM2_CORE_SPACES_VECTORSPACE_H

#include "eucledianspace.h"


#define GM2_DEFINE_VECTORSPACE_TYPES_0(TRANSITIVE_BASE)                        \
                                                                               \
  /* Space Info */                                                             \
  static constexpr auto Dimension = TRANSITIVE_BASE::Dimension;                \
  static constexpr auto FrameDim  = TRANSITIVE_BASE::FrameDim;                 \
  static constexpr auto VectorDim = TRANSITIVE_BASE::VectorDim;                \
                                                                               \
  /* Types */                                                                  \
  using Vector = typename TRANSITIVE_BASE::Vector;                             \
  using Frame  = typename TRANSITIVE_BASE::Frame;


#define GM2_DEFINE_VECTORSPACE_TYPES(TRANSITIVE_BASE)                          \
                                                                               \
  GM2_DEFINE_VECTORSPACE_TYPES_0(TRANSITIVE_BASE)                              \
                                                                               \
  /* Space Info */                                                             \
  using VectorSpaceType = typename TRANSITIVE_BASE::VectorSpaceType;


#define GM2_DEFINE_VECTORSPACE_TYPES_PREFIX(PREFIX, TRANSITIVE_BASE)           \
                                                                               \
  /* Space Info */                                                             \
  using PREFIX##VectorSpaceType = typename TRANSITIVE_BASE::VectorSpaceType;   \
  static constexpr auto PREFIX##Dimension = TRANSITIVE_BASE::Dimension;        \
  static constexpr auto PREFIX##FrameDim  = TRANSITIVE_BASE::FrameDim;         \
  static constexpr auto PREFIX##VectorDim = TRANSITIVE_BASE::VectorDim;        \
                                                                               \
  /* Types */                                                                  \
  using PREFIX##Vector = typename TRANSITIVE_BASE::Vector;                     \
  using PREFIX##Frame  = typename TRANSITIVE_BASE::Frame;



namespace gm::spaces
{

  template <typename Type_T, size_t Dim_T>
  struct VectorSpace {

    using Type       = Type_T;
    using VectorSpaceType  = VectorSpace<Type, Dim_T>;

    // Dims
    constexpr static auto Dimension = Dim_T;
    constexpr static auto FrameDim  = Dim_T;
    constexpr static auto VectorDim = Dim_T;

    // VectorSpace
    using Vector = VectorT<Type, VectorDim>;
    using Frame  = MatrixT<Type, VectorDim, FrameDim>;
  };

  namespace vectorspace
  {

    template <typename VectorSpace_T>
    const typename VectorSpace_T::Frame identityFrame()
    {
      using Type = typename VectorSpace_T::Type;
      auto FrameDim    = VectorSpace_T::FrameDim;
      auto VectorDim   = VectorSpace_T::VectorDim;
      auto MinDim      = std::min(VectorDim, FrameDim);

      typename VectorSpace_T::Frame I(0);
      for (size_t i = 0; i < MinDim; ++i) I(i, i) = Type(1);
      //      blaze::diagonal(I) = VectorT<Unit, MinDim>(1); // Next blaze
      return I;
    }

  }   // namespace vectorspace

}   // namespace gm::spaces



#endif   // GM2_CORE_SPACES_VECTORSPACE_H
