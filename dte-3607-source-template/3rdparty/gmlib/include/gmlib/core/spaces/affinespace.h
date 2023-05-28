#ifndef GM2_CORE_SPACES_AFFINESPACE_H
#define GM2_CORE_SPACES_AFFINESPACE_H


#include "vectorspace.h"




#define GM2_DEFINE_AFFINESPACE_TYPES_0(TRANSITIVE_BASE)                        \
                                                                               \
  /* Vector Space */                                                           \
  GM2_DEFINE_VECTORSPACE_TYPES(TRANSITIVE_BASE)                                \
                                                                               \
  /* Space Info */                                                             \
  static constexpr auto ASFrameDim = TRANSITIVE_BASE::ASFrameDim;              \
                                                                               \
  /* Types */                                                                  \
  using Point   = typename TRANSITIVE_BASE::Point;                             \
  using ASFrame = typename TRANSITIVE_BASE::ASFrame;

#define GM2_DEFINE_AFFINESPACE_TYPES(TRANSITIVE_BASE)                          \
                                                                               \
  GM2_DEFINE_AFFINESPACE_TYPES_0(TRANSITIVE_BASE)                              \
                                                                               \
  /* Space Info */                                                             \
  using AffineSpaceType = typename TRANSITIVE_BASE::AffineSpaceType;

#define GM2_DEFINE_AFFINESPACE_TYPES_PREFIX(PREFIX, TRANSITIVE_BASE)           \
                                                                               \
  /* Vector Space */                                                           \
  GM2_DEFINE_VECTORSPACE_TYPES_PREFIX(PREFIX, TRANSITIVE_BASE)                 \
                                                                               \
  /* Space Info */                                                             \
  using PREFIX##AffineSpaceType = typename TRANSITIVE_BASE::AffineSpaceType;   \
  static constexpr auto PREFIX##ASFrameDim = TRANSITIVE_BASE::ASFrameDim;      \
                                                                               \
  /* Types */                                                                  \
  using PREFIX##Point   = typename TRANSITIVE_BASE::Point;                     \
  using PREFIX##ASFrame = typename TRANSITIVE_BASE::ASFrame;


namespace gm::spaces
{

  template <typename Type_T, size_t Dim_T>
  struct AffineSpace {

    using Type      = Type_T;
    using AffineSpaceType = AffineSpace<Type_T, Dim_T>;

    // Transfinit types
    using VectorSpaceType = VectorSpace<Type_T, Dim_T>;
    GM2_DEFINE_VECTORSPACE_TYPES_0(VectorSpaceType)

    // Affine space
    static constexpr auto ASFrameDim = FrameDim + 1;
    using Point                      = VectorT<Type, VectorDim>;
    using ASFrame                    = MatrixT<Type, VectorDim, ASFrameDim>;
  };






  namespace affinespace
  {



    template <typename AffineSpace_T>
    auto vSpaceFrame(const typename AffineSpace_T::ASFrameH& frame)
    {
      return blaze::submatrix<0UL, 0UL, AffineSpace_T::VectorDim,
                              AffineSpace_T::ASFrameDim>(frame);
    }

  }   // namespace affinespace

}   // namespace gm::spaces


#endif   // GM2_CORE_SPACES_AFFINESPACE_H
