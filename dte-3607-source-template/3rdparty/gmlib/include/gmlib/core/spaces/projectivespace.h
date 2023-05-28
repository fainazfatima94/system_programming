#ifndef GM2_CORE_SPACES_PROJECTIVESPACE_H
#define GM2_CORE_SPACES_PROJECTIVESPACE_H


#include "affinespace.h"



#define GM2_DEFINE_PROJECTIVESPACE_TYPES_0(TRANSITIVE_BASE)                    \
                                                                               \
  /* Affine Space */                                                           \
  GM2_DEFINE_AFFINESPACE_TYPES(TRANSITIVE_BASE)                                \
                                                                               \
  /* Space Info */                                                             \
  static constexpr auto VectorHDim = TRANSITIVE_BASE ::VectorHDim;             \
                                                                               \
  /* Types */                                                                  \
  using PointH   = typename TRANSITIVE_BASE ::PointH;                          \
  using VectorH  = typename TRANSITIVE_BASE ::VectorH;                         \
  using ASFrameH = typename TRANSITIVE_BASE ::ASFrameH;


#define GM2_DEFINE_PROJECTIVESPACE_TYPES(TRANSITIVE_BASE)                      \
                                                                               \
  GM2_DEFINE_PROJECTIVESPACE_TYPES_0(TRANSITIVE_BASE)                          \
                                                                               \
  /* Space Info */                                                             \
  using ProjectiveSpaceType = typename TRANSITIVE_BASE::ProjectiveSpaceType;


#define GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(PREFIX, TRANSITIVE_BASE)       \
                                                                               \
  /* Affine Space */                                                           \
  GM2_DEFINE_AFFINESPACE_TYPES_PREFIX(PREFIX, TRANSITIVE_BASE)                 \
                                                                               \
  /* Space Info */                                                             \
  using PREFIX##ProjectiveSpaceType =                                          \
    typename TRANSITIVE_BASE::ProjectiveSpaceType;                             \
  static constexpr auto PREFIX##VectorHDim = TRANSITIVE_BASE ::VectorHDim;     \
                                                                               \
  /* Types */                                                                  \
  using PREFIX##PointH   = typename TRANSITIVE_BASE ::PointH;                  \
  using PREFIX##VectorH  = typename TRANSITIVE_BASE ::VectorH;                 \
  using PREFIX##ASFrameH = typename TRANSITIVE_BASE ::ASFrameH;


namespace gm::spaces
{



  template <typename Type_T, size_t Dim_T>
  struct ProjectiveSpace : AffineSpace<Type_T, Dim_T> {

    using Type          = Type_T;
    using ProjectiveSpaceType = ProjectiveSpace<Type_T, Dim_T>;

    // Transfinite types
    using AffineSpaceType = AffineSpace<Type_T, Dim_T>;
    GM2_DEFINE_AFFINESPACE_TYPES_0(AffineSpaceType)

    // Projective space
    static constexpr auto VectorHDim = VectorDim + 1;
    using PointH                     = VectorT<Type, VectorHDim>;
    using VectorH                    = VectorT<Type, VectorHDim>;
    using ASFrameH                   = MatrixT<Type, VectorHDim, ASFrameDim>;
  };



  namespace projectivespace
  {

    template <typename ProjectiveSpace_T>
    const typename ProjectiveSpace_T::ASFrameH identityFrame()
    {
      using Type = typename ProjectiveSpace_T::Type;
      auto ASFrameDim  = ProjectiveSpace_T::ASFrameDim;
      auto VectorHDim  = ProjectiveSpace_T::VectorHDim;

      typename ProjectiveSpace_T::ASFrameH I(Type(0));
      // blaze::submatrix<0UL, 0UL, VectorHDim - 1, ASFrameDim - 1>(I) // next
      // blaze
      blaze::submatrix(I, 0UL, 0UL, VectorHDim - 1, ASFrameDim - 1)
        = vectorspace::identityFrame<
          typename ProjectiveSpace_T::VectorSpaceType>();
      I(VectorHDim - 1, ASFrameDim - 1) = Type(1);
      return I;
    }

    template <typename ProjectiveSpace_T>
    auto aSpaceFrame(const typename ProjectiveSpace_T::ASFrameH& frame)
    {
      return blaze::submatrix<0UL, 0UL, ProjectiveSpace_T::VectorDim,
                              ProjectiveSpace_T::ASFrameDim>(frame);
    }

    template <typename ProjectiveSpace_T>
    auto vSpaceFrame(const typename ProjectiveSpace_T::ASFrameH& frame)
    {
      return blaze::submatrix<0UL, 0UL, ProjectiveSpace_T::VectorDim,
                              ProjectiveSpace_T::FrameDim>(frame);
    }

    template <typename ProjectiveSpace_T>
    auto pSpaceFrameFromDup(const typename ProjectiveSpace_T::Vector& dir,
                            const typename ProjectiveSpace_T::Vector& up,
                            const typename ProjectiveSpace_T::Point&  origin)
    {

      //      if constexpr (ProjectiveSpace_T::VectorDim == 3) {

      using ReturnType = typename ProjectiveSpace_T::ASFrameH;

      auto asframeh = identityFrame<ProjectiveSpace_T>();
      auto asframe  = blaze::submatrix<0UL, 0UL, ProjectiveSpace_T::VectorDim,
                                      ProjectiveSpace_T::ASFrameDim>(asframeh);

      auto fdir  = blaze::column<0UL>(asframe);
      auto fside = blaze::column<1UL>(asframe);
      auto fup   = blaze::column<2UL>(asframe);
      auto fpos  = blaze::column<3UL>(asframe);

      fdir  = blaze::normalize(dir);
      fup   = blaze::normalize(up - (up * dir) * dir);
      fside = blaze::normalize(blaze::cross(fup, fdir));
      fpos  = blaze::evaluate(origin);

      return ReturnType(asframeh);
      //      }
    }


    namespace detail
    {
      template <typename ProjectiveSpace_T, size_t COL_T>
      auto aSpaceFrameColumn(const typename ProjectiveSpace_T::ASFrameH& frame)
      {
        return blaze::column<0UL>(
          blaze::submatrix<0UL, COL_T, ProjectiveSpace_T::VectorDim, 1UL>(
            frame));
      }
    }   // namespace detail

    /*!
     * Returns a frames direction-axis, given a projective frame, with respect
     * to a parent space
     */
    template <typename ProjectiveSpace_T>
    auto directionAxis(const typename ProjectiveSpace_T::ASFrameH& frame)
    {
      return detail::aSpaceFrameColumn<ProjectiveSpace_T, 0UL>(frame);
    }

    template <typename ProjectiveSpace_T>
    auto sideAxis(const typename ProjectiveSpace_T::ASFrameH& frame)
    {
      return detail::aSpaceFrameColumn<ProjectiveSpace_T, 1UL>(frame);
    }

    template <typename ProjectiveSpace_T>
    auto upAxis(const typename ProjectiveSpace_T::ASFrameH& frame)
    {
      return detail::aSpaceFrameColumn<ProjectiveSpace_T, 2UL>(frame);
    }

    template <typename ProjectiveSpace_T>
    auto frameOrigin(const typename ProjectiveSpace_T::ASFrameH& frame)
    {
      return detail::aSpaceFrameColumn<ProjectiveSpace_T, 3UL>(frame);
    }


    /*!
     *  Returns a frames direction-axis, in homogenuous coordinates, with
     * respect to a parent space
     */
    template <typename ProjectiveSpace_T>
    auto directionAxisH(const typename ProjectiveSpace_T::ASFrameH& frame)
    {
      return blaze::column<0UL>(frame);
    }

    template <typename ProjectiveSpace_T>
    auto sideAxisH(const typename ProjectiveSpace_T::ASFrameH& hframe)
    {
      return blaze::column<1UL>(hframe);
    }

    template <typename ProjectiveSpace_T>
    auto upAxisH(const typename ProjectiveSpace_T::ASFrameH& hframe)
    {
      return blaze::column<2UL>(hframe);
    }

    template <typename ProjectiveSpace_T>
    auto frameOriginH(const typename ProjectiveSpace_T::ASFrameH& hframe)
    {
      return blaze::column<3UL>(hframe);
    }



    // clang-format off
    //   -- Concept TS
    //   template <ProjectiveSpaceConcept&&> void move(...) {}
    //   -- C++20
    //   template <typename ProjectiveSpace_T> void move(...) ->
    //     ProjectiveSpaceConcept<ProjectiveSpace_T> {}
    //   -- Type traits
    //   template <typename ProjectiveSpace_T>
    //     enable_if_t<is_projectivespace<ProjectiveSpace_T>,void>
    //     move(...) {}
    // clang-format on
    template <typename ProjectiveSpace_T>
    void translateParent(typename ProjectiveSpace_T::ASFrameH& homogenous_frame,
                         const typename ProjectiveSpace_T::Vector& direction)
    {
      constexpr auto ASFrameDim = ProjectiveSpace_T::ASFrameDim;
      constexpr auto VectorDim  = ProjectiveSpace_T::VectorDim;
      blaze::column(
        blaze::submatrix(homogenous_frame, 0UL, 0UL, VectorDim, ASFrameDim),
        3UL)
        += direction;
    }

    template <typename ProjectiveSpace_T>
    std::enable_if_t<ProjectiveSpace_T::VectorDim == 3, void>
    rotateParent(typename ProjectiveSpace_T::ASFrameH& frame, double ang,
                 const typename ProjectiveSpace_T::Vector& axis)
    {
      const auto rot_frame = algorithms::rotationMatrix(ang, axis);

      const auto dir = projectivespace::directionAxis<ProjectiveSpace_T>(frame);
      const auto side = projectivespace::sideAxis<ProjectiveSpace_T>(frame);
      const auto up   = projectivespace::upAxis<ProjectiveSpace_T>(frame);

      blaze::subvector<0UL, 3UL>(blaze::column<0UL>(frame)) = rot_frame * dir;
      blaze::subvector<0UL, 3UL>(blaze::column<1UL>(frame)) = rot_frame * side;
      blaze::subvector<0UL, 3UL>(blaze::column<2UL>(frame)) = rot_frame * up;
    }

  }   // namespace projectivespace

}   // namespace gm::spaces



#endif   // GM2_CORE_SPACES_PROJECTIVESPACE_H
