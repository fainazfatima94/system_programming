#ifndef GM2_SPACEOBJECT_H
#define GM2_SPACEOBJECT_H

// gm
#include "spaces/projectivespace.h"

// gmconcepts
#include <gmconcepts/manifolds.h>

// stl
#include <cassert>
#include <iostream>



namespace gm
{

  template <typename Space_T = spaces::ProjectiveSpace<double, 3ul>>
  struct SpaceObject;

  template <typename Type_T, size_t Dimension_T>
  struct SpaceObject<spaces::ProjectiveSpace<Type_T, Dimension_T>> {

    // Embedded (projective) space types
    using Space = spaces::ProjectiveSpace<double, Dimension_T>;
    using Type  = typename Space::Type;
    GM2_DEFINE_PROJECTIVESPACE_TYPES(Space)

    // Constructors
    SpaceObject() {}
    //    virtual ~SpaceObject() = default;

    static void visitAndPrint(std::string const& tag) {

      std::cout << "SpaceObject: " << tag << '\n';
      std::cout << "  Dimension_T: " << Dimension_T << '\n';
      std::cout << "  Dimension: " << Dimension << '\n';
      std::cout << "  FrameDim: " << FrameDim << '\n';
      std::cout << "  VectorDim: " << VectorDim << '\n';
      std::cout << "  ASFrameDim: " << ASFrameDim << '\n';
      std::cout << "  VectorHDim: " << VectorHDim << '\n';
      std::cout << std::endl;
    }

    //! Return the SpaceObject's projective-space frame;
    //! with respect to parent coordinates
    ASFrameH pSpaceFrameParent() const { return m_frame; }

    //! Return the SpaceObject's affine-space frame;
    //! with respect to parent coordinates
    ASFrame aSpaceFrameParent() const
    {
      return spaces::projectivespace::aSpaceFrame<Space>(
        pSpaceFrameParent());
    }

    //! Return the SpaceObject's vector-space frame;
    //! with respect to parent coordinates
    Frame vSpaceFrameParent() const
    {
      return spaces::projectivespace::vSpaceFrame<Space>(
        pSpaceFrameParent());
    }


    //! Return the SpaceObject frame's direction-axis;
    //!  with respect to parent space
    const Vector directionAxisParent() const
    {
      return spaces::projectivespace::directionAxis<Space>(
        pSpaceFrameParent());
    }

    //! Return the SpaceObject frame's side-axis;
    //! with respect to parent space
    const Vector sideAxisParent() const
    {
      return spaces::projectivespace::sideAxis<Space>(pSpaceFrameParent());
    }

    //! Return the SpaceObject frame's up-axis;
    //! with respect to parent space
    const Vector upAxisParent() const
    {
      return spaces::projectivespace::upAxis<Space>(pSpaceFrameParent());
    }

    //! Return the SpaceObject frame's origin;
    //! with respect to parent space
    const Point frameOriginParent() const
    {
      return spaces::projectivespace::frameOrigin<Space>(
        pSpaceFrameParent());
    }


    //! Return the SpaceObject frame's direction-axis
    //! in homogeneous coordinates;
    //! with respect to parent space
    const VectorH directionAxisParentH() const
    {
      return spaces::projectivespace::directionAxisH<Space>(
        pSpaceFrameParent());
    }

    //! Return the SpaceObject frame's side-axis in homogeneous coordinates;
    //! with respect to parent space
    const VectorH sideAxisParentH() const
    {
      return spaces::projectivespace::sideAxisH<Space>(
        pSpaceFrameParent());
    }

    //! Return the SpaceObject frame's up-axis in homogeneous coordinates;
    //! with respect to parent space
    const VectorH upAxisParentH() const
    {
      return spaces::projectivespace::upAxisH<Space>(pSpaceFrameParent());
    }

    //! Return the SpaceObject frame's origin in homogeneous coordinates;
    //! with respect to parent space
    const PointH frameOriginParentH() const
    {
      return spaces::projectivespace::frameOriginH<Space>(pSpaceFrameParent());
    }


    // clang-format off
    //! Apply a translation "locally" to the space object;
    //! before the current transformation stack
    void translateLocal(const Vector& v) {
      translateParent(vSpaceFrameParent() * v);
    }

    //! Apply a rotation "locally" to the space object;
    //! before the current transformation stack
    void rotateLocal(double ang, const Vector& axis) {
      rotateParent(ang, vSpaceFrameParent() * axis);
    }

    //! Apply a translation to the space object;
    //! with respect to a parent frame
    void translateParent(const Vector& v) {
      spaces::projectivespace::translateParent<Space>(m_frame, v);
    }


    //! Apply a rotation to the space object;
    //! with respect to a parent frame
    void rotateParent(double ang, const Vector& axis) {
      if constexpr (VectorDim == 3) {
        spaces::projectivespace::rotateParent<Space>(m_frame, ang, axis);
      }
    }


    //    //! Set the objects frame; in referrece of a parent space
    //    //! Right-hand coordinate system
    //    //! Orthogonal frame: [[dir,side,up],origin]
    //    void setFrameParent(const Vector& direction_axis,
    //                        const Vector& up_axis,
    //                        const Point&  frame_origin) {
    //      if constexpr (VectorDim == 3UL) {

    //        m_frame = spaces::projectivespace::pSpaceFrameFromDup<EmbedSpace>(dir,
    //        up,
    //                                                                          origin);


    ////        // subview(s)
    ////        auto asframe = blaze::submatrix<0UL, 0UL, VectorDim,
    ///   ASFrameDim>(m_frame); /      auto fdir    = blaze::column<0UL>(asframe); /
    ///   auto fside   = blaze::column<1UL>(asframe); /      auto fup     =
    ///   blaze::column<2UL>(asframe); /      auto fpos    =
    ///   blaze::column<3UL>(asframe);

    ////        // values
    ////        fdir  = blaze::normalize(dir);
    ////        fup   = blaze::normalize(up - (up * dir) * dir);
    ////        fside = blaze::normalize(blaze::cross(fup, fdir));
    ////        fpos  = blaze::evaluate(origin);
    //      }
    //    }
    // clang-format on


    // Members
//  private:
    /*!
     * Right-hand-system: [dir,side,up,pos]
     * --
     * the projective space object's frame in the projective space
     */
    ASFrameH m_frame{spaces::projectivespace::identityFrame<Space>()};
  };


}   // namespace gm

#endif   // GM2_SPACEOBJECT_H
