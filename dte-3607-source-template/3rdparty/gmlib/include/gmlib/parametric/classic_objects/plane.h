#ifndef GM2_PARAMETRIC_SURFACE_PLANE_H
#define GM2_PARAMETRIC_SURFACE_PLANE_H


#include "../tensorproductsurface.h"

#include "../../core/coreutils.h"

namespace gm::parametric
{

  namespace plane
  {

    template <typename PointH, typename VectorH, typename Type_T>
    PointH evaluateAt(Type_T const& u, Type_T const& v, PointH const& p,
                      VectorH const& U, VectorH const& V)
    {
      return p + u * U + v * V;
    }

    template <typename VectorH>
    VectorH derivativeAt(size_t order_u, size_t order_v, VectorH const& u,
                         VectorH const& v)
    {
      assert(order_u > 0 or order_v > 0);

      if (order_u == 1 and order_v == 0)
        return u;
      else if (order_u == 0 and order_v == 1)
        return v;
      else
        return VectorH(0);
    }

  }   // namespace plane



  template <gmc::spaces::ProjectiveSpace EmbedSpace_T
            = spaces::ProjectiveSpace<double, 3ul>>
  struct Plane : TensorProductSurface<EmbedSpace_T> {

    /***
     * Boiler plate */

    /* Base */
    using Base             = TensorProductSurface<EmbedSpace_T>;
    using EmbedSpaceObject = typename Base::EmbedSpaceObject;
    using EmbedSpace       = typename Base::EmbedSpace;
    using Type             = typename Base::Type;


    /* Domain space */
    using DomainSpace      = typename Base::DomainSpace;
    using Domain           = typename Base::Domain;
    using DomainInfo       = typename Base::DomainInfo;
    using Parameterization = typename Base::Parameterization;
    GM2_MAPPING_KERNEL_BOILER_PLATE

    /* Interval Meta */
    using TraceDirection = typename Parameterization::TraceDirection;
    using DTraceDirArray = typename Parameterization::TraceDirArray;

    /*** Boiler plate END */




    // Constructor(s)
  private:
    struct tag_internal {
    };

    template <typename... Ts>
    explicit Plane(tag_internal, Point const& origin, Vector const& u_vector,
                   Vector const& v_vector, Ts&&... ts)
      : Base(std::forward<Ts>(ts)...),
        m_pt{utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
          origin, Type(1))},
        m_u{utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
          u_vector, Type(0))},
        m_v{utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
          v_vector, Type(0))}
    {
    }

  public:
    template <typename... Ts>
    requires gmc::spaces::R2<EmbedSpace> explicit Plane(
      Point const&  origin   = Point{0.0, 0.0},
      Vector const& u_vector = Vector{1.0, 0.0},
      Vector const& v_vector = Vector{0.0, 1.0}, Ts&&... ts)
      : Plane(tag_internal{}, origin, u_vector, v_vector,
              std::forward<Ts>(ts)...)
    {
    }

    template <typename... Ts>
    requires gmc::spaces::R3<EmbedSpace> explicit Plane(
      Point const&  origin   = Point{0.0, 0.0, 0.0},
      Vector const& u_vector = Vector{1.0, 0.0, 0.0},
      Vector const& v_vector = Vector{0.0, 1.0, 0.0}, Ts&&... ts)
      : Plane(tag_internal{}, origin, u_vector, v_vector,
              std::forward<Ts>(ts)...)
    {
    }


    // Members
    VectorH m_pt;
    VectorH m_u;
    VectorH m_v;


    // Methods
    DomainInfo const& domain() const override
    {
      static const DomainInfo d{
        .Uq = Parameterization::TensorAxisTopology::Qualification::Loop,
        .Vq = Parameterization::TensorAxisTopology::Qualification::Loop,
        .U  = {{Type(0), Type(1)}},
        .V  = {{Type(0), Type(1)}}};
      return d;
    }


    PointH evaluateAt(DPoint const& uv, DTraceDirArray const& /*trace_dir*/ = {
                                          TraceDirection::LeftToRight,
                                          TraceDirection::LeftToRight,
                                        }) const override
    {
      return plane::evaluateAt<PointH>(uv[0], uv[1], m_pt, m_u, m_v);
    }

    std::optional<VectorH>
    partialDerivativeAt(DSizeArray const& order, DPoint const& /*uv*/,
                        DTraceDirArray const& /*trace_dir*/ = {
                          TraceDirection::LeftToRight,
                          TraceDirection::LeftToRight,
                        }) const override
    {
      return plane::derivativeAt(order[0], order[1], m_u, m_v);
    }
  };


}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_SURFACE_PLANE_H
