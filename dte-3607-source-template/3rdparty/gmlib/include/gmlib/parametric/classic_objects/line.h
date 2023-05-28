#ifndef GM_PARAMETRIC_CURVE_LINE_H
#define GM_PARAMETRIC_CURVE_LINE_H



#include "../curve.h"
#include "../parameterizations.h"
#include "../utils/numeric_differentiation.h"

#include "../../core/coreutils.h"

namespace gm::parametric
{

  namespace line
  {

    template <typename PointH, typename VectorH, typename Type_T>
    PointH evaluateAt(Type_T const& t, PointH const& p, VectorH const& v)
    {
      return p + t * v;
    }

    template <typename VectorH>
    VectorH derivativeAt(VectorH const& v)
    {
      return v;
    }

  }   // namespace line





  template <gmc::spaces::ProjectiveSpace EmbedSpace_T
            = spaces::ProjectiveSpace<double, 3ul>>
  struct Line : Curve<EmbedSpace_T> {

    /*** Boiler plate */

    /* Base */
    using Base             = Curve<EmbedSpace_T>;
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

    /*** Boiler plate END */





  // Constructor(s)
  private:
    struct tag_internal {
    };

    template <typename... Ts>
    explicit Line(tag_internal, Point const& origin, Vector const& direction,
                  Ts&&... ts)
      : Base(std::forward<Ts>(ts)...),
        m_pt(utils::extendStaticContainer<Point, VectorH, VectorDim, 1UL>(
          origin, Type(1))),
        m_v(utils::extendStaticContainer<Vector, VectorH, VectorDim, 1UL>(
          direction, Type(0)))
    {
    }

  public:
    template <typename... Ts>
    requires gmc::spaces::R2<EmbedSpace> explicit Line(Point const& origin
                                                       = Point{0.0},
                                                       Vector const& direction
                                                       = Vector{1.0, 0.0},
                                                       Ts&&... ts)
      : Line(tag_internal{}, origin, direction, std::forward<Ts>(ts)...)
    {
    }

    template <typename... Ts>
    requires gmc::spaces::R3<EmbedSpace> explicit Line(Point const& origin
                                                       = Point{0.0},
                                                       Vector const& direction
                                                       = Vector{1.0, 0.0, 0.0},
                                                       Ts&&... ts)
      : Line(tag_internal{}, origin, direction, std::forward<Ts>(ts)...)
    {
    }


    // Members
    VectorH m_pt;
    VectorH m_v;


    // Methods
    DomainInfo const& domain() const override
    {
      using Interval = typename DomainInfo::Interval;
      static const DomainInfo d{.Iq = DomainInfo::Qualification::Closed,
                                .I  = Interval{{0, 1}}};
      return d;
    }

    PointH evaluateAt(
      Type const& t,
      TraceDirection /*trace_dir*/ = TraceDirection::LeftToRight) const override
    {
      return line::evaluateAt(t, m_pt, m_v);
    }

    std::optional<VectorH> derivativeAt(
      size_t order, Type const& /*t*/,
      TraceDirection /*trace_dir*/ = TraceDirection::LeftToRight) const override
    {
      if (order == 1ul) return line::derivativeAt(m_v);
      return {VectorH(0)};
    }
  };

}   // namespace gm::parametric



#endif   // GM_PARAMETRIC_CURVE_LINE_H
