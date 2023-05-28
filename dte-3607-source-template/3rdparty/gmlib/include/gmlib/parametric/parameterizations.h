#ifndef GM2_PARAMETERIZATIONS_H
#define GM2_PARAMETERIZATIONS_H

#include "../core/spaces/projectivespace.h"

// gmconcepts
#include <gmconcepts/spaces.h>

// stl
#include <vector>


namespace gm::parametric::domain
{

  namespace topology
  {
    template <typename Type_T>
    struct IntervalMeta {
      using Type     = Type_T;

      struct Interval : VectorT<Type, 2ul> {
        Type start() const { return this->operator[](0ul); }
        Type end() const {return this->operator [](1ul);}
      };

      enum class TraceDirection { LeftToRight, RightToLeft };
      enum class Qualification {
        Open,            // (a,b)
        Closed,          // [a,b]
        HalfOpenStart,   // (a,b]
        HalfOpenEnd,     // [a,b)
        Loop             // [a,b) and I(a) == I(b)
      };
    };
  }   // namespace topology

  namespace parameterization {

    /**!
     * Gives the questions
     * evaluateAt(   domain position ) -> Point
     * derivativeAt( domain position ) -> Vector
     */
    template <typename Space_T>
    requires gmc::spaces::Space<Space_T>and gmc::spaces::OneDimensionalSpace<
      Space_T>
    struct IntervalParameterization {

      // clang-format off
      using DomainSpace      = Space_T;
      using Type             = typename DomainSpace::Type;

      // Topology
      using IntervalTopology = typename topology::IntervalMeta<Type>;
      using Interval         = typename IntervalTopology::Interval;
      using Qualification    = typename IntervalTopology::Qualification;

      // Trace
      using TraceDirection   = typename IntervalTopology::TraceDirection;
      // clang-format on
    };


    template <typename Space_T>
    requires gmc::spaces::Space<Space_T>
    struct TensorProductParameterization {

      // clang-format off
      using DomainSpace               = Space_T;
      using Type                      = typename DomainSpace::Type;
      static constexpr auto Dimension = DomainSpace::Dimension;

      // Topology
      using TensorAxisTopology = typename topology::IntervalMeta<Type>;
      using Interval           = typename TensorAxisTopology::Interval;
      using Qualification      = typename TensorAxisTopology::Qualification;

      // Trace
      using TraceDirection     = typename TensorAxisTopology::TraceDirection;
      using TraceDirArray      = std::array<TraceDirection, Dimension>;
      using TraceDDPair        = std::pair<TraceDirection, VectorT<Type,Dimension>>;

      // clang-format on
    };


    namespace gbc {

      template <typename Space_T>
      requires gmc::spaces::Space<
        Space_T>
      struct MVCParameterization {
        using DomainSpace = Space_T;
      };
    }

  }   // namespace parameterization

//  namespace domain
//  {






    /********
     *
     * Domain/DomainInfo TEMPLATES
     */
    template <typename Parameterization_T>
    struct Domain;

    template <typename Domain_T>
    struct DomainInfo;



    /********
     *
     * Domain/DomainInfo SPECIALIZATION -- using declarations
     */

    template <typename Space_T>
    using IntervalDomain
      = Domain<parameterization::IntervalParameterization<Space_T>>;

    template <typename Space_T>
    using IntervalDomainInfo = DomainInfo<IntervalDomain<Space_T>>;

    template <typename Space_T>
    using TensorProductDomain
      = Domain<parameterization::TensorProductParameterization<Space_T>>;

    template <typename Space_T>
    using TensorProductDomainInfo = DomainInfo<TensorProductDomain<Space_T>>;


    template <typename Space_T>
    using MeanValueCoordinateDomain
      = Domain<parameterization::gbc::MVCParameterization<Space_T>>;





    /********
     *
     * Domain/DomainInfo SPECIALIZATION -- definitions
     */



    /***
     * Interval Curve
     */
    template <typename Type_T/*typename Parameterization_T*/>
    struct Domain<parameterization::IntervalParameterization<
      spaces::ProjectiveSpace<Type_T, 1ul>>> {

      // Domain
      using Parameterization = parameterization::IntervalParameterization<
        spaces::ProjectiveSpace<Type_T, 1ul>>;
      using DomainSpace = typename Parameterization::DomainSpace;
      using Type        = typename DomainSpace::Type;
      using DomainInfo  = gm::parametric::domain::DomainInfo<Domain<Parameterization>>;

      // Topology
      using IntervalTopology = typename Parameterization::IntervalTopology;
      using Qualification    = typename IntervalTopology::Qualification;
      using Interval         = typename IntervalTopology::Interval;

      // Trace
      using TraceDirection = typename Parameterization::TraceDirection;
    };


    template <typename Type_T/*typename Domain_T*/>
    struct DomainInfo<Domain<parameterization::IntervalParameterization<
      spaces::ProjectiveSpace<Type_T, 1ul>>>> {

      using Domain = IntervalDomain<spaces::ProjectiveSpace<Type_T, 1ul>>;
      using Qualification = typename Domain::Qualification;
      using Interval      = typename Domain::Interval;

      // Data (runtime/mutable states)
      Qualification Iq;
      Interval      I;
    };



    /***
     * Tensor Product Surface and Volume
     */

    template <typename Type_T, size_t Dimension_T>
    struct Domain<parameterization::TensorProductParameterization<
      spaces::ProjectiveSpace<Type_T, Dimension_T>>> {

      // Domain
      using Parameterization = parameterization::TensorProductParameterization<
                    spaces::ProjectiveSpace<Type_T, Dimension_T>>;
      using DomainSpace = typename Parameterization::DomainSpace;
      using Type        = typename DomainSpace::Type;
      using DomainInfo  = gm::parametric::domain::DomainInfo<Domain<Parameterization>>;

      // Topology
      using TensorAxisTopology = typename Parameterization::TensorAxisTopology;
      using Qualification      = typename TensorAxisTopology::Qualification;
      using Interval           = typename TensorAxisTopology::Interval;

      // Trace
      using TraceDirection = typename Parameterization::TraceDirection;
    };

    template <typename Type_T>
    struct DomainInfo<Domain<parameterization::TensorProductParameterization<
      spaces::ProjectiveSpace<Type_T, 2ul>>>> {

      using Domain = gm::parametric::domain::Domain<parameterization::TensorProductParameterization<
        spaces::ProjectiveSpace<Type_T, 2ul>>>;
      using Qualification = typename Domain::Qualification;
      using Interval      = typename Domain::Interval;

      // Data (runtime/mutable states)
      Qualification Uq;
      Qualification Vq;
      Interval      U;
      Interval      V;
    };

    template <typename Type_T>
    struct DomainInfo<Domain<parameterization::TensorProductParameterization<
      spaces::ProjectiveSpace<Type_T, 3ul>>>> {

      using Domain = gm::parametric::domain::Domain<parameterization::TensorProductParameterization<
        spaces::ProjectiveSpace<Type_T, 3ul>>>;
      using Qualification = typename Domain::Qualification;
      using Interval      = typename Domain::Interval;

      // Data (runtime/mutable states)
      Qualification Uq;
      Qualification Vq;
      Qualification Wq;
      Interval      U;
      Interval      V;
      Interval      W;
    };



    /***
     * Polygon MeanValueCoordinates
     */
    template <typename Type_T, size_t Dimension_T>
    struct Domain<parameterization::gbc::MVCParameterization<
      spaces::ProjectiveSpace<Type_T, Dimension_T>>> {

      // Domain
      using Parameterization = parameterization::gbc::MVCParameterization<
        spaces::ProjectiveSpace<Type_T, Dimension_T>>;
      using DomainSpace = typename Parameterization::DomainSpace;
      using Type        = typename DomainSpace::Type;
      using DomainInfo  = gm::parametric::domain::DomainInfo<Domain<Parameterization>>;

      // Polygondomain types
      using Lambda           = DVectorT<Type>;
      using Lambdas          = DVectorT<Lambda>;
      using Point            = typename DomainSpace::Point;
//      using PointH           = typename DomainSpace::PointH;
      using Polygon          = DVectorT<Point>;



      // Topology
//      using TensorAxisTopology = typename Parameterization::TensorAxisTopology;
//      using Qualification      = typename TensorAxisTopology::Qualification;
//      using Interval           = typename TensorAxisTopology::Interval;

      // Trace
//      using TraceDirection = typename Parameterization::TraceDirection;
    };

    template <typename Type_T>
    struct DomainInfo<Domain<parameterization::gbc::MVCParameterization<
      spaces::ProjectiveSpace<Type_T, 2ul>>>> {

      using Domain    = gm::parametric::domain::Domain<parameterization::gbc::MVCParameterization<
        spaces::ProjectiveSpace<Type_T, 2ul>>>;
      using Polygon   = typename Domain::Polygon;

      // Data (runtime/mutable states)
      Polygon polygon;
    };



//    template <typename Type_T>
//    struct DomainInfo<TensorProductDomain<Type_T, 3ul>> {

//      using Domain        = TensorProductDomain<Type_T, 3ul>;
//      using Qualification = typename Domain::Qualification;
//      using Interval      = typename Domain::Interval;

//      // Data (runtime/mutable states)
//      Qualification Uq;
//      Qualification Vq;
//      Qualification Wq;
//      Interval      U;
//      Interval      V;
//      Interval      W;
//    };

//    template <typename Type_T>
//    struct Domain<parameterization::TensorProductParameterization<
//                    spaces::ProjectiveSpace<Type_T, 3ul>>,
//                  spaces::ProjectiveSpace<Type_T, 3ul>> {

//      // Domain
//      using Parameterization = parameterization::TensorProductParameterization<
//                    spaces::ProjectiveSpace<Type_T, 3ul>>;
//      using DomainSpace = typename Parameterization::DomainSpace;
//      using Type        = typename DomainSpace::Type;

//      // Topology
//      using TensorAxisTopology = typename Parameterization::TensorAxisTopology;
//      using Qualification      = typename TensorAxisTopology::Qualification;
//      using Interval           = typename TensorAxisTopology::Interval;

//      // Data (runtime/mutable states)
//      Qualification Uq;
//      Qualification Vq;
//      Qualification Wq;
//      Interval      U;
//      Interval      V;
//      Interval      W;
//    };





    //  template <typename Space_T>
    //  requires gmc::spaces::Space<Space_T> and
    //  gmc::spaces::ThreeDimensionalSpace<Space_T> struct TensorProduct {
    //    using DomainSpace = Space_T;

    //    struct Meta {
    //      enum class Topology { Open = false, Closed = true };
    //      Topology                   topology_u;
    //      Topology                   topology_v;
    //      Topology                   topology_w;
    //      typename DomainSpace::Type start_u;
    //      typename DomainSpace::Type start_v;
    //      typename DomainSpace::Type start_w;
    //      typename DomainSpace::Type end_u;
    //      typename DomainSpace::Type end_v;
    //      typename DomainSpace::Type end_w;
    //    };
    //  };

    //  template <gmc::spaces::Space Space_T>
    //  struct TensorProductMultiMap {
    //    using DomainSpace = Space_T;
    //  };



//  }   // namespace domain

}   // namespace gm::parametric::domain

#endif   // GM2_PARAMETERIZATIONS_H
