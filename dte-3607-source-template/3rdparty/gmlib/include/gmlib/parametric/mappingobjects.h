#ifndef GM2_PARAMETRIC_MAPPINGOBJECTS_H
#define GM2_PARAMETRIC_MAPPINGOBJECTS_H

// gm
#include "../core/spaceobject.h"
#include "../core/coreutils.h"

#include "parameterizations.h"

// gmconcepts
#include <gmconcepts/parameterization.h>

// stl
#include <utility> //declval
#include <array>
#include <mutex>





#define GM2_MAPPING_KERNEL_BOILER_PLATE                                        \
                                                                               \
  /* Embed space types */                                                      \
  GM2_DEFINE_PROJECTIVESPACE_TYPES(EmbedSpace)                                 \
                                                                               \
  /* Domain space types */                                                     \
  GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(D, DomainSpace)                      \
                                                                               \
  /* Static domain space types */                                              \
  using DSizeArray    = std::array<size_t, DomainSpace::VectorDim>;            \
  using DBoolArray    = std::array<bool, DomainSpace::VectorDim>;


namespace gm::parametric::tt
{


  template <typename EmbedSpace_T, typename DomainSpace_T>
  struct TT_MappingObject
  {
    /* Embed space types */
    GM2_DEFINE_PROJECTIVESPACE_TYPES(EmbedSpace_T)

    /* Domain space types */
    GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(D, DomainSpace_T)

    /* Static domain space types */
    using DSizeArray    = std::array<size_t, DomainSpace_T::VectorDim>;
    using DBoolArray    = std::array<bool, DomainSpace_T::VectorDim>;
  };


  template <typename MappingObject_T, typename EmbedObject_T>
  struct TT_MappingMappingObjectKernel
  {
    using DomainSpace = typename MappingObject_T::DomainSpace;
    GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(D, DomainSpace)
    using Domain           = typename MappingObject_T::Domain;
    using DomainInfo       = typename MappingObject_T::DomainInfo;
    using Parameterization = typename MappingObject_T::Parameterization;
    using DSizeArray       = typename MappingObject_T::DSizeArray;
    using DBoolArray       = typename MappingObject_T::DBoolArray;

    /* Mapping space */
    using MappingSpace =
      typename EmbedObject_T::DomainSpace; /* ==  MappingObjectType::EmbedSpace */
    using MappingSpaceParameterization = typename EmbedObject_T::Parameterization;
    using MappingSpaceObject           = typename MappingObject_T::EmbedSpaceObject;
    using MappingObjectKernel          = typename MappingObject_T::MappingKernel;
    GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(M, MappingSpace)
    using MSizeArray = typename EmbedObject_T::DSizeArray;
    using MBoolArray = typename EmbedObject_T::DBoolArray;

    /* Embed space */
    using EmbedSpace        = typename EmbedObject_T::EmbedSpace;
    using EmbedSpaceObject  = typename EmbedObject_T::EmbedSpaceObject;
    using EmbedObjectKernel = typename EmbedObject_T::MappingKernel;
    GM2_DEFINE_PROJECTIVESPACE_TYPES(EmbedSpace)
  };

}






#define GM2_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MAPPINGOBJECT_T,                 \
                                                EMBEDOBJECT_T)                   \
                                                                                 \
  using MappingObjectType = MAPPINGOBJECT_T;                                         \
  using EmbedObjectType   = EMBEDOBJECT_T;                                           \
                                                                                 \
  /* Domain space */                                                             \
  using DomainSpace = typename MappingObjectType::DomainSpace;                       \
  GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(D, DomainSpace)                        \
  using Domain           = typename MappingObjectType::Domain;                       \
  using DomainInfo       = typename MappingObjectType::DomainInfo;                   \
  using Parameterization = typename MappingObjectType::Parameterization;             \
  using DSizeArray       = typename MappingObjectType::DSizeArray;                   \
  using DBoolArray       = typename MappingObjectType::DBoolArray;                   \
                                                                                 \
  /* Mapping space */                                                            \
  using MappingSpace =                                                           \
    typename EmbedObjectType::DomainSpace; /* ==  MappingObjectType::EmbedSpace */       \
  using MappingSpaceParameterization = typename EmbedObjectType::Parameterization;   \
  using MappingSpaceObject           = typename MappingObjectType::EmbedSpaceObject; \
  using MappingObjectKernel          = typename MappingObjectType::MappingKernel;    \
  GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(M, MappingSpace)                       \
  using MSizeArray = typename EmbedObjectType::DSizeArray;                           \
  using MBoolArray = typename EmbedObjectType::DBoolArray;                           \
                                                                                 \
  /* Embed space */                                                              \
  using EmbedSpace        = typename EmbedObjectType::EmbedSpace;                    \
  using EmbedSpaceObject  = typename EmbedObjectType::EmbedSpaceObject;              \
  using EmbedObjectKernel = typename EmbedObjectType::MappingKernel;                 \
  GM2_DEFINE_PROJECTIVESPACE_TYPES(EmbedSpace)









#define GM2_CURVE_IN_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MAPPINGOBJECT_T,      \
                                                         EMBEDOBJECT_T)        \
  GM2_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MAPPINGOBJECT_T, EMBEDOBJECT_T)      \
                                                                               \
  /* Evaluation type */                                                        \
  using EvaluationResult =                                                     \
    typename datastructures::parametrics::curve::EvaluationResult<VectorH>;









#define GM2_SURFACE_IN_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MAPPINGOBJECT_T,    \
                                                           EMBEDOBJECT_T)      \
  GM2_MAPPING_MAPPING_KERNEL_BOILER_PLATE(MAPPINGOBJECT_T, EMBEDOBJECT_T)      \
                                                                               \
  /* Evaluation type */                                                        \
  using EvaluationResult =                                                     \
    typename datastructures::parametrics::surface::EvaluationResult<VectorH>;










namespace gm::parametric
{


  namespace proto
  {

    // Check if a derivative of a given order is defined
    template <gmc::MappingObject T, size_t Order_T>
    struct isDerivativeOfOrderDefined : std::false_type {
    };

    // shorthand notation -- stl congruent
    template <gmc::MappingObject T, size_t Order_T>
    inline constexpr bool isDerivativeOfOrderDefined_v
      = isDerivativeOfOrderDefined<T, Order_T>::value;

    // Check for gmc::ParametricCurve types
    template <gmc::ParametricCurve Curve_T, size_t Order_T>
    requires requires(Curve_T const& curve, typename Curve_T::Type const& t)
    {
      {
        curve.template derivativeAt<Order_T>(t, std::declval<bool>())
      }
      ->std::same_as<typename Curve_T::VectorH>;
    }
    struct isDerivativeOfOrderDefined<Curve_T, Order_T> : std::true_type {
    };

  }   // namespace proto




  /* Type_T, EmbedDimension_T and EmbedSpace_T is provided for
   * partial-specialization purposes */
//  template <template <gmc::spaces::ProjectiveSpace /* EmbedSpace */>
//            typename MappingKernel_T,
//            gmc::spaces::ProjectiveSpace EmbedSpace_T>
  template <template <typename /* EmbedSpace */>
            typename MappingKernel_T,
            typename EmbedSpace_T>
  struct MappingObject : MappingKernel_T<EmbedSpace_T> {

    /* Base */
    using Base          = MappingKernel_T<EmbedSpace_T>;
    using MappingKernel = Base;
    using Type          = typename MappingKernel::Type;

    /* Domain space */
    using DomainSpace = typename MappingKernel::DomainSpace;
    GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(D, DomainSpace)
    using Parameterization = typename MappingKernel::Parameterization;

    using DSizeArray = typename MappingKernel::DSizeArray;
    using DBoolArray = typename MappingKernel::DBoolArray;

    /* Embed space */
    using EmbedSpace       = typename MappingKernel::EmbedSpace;
    using EmbedSpaceObject = typename MappingKernel::EmbedSpaceObject;
    GM2_DEFINE_PROJECTIVESPACE_TYPES(EmbedSpace)

    /* Constructor(s) */
    using Base::Base;


    // Auto generated methods when MappingObject meets the concept-requiremets
    // for TensorProductSurface
    DPoint startParameters() const
      requires gmc::parameterization::TensorProductSurface<Parameterization>
    {
      return DPoint{this->parameterUStart(), this->parameterVStart()};
    }

    DPoint endParameters() const
      requires gmc::parameterization::TensorProductSurface<Parameterization>
    {
      return DPoint{this->parameterUEnd(), this->parameterVEnd()};
    }

  };



//  template <template <gmc::spaces::ProjectiveSpace /* EmbedSpace */>
//            typename CurveTypeMappingKernel_T,
//            gmc::spaces::ProjectiveSpace EmbedSpace_T>
//  using CurveMappingObject = MappingObject<CurveTypeMappingKernel_T, EmbedSpace_T>;











  template <
    template <
      template <typename> typename /*MappingSpaceObject*/,
      typename /*EmbedSpaceObject*/ >
    typename MappingMappingKernel_T,
    template < typename /*EmbedSpace (MappingSpace) */ >
    typename MappingSpaceObject_T,
    typename EmbedSpace_T>
  struct MappingMappingObject
    : MappingMappingKernel_T<MappingSpaceObject_T, EmbedSpace_T>
  {

    /* Base */
    using Base = MappingMappingKernel_T<MappingSpaceObject_T, EmbedSpace_T>;





    /* NEW types */

    // Invariant types (from input templates)
//    using MappingSpaceObject = MappingSpaceObject_T;
//    using EmbedSpaceObject = EmbedSpaceObject_T;

    using MappingMappingKernel
      = MappingMappingKernel_T<MappingSpaceObject_T, EmbedSpace_T>;
    // requires std::SameAs<MappingSpaceObject::EmbedSpace,EmbedSpaceObject::DomainSpace>;

    // Derived types
//    using DomainSpace  = typename MappingSpaceObject::DomainSpace;
//    using MappingSpace = typename EmbedSpaceObject::DomainSpace;
//    using EmbedSpace   = typename EmbedSpaceObject::EmbedSpace;




    /* NEW types -- END */





//    using MappingMappingKernel = Base;
    using Type                 = typename MappingMappingKernel::Type;

    /* Domain space */
    using DomainSpace = typename MappingMappingKernel::DomainSpace;
    GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(D, DomainSpace)
    using Parameterization = typename MappingMappingKernel::Parameterization;
    using DSizeArray       = typename MappingMappingKernel::DSizeArray;
    using DBoolArray       = typename MappingMappingKernel::DBoolArray;

      /* Mapping space */
    using MappingSpace = typename MappingMappingKernel::MappingSpace;
    using MappingSpaceParameterization =
      typename MappingMappingKernel::MappingSpaceParameterization;
    using MappingSpaceObject =
      typename MappingMappingKernel::MappingSpaceObject;
    using MappingObjectType = typename MappingMappingKernel::MappingObjectType;
    using MappingObjectKernel =
      typename MappingMappingKernel::MappingObjectKernel;
    GM2_DEFINE_PROJECTIVESPACE_TYPES_PREFIX(M, MappingSpace)
    using MSizeArray = typename MappingMappingKernel::MSizeArray;
    using MBoolArray = typename MappingMappingKernel::MBoolArray;

      /* Embed space */
    using EmbedSpace        = typename MappingMappingKernel::EmbedSpace;
    using EmbedSpaceObject  = typename MappingMappingKernel::EmbedSpaceObject;
    using EmbedObjectType       = typename MappingMappingKernel::EmbedObjectType;
    using EmbedObjectKernel = typename MappingMappingKernel::EmbedObjectKernel;
    GM2_DEFINE_PROJECTIVESPACE_TYPES(EmbedSpace)

    /* Transitivity */
    using MappingKernel = MappingMappingKernel;


    // Constructor(s)
    MappingMappingObject() = delete;
    using Base::Base;


//    // Auto generated domain space methods for when MappingObject meets the
//    // concept-requiremets of ParametricCurve
//    bool isIntervalClosed() const requires gmc::ParametricCurve<MappingObject>
//    {
//      return this->mappingObject().isIntervalClosed();
//    }
//    Type intervalStart() const requires gmc::ParametricCurve<MappingObject>
//    {
//      return this->mappingObject().intervalStart();
//    }
//    Type intervalEnd() const requires gmc::ParametricCurve<MappingObject>
//    {
//      return this->mappingObject().intervalEnd();
//    }

//    // Auto generated domain space methods for when MappingObject meets the
//    // concept-requiremets of TensorProductSurface
//    bool isClosedInU() const requires gmc::TensorProductSurface<MappingObject>
//    {
//      return this->mappingObject().isClosedInU();
//    }
//    bool isClosedInV() const requires gmc::TensorProductSurface<MappingObject>
//    {
//      return this->mappingObject().isClosedInV();
//    }
//    Type parameterUStart() const
//      requires gmc::TensorProductSurface<MappingObject>
//    {
//      return this->mappingObject().parameterUStart();
//    }
//    Type parameterUEnd() const requires gmc::TensorProductSurface<MappingObject>
//    {
//      return this->mappingObject().parameterUEnd();
//    }
//    Type parameterVStart() const
//      requires gmc::TensorProductSurface<MappingObject>
//    {
//      return this->mappingObject().parameterVStart();
//    }
//    Type parameterVEnd() const requires gmc::TensorProductSurface<MappingObject>
//    {
//      return this->mappingObject().parameterVEnd();
//    }
  };


}   // namespace gm::parametric

#endif   // GM2_PARAMETRIC_MAPPINGOBJECTS_H
