#ifndef GMC_MAPPING_H
#define GMC_MAPPING_H

#include "spaces.h"
#include "parameterization.h"

namespace gmc {


  template <typename Object_T>
  concept SpaceObject = spaces::Space<typename Object_T::Space>;



  template <typename Kernel_T>
  concept MappingKernel =
    // Construct predicates
    // clang-format off
    spaces::Space<typename Kernel_T::DomainSpace> and
    parameterization::Parameterization<typename Kernel_T::Parameterization> and
    //
    spaces::Space<typename Kernel_T::EmbedSpace> and
    SpaceObject<typename Kernel_T::EmbedSpaceObject> and
    // clang-format on

    // Transitive Requirements
    requires
  {
    typename Kernel_T::Type;
  };



  template <typename Object_T>
  concept MappingObject =

    // Mapping object IS a mapping kernel
    MappingKernel<Object_T> and

    // Construct predicates
    // clang-format off
//    MappingKernel<Object_T> and
    MappingKernel<typename Object_T::MappingKernel> and
    spaces::Space<typename Object_T::DomainSpace> and
    SpaceObject<typename Object_T::EmbedSpaceObject>  and
    parameterization::Parameterization<typename Object_T::Parameterization> and
    // clang-format on

    // Transitive Requirements
    requires
  {
    // Basic requirements
    typename Object_T::Type;
    typename Object_T::MappingKernel;   // domain- to embed-space

    // Domain space requirements
    {std::same_as<typename Object_T::Parameterization::DomainSpace,
                  typename Object_T::DomainSpace>};

    // Embed space requirements
    {std::same_as<typename Object_T::EmbedSpaceObject::Space,
                  typename Object_T::EmbedSpace>};
    requires Object_T::EmbedSpace::Dimension
      == Object_T::EmbedSpaceObject::Dimension;
  };










  template <typename Kernel_T>
  concept MappingMappingKernel =

    // MappingMappingKernel IS A MappingKernel
      MappingKernel<Kernel_T> and

    // clang-format off
//    spaces::Space<typename Kernel_T::DomainSpace> and
//    parameterization::Parameterization<typename Kernel_T::Parameterization> and
    //
    spaces::Space<typename Kernel_T::MappingSpace> and
    SpaceObject<typename Kernel_T::MappingSpaceObject> and
    parameterization::Parameterization<typename Kernel_T::MappingSpaceParameterization> and
    MappingObject<typename Kernel_T::MappingObject> and
    MappingKernel<typename Kernel_T::MappingObjectKernel> and
    //
//    spaces::Space<typename Kernel_T::EmbedSpace> and
//    SpaceObject<typename Kernel_T::EmbedSpaceObject> and
    MappingObject<typename Kernel_T::EmbedObject> and
    MappingKernel<typename Kernel_T::EmbedObjectKernel>
    // clang-format off

    // Transitive Requirements
//    requires
//  {
//    typename Kernel_T::Type;
//  }
    // Method Requirements
//  and requires(Kernel_T& O)
//  {
//    {std::same_as<decltype(O.mappingObject()),
//                  typename Kernel_T::MappingObject>};
//    {std::same_as<decltype(O.embedObject()),
//                  typename Kernel_T::EmbedObject>};
//  }
//  and requires(Kernel_T const& O)
//  {
//    {std::same_as<typename std::decay_t<decltype(O.mappingObject())>,
//                  typename Kernel_T::MappingObject>};
//    {std::same_as<typename std::decay_t<decltype(O.embedObject())>,
//                  typename Kernel_T::EmbedObject>};
//  }
      ;











  template <typename Object_T>
  concept MappingMappingObject =

    // IS A
//    MappingKernel<Object_T> and
    MappingMappingKernel<Object_T> and

    // HAS A
    // Construct predicates
    // clang-format off
    MappingObject<Object_T> and
//    MappingMappingKernel<Object_T> and
    MappingMappingKernel<typename Object_T::MappingMappingKernel> and
    SpaceObject<typename Object_T::MappingSpaceObject> and
    // clang-format on

    // API requirements -- non-const
    requires(Object_T& obj)
  {
    // clang-format off
    { obj.mappingObject() } -> std::same_as<typename Object_T::MappingObject&>;
    { obj.embedObject() }   -> std::same_as<typename Object_T::EmbedObject&>;
    // clang-format on
  }
  and
    // API requirements -- const
    requires(Object_T const& obj)
  {
    // clang-format off
    { obj.mappingObject() } -> std::same_as<typename Object_T::MappingObject const&>;
    { obj.embedObject() }   -> std::same_as<typename Object_T::EmbedObject const&>;
    // clang-format on
  }
  and

    // Transitive Requirements
    requires
  {
    // Basic requirements
    typename Object_T::MappingObjectKernel;    // domain- to mapping-space
    typename Object_T::EmbedObjectKernel;      // mapping- to embed-space
    typename Object_T::MappingMappingKernel;   // domain- to embed-space
    {std::same_as<typename Object_T::MappingKernel,
                  typename Object_T::MappingMappingKernel>};

    // Mapping space requirements
    {spaces::Space<typename Object_T::MappingSpace>};
    {parameterization::Parameterization<
      typename Object_T::MappingSpaceParameterization>};

    // Object type requirements
    typename Object_T::MappingObject;   // domain- to mapping-space
    typename Object_T::EmbedObject;     // mapping- to embed-space

    // Mapping kernel congruency requirements
    {std::same_as<typename Object_T::MappingObject::EmbedSpace,
                  typename Object_T::EmbedObject::DomainSpace>};
  };


}   // namespace gmc


#endif // GMC_MAPPING_H
