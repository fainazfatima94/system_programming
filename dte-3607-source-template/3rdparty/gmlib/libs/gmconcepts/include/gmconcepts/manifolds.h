#ifndef GMC_MANIFOLDS_H
#define GMC_MANIFOLDS_H


#include "mapping.h"
#include "parameterization.h"


namespace gmc
{


  /*
   * Curves
   */

  template <typename Curve_T>
  concept Curve = MappingObject<Curve_T>and
    parameterization::Curve<typename Curve_T::Parameterization>;

  template <typename Curve_T>
  concept ParametricCurveKernel = MappingKernel<Curve_T>and
    parameterization::Curve<typename Curve_T::Parameterization>and requires(
      Curve_T const& C, size_t order, typename Curve_T::Type t,
      typename Curve_T::TraceDirection
        trace_dir)   // Enforces constness on callable member functions
  {
    // clang-format off
    { C.domain() } -> std::same_as<typename Curve_T::DomainInfo const&>;

    { C.evaluateAt(t,trace_dir) } -> std::same_as<typename Curve_T::PointH>;
//    { C.derivativeAt(order,t,trace_dir) } -> std::same_as<std::optional<typename Curve_T::VectorH>>;
    // clang-format on
  };

  template <typename Curve_T>
  concept ParametricCurve = Curve<Curve_T>and ParametricCurveKernel<Curve_T>;



  /*
   * Surfaces
   */

  template <typename Surface_T>
  concept Surface = MappingObject<Surface_T>and
    parameterization::Surface<typename Surface_T::Parameterization>;

  template <typename Surface_T>
  concept TensorProductSurfaceKernel
    = MappingKernel<Surface_T>and parameterization::
      TensorProductSurface<typename Surface_T::Parameterization>and requires(
        Surface_T const& S, typename Surface_T::DSizeArray order,
        typename Surface_T::DPoint uv,
        typename Surface_T::DTraceDirArray
          trace_dir)   // Enforces constness on callable member functions
  {
    // clang-format off
    { S.domain() } -> std::same_as<typename Surface_T::DomainInfo const&>;

    { S.evaluateAt(uv, trace_dir) } ->std::same_as<typename Surface_T::PointH>;
//    { S.partialDerivativeAt(order, uv, trace_dir) } ->std::same_as<std::optional<typename Surface_T::VectorH>>;
    // clang-format on
  };


  template <typename Surface_T>
  concept TensorProductSurface
    = Surface<Surface_T>and TensorProductSurfaceKernel<Surface_T>;


  template <typename Surface_T>
  concept PolygonalSurface = Surface<Surface_T>;




  /*
   * SubMapping manifolds
   */

  template <typename SubCurve_T>
  concept SubCurve = Curve<SubCurve_T>and MappingMappingObject<SubCurve_T>;

  template <typename SubSurface_T>
  concept SubSurface
    = Surface<SubSurface_T>and MappingMappingObject<SubSurface_T>;


}   // namespace gmc



#endif   // GMC_MANIFOLDS_H
