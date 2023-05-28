#ifndef GM2_PARAMETRIC_UTILS_MAPPINGOBJECT_UTILS_H
#define GM2_PARAMETRIC_UTILS_MAPPINGOBJECT_UTILS_H

#include "numeric_differentiation.h"

// gmconcepts
#include <gmconcepts/mapping.h>

namespace gm::parametric::mappingobject::utils
{

  template <typename MappingObject_T>
  typename MappingObject_T::VectorH
  derivativeAt(MappingObject_T const&                   obj,
               typename MappingObject_T::DPoint const&  par,
               typename MappingObject_T::DVector const& dir,
               typename MappingObject_T::Type const&    lim_step = 1e-6)

    requires gmc::MappingObject<MappingObject_T>and requires
  {
    {
      obj.evaluateAt(par)
    }
    ->std::same_as<typename MappingObject_T::PointH>;
  }
  {
    static constexpr divideddifference::fd::CentralDifference D{};
    const auto                                                Ds
      = D(par, blaze::normalize(dir) * lim_step, [&obj](const auto& at_par) {
          return blaze::evaluate(
            blaze::subvector<0ul, MappingObject_T::VectorDim>(
              obj.evaluateAt(at_par)));
        });

    return ::gm::utils::extendStaticContainer<
      typename MappingObject_T::Point, typename MappingObject_T::VectorH,
      MappingObject_T::VectorDim, 1UL>(Ds, typename MappingObject_T::Type(0));
  }

}   // namespace gm::parametric::mappingobject::utils




#endif   // GM2_PARAMETRIC_UTILS_MAPPINGOBJECT_UTILS_H
