#ifndef GMC_PARAMETERIZATION_H
#define GMC_PARAMETERIZATION_H

#include "spaces.h"

namespace gmc::parameterization
{

  template <typename Parameterization_T>
  concept Parameterization
    = spaces::Space<typename Parameterization_T::DomainSpace>;

  template <typename Parameterization_T>
  concept Curve = Parameterization<Parameterization_T>and
    spaces::OneDimensionalSpace<typename Parameterization_T::DomainSpace>;

  template <typename Parameterization_T>
  concept CurveInterval = Curve<Parameterization_T>;

  template <typename Parameterization_T>
  concept Surface = Parameterization<Parameterization_T>and
    spaces::TwoDimensionalSpace<typename Parameterization_T::DomainSpace>;


  template <typename Parameterization_T>
  concept TensorProductSurface = Surface<Parameterization_T>;


}   // namespace gmc::parameterization


#endif // GMC_PARAMETERIZATION_H
