#ifndef GM2_PARAMETRIC_UTILS_BSPLINEUTILS_H
#define GM2_PARAMETRIC_UTILS_BSPLINEUTILS_H


#include "../../core/gm2_blaze.h"


namespace gm::parametric::bspline
{

  template <typename Unit_T>
  using BSplineKnotVector = DVectorT<Unit_T>;

  template <typename Unit_T>
  auto generateUniformKnotVector(const size_t& no_coefs, const size_t& degree)
  {
    const auto order = degree + 1;

    BSplineKnotVector<Unit_T> t(no_coefs + order);
    int                        step_knots = int(t.size()) - int((order * 2));

    // implicit paramterization [0,max(knot_value)]
    Unit_T knot_value = Unit_T(0);
    size_t i          = 0;

    // Set the start knots
    for (; i < order; i++) t[i] = knot_value;

    // Set the "step"-knots
    for (int j = 0; j < step_knots; j++) t[i++] = ++knot_value;

    // Set the end knots
    knot_value++;
    for (; i < t.size(); i++) t[i] = knot_value;

    // implicit normalize paramterization [0,1]
    t *= 1 / Unit_T(knot_value);

    return t;
  }

  template <typename Unit_T>
  auto findKnotIndex(const BSplineKnotVector<Unit_T>& T, const Unit_T& t,
                     size_t degree)
  {
    auto ti = degree;
    while (t > T[ti + 1]) ti++;
    return ti;
  }




}   // namespace gm::parametric::bspline




#endif   // GM2_PARAMETRIC_UTILS_BSPLINEUTILS_H
