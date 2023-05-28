#ifndef GM2_CORE_BASISGENERATORS_BERNSTEINBASISGENERATORS_H
#define GM2_CORE_BASISGENERATORS_BERNSTEINBASISGENERATORS_H

#include "../gm2_blaze.h"

namespace gm::basis
{

  namespace detail
  {

    template <typename Unit_Type_T>
    inline   // expression 5.45 (page 119) in "Blend book".
      Unit_Type_T
      getW(const DVectorT<Unit_Type_T>& tv, Unit_Type_T t, size_t i, size_t d)
    {
      return (t - tv[i]) / (tv[i + d] - tv[i]);
    }


    template <typename Unit_Type_T>
    inline   // expression 5.47 (page 121) in "Blend book".
      Unit_Type_T
      delta(const DVectorT<Unit_Type_T>& tv, size_t i, size_t d, Unit_Type_T s)
    {
      return s / (tv[i + d] - tv[i]);
    }

  }   // namespace detail


  template <typename Unit_Type_T>
  inline DMatrixT<Unit_Type_T>
  generateBSplineBasisMatrix(const DVectorT<Unit_Type_T>& tv, size_t tvi,
                             Unit_Type_T t, size_t d,
                             Unit_Type_T scale = Unit_Type_T(1))
  {
    // The knot-index tvi must be such that: tv[tvi] <= t <= tv[tvi+1], depending
    // on left / right evaluation

    DMatrixT<Unit_Type_T> mat;

    // For the linar factor - function mapping the knot intervalls to [0,1].
    DVectorT<Unit_Type_T> w(d);

    // Compute the B-splines (polynomials), degree 1 -> d, one for each row.
    // Starts from the second bottom row (degree 1), then goes upwards (degree
    // 2,...,d).
    mat.resize(d + 1, d + 1);
    mat(d - 1,1) = detail::getW(tv, t, tvi, 1);
    mat(d - 1,0) = 1 - mat(d - 1,1);

    {
      ;
      for (int i = int(d) - 2, k = 2; i >= 0; i--, k++) {
        // Generate w, expression 5.45 (page 119) in "Blend book".
        for (int j = 0; j < k; j++)
          w[size_t(j)]
            = detail::getW(tv, t, size_t(int(tvi) - k + j + 1), size_t(k));

        // Compute the k b-splines
        mat(size_t(i),0) = (1 - w[0]) * mat(size_t(i) + 1,0);
        for (int j = 1; j < int(d) - i; j++)
          mat(size_t(i), size_t(j))
            = w[size_t(j - 1)] * mat(size_t(i + 1), size_t(j - 1))
              + (1 - w[size_t(j)]) * mat(size_t(i + 1), size_t(j));
        mat(size_t(i), size_t(int(d) - i))
          = w[size_t(k - 1)] * mat(size_t(i + 1), size_t(int(d) - i - 1));
      }
    }

    // Compute all deriatives for the derivatives B-splines (polynomials), 1st,
    // 2nd,...,d-order derivatives.
    mat(d,1) = detail::delta(tv, tvi, 1, scale);
    mat(d,0) = -mat(d,1);

    for (int k = 2; k <= int(d); k++) {
      // Generate w' (delta) for the derivatives, expression 5.47 (page 121) in
      // "Blend book".
      for (int j = 0; j < k; j++)
        w[size_t(j)] = k * detail::delta(tv, size_t(int(tvi) - k + j + 1), size_t(k), scale);

      for (int i = int(d); i > int(d) - k; i--) {
        mat(size_t(i),size_t(k)) = w[size_t(k - 1)] * mat(size_t(i),size_t(k - 1));
        for (int j = k - 1; j > 0; j--)
          mat(size_t(i), size_t(j))
            = w[size_t(j - 1)] * mat(size_t(i), size_t(j - 1))
              - w[size_t(j)] * mat(size_t(i), size_t(j));
        mat(size_t(i), 0) = -w[0] * mat(size_t(i), 0);
      }
    }

    return mat;
  }

  template <typename Unit_Type_T>
  inline DMatrixT<Unit_Type_T>
  generateBernsteinBasisMatrix(int d, Unit_Type_T t,
                               Unit_Type_T scale = Unit_Type_T(1))
  {

    // Described on page 91-92 in "Blend book"

    // Initiate result matrix
    DMatrixT<Unit_Type_T> mat(size_t(d + 1), size_t(d + 1));

    // Escape if the degree is 0
    if (d < 1) {
      mat(0, 0) = Unit_Type_T(1);
      return mat;
    }

    // Compute the Bernstein (polynomials), degree 1 -> d, one for each row.
    // Starts from the second bottom row (degree 1), then goes upwards (degree
    // 2,...,d).
    mat(size_t(d - 1), size_t(0)) = 1 - t;
    mat(size_t(d - 1), size_t(1)) = t;

    for (int i = d - 2; i >= 0; i--) {
      mat(size_t(i), size_t(0)) = (1 - t) * mat(size_t(i + 1), size_t(0));
      for (int j = 1; j < d - i; j++)
        mat(size_t(i), size_t(j)) = t * mat(size_t(i + 1), size_t(j - 1))
                                    + (1 - t) * mat(size_t(i + 1), size_t(j));
      mat(size_t(i), size_t(d - i)) = t * mat(size_t(i + 1), size_t(d - i - 1));
    }

    // Compute all the deriatives
    mat(size_t(d), size_t(0)) = -scale;
    mat(size_t(d), size_t(1)) = scale;

    for (int k = 2; k <= d; k++) {
      const double s = k * scale;
      for (int i = d; i > d - k; i--) {
        mat(size_t(i), size_t(k)) = s * mat(size_t(i), size_t(k - 1));
        for (int j = k - 1; j > 0; j--)
          mat(size_t(i), size_t(j))
            = s * (mat(size_t(i), size_t(j - 1)) - mat(size_t(i), size_t(j)));
        mat(size_t(i), size_t(0)) = -s * mat(size_t(i), size_t(0));
      }
    }

    return mat;
  }


}   // namespace gm::basis

#endif   // GM2_CORE_BASISGENERATORS_BERNSTEINBASISGENERATORS_H
