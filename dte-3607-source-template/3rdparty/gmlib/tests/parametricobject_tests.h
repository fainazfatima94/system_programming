#ifndef PARAMETRICOBJECT_TESTS_H
#define PARAMETRICOBJECT_TESTS_H

#include <gtest/gtest.h>   // googletest header file

#include <parametric/parametricobject.h>

namespace gmlib2::testing {

  template <typename ParametricObject_T>
  void isParametricObject() {

    EXPECT_TRUE(gmlib2::traits::Is_VectorSpace<
                typename ParametricObject_T::PSpace>::value);
    EXPECT_TRUE(gmlib2::traits::Is_AffineSpace<
                typename ParametricObject_T::PSpace>::value);
    EXPECT_TRUE(gmlib2::traits::Is_ProjectiveSpace<
                typename ParametricObject_T::PSpace>::value);
    EXPECT_TRUE(gmlib2::traits::Is_ParametricObject<ParametricObject_T>::value);
  }

}




#endif // PARAMETRICOBJECT_TESTS_H
