#ifndef GMC_PLATFORM_H
#define GMC_PLATFORM_H



#ifdef _MSVC_LANG

// Alternate spelling of operator keywords
#include <ciso646>

#endif   // _MSVC_LANG




// stl
//#include <concepts>
#include <type_traits>


#ifndef __cpp_lib_concepts

/////////////////////////////////
// Concepts from cppreference.com
// -- implementation examples

namespace std
{

  ////////////////////
  // Core language concepts
  namespace detail
  {
    template <class T, class U>
    concept SameHelper = is_same_v<T, U>;
  }

  template <class T, class U>
  concept same_as = detail::SameHelper<T, U>&& detail::SameHelper<U, T>;

  template <class Derived, class Base>
  concept derived_from = is_base_of_v<Base, Derived>&&
    is_convertible_v<const volatile Derived*, const volatile Base*>;

  template <class From, class To>
  concept convertible_to
    = is_convertible_v<From, To>&& requires(add_rvalue_reference_t<From> (&f)())
  {
    static_cast<To>(f());
  };

  template <class T>
  concept integral = is_integral_v<T>;

  template <class T>
  concept signed_integral = is_integral_v<T>&& is_signed_v<T>;

  template <class T>
  concept unsigned_integral = integral<T> && !signed_integral<T>;


  template <class T>
  concept floating_point = is_floating_point_v<T>;










  ////////////////////
  // Callable concepts
  template <class F, class... Args>
  concept invocable = requires(F&& f, Args&&... args)
  {
    invoke(forward<F>(f), forward<Args>(args)...);
    /* not required to be equality preserving */
  };

  template <class F, class... Args>
  concept regular_invocable = invocable<F, Args...>;



}   // namespace std
#endif




#endif   // GMC_PLATFORM_H
