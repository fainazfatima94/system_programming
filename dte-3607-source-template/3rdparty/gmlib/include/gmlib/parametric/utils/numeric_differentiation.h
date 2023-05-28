#ifndef GM2_PARAMETRIC_UTILS_NUMERIC_DIFFERENTIATION_H
#define GM2_PARAMETRIC_UTILS_NUMERIC_DIFFERENTIATION_H


#include "../../core/divideddifference/finitedifference.h"
#include "../../core/gm2_blaze.h"

// gmconcepts
#include <gmconcepts/mapping.h>
#include <gmconcepts/manifolds.h>

namespace gm::parametric::numericdiff
{

  namespace finitediff {

    namespace fdkernel
    {

      template <typename = void>
      struct FD_ForwardDifference {

        template <std::floating_point Type_T, std::regular_invocable<Type_T> Fn_T>
        //
        constexpr std::pair<std::invoke_result_t<Fn_T, Type_T>,
                            std::invoke_result_t<Fn_T, Type_T>>
        //
        operator()(Type_T x, Type_T h, Fn_T fn) const
        // requires return_type_requirement<std::invoke_result_t<Fn_T, Type_T>>
        {
          //        using EType = typename std::decay_t<Vector_T>::ElementType;
          return std::pair(fn(x + h), fn(x));
        }

        template <typename TypeP_T, typename TypeV_T,
                  std::regular_invocable<TypeP_T> Fn_T>
        //
        //      constexpr std::pair<std::invoke_result_t<Fn_T, Type_T>,
        //                          std::invoke_result_t<Fn_T, Type_T>>
        auto
        //
        operator()(TypeP_T x, TypeV_T h, Fn_T fn) const
        // requires return_type_requirement<std::invoke_result_t<Fn_T, Type_T>>
        {
          //        using EType = typename std::decay_t<Vector_T>::ElementType;
          return std::pair(fn(x + h), fn(x));
        }
      };

      template <typename = void>
      struct FD_BackwardDifference {

        template <std::floating_point Type_T, std::regular_invocable<Type_T> Fn_T>
        //
        constexpr std::pair<std::invoke_result_t<Fn_T, Type_T>,
                            std::invoke_result_t<Fn_T, Type_T>>
        //
        operator()(Type_T x, Type_T h, Fn_T fn) const
        // requires return_type_requirement<std::invoke_result_t<Fn_T, Type_T>>
        {
          //        using EType = typename std::decay_t<Vector_T>::ElementType;
          return std::pair(fn(x), fn(x - h));
        }

        template <typename TypeP_T, typename TypeV_T,
                  std::regular_invocable<TypeP_T> Fn_T>
        //
        //      constexpr std::pair<std::invoke_result_t<Fn_T, Type_T>,
        //                          std::invoke_result_t<Fn_T, Type_T>>
        auto
        //
        operator()(TypeP_T x, TypeV_T h, Fn_T fn) const
        // requires return_type_requirement<std::invoke_result_t<Fn_T, Type_T>>
        {
          //        using EType = typename std::decay_t<Vector_T>::ElementType;
          return std::pair(fn(x), fn(x - h));
        }
      };


      template <typename = void>
      struct FD_CentralDifference {

        template <std::floating_point Type_T, std::regular_invocable<Type_T> Fn_T>
        //
        constexpr std::pair<std::invoke_result_t<Fn_T, Type_T>,
                            std::invoke_result_t<Fn_T, Type_T>>
        //
        operator()(Type_T x, Type_T h, Fn_T fn) const
        // requires return_type_requirement<std::invoke_result_t<Fn_T, Type_T>>
        {
          //        using EType = typename std::decay_t<Vector_T>::ElementType;
          return std::pair(fn(x + 0.5 * h), fn(x - 0.5 * h));
        }

        template <typename TypeP_T, typename TypeV_T,
                  std::regular_invocable<TypeP_T> Fn_T>
        //
        //      constexpr std::pair<std::invoke_result_t<Fn_T, Type_T>,
        //                          std::invoke_result_t<Fn_T, Type_T>>
        auto
        //
        operator()(TypeP_T x, TypeV_T h, Fn_T fn) const
        // requires return_type_requirement<std::invoke_result_t<Fn_T, Type_T>>
        {
          //        using EType = typename std::decay_t<Vector_T>::ElementType;
          return std::pair(fn(x + 0.5 * h), fn(x - 0.5 * h));
        }
      };

    }   // namespace fdkernel


    template <typename FDKernel_T>
    struct TwoPointFD {

      template <typename TypeX_T, typename TypeH_T, typename Fn_T>
      auto operator()(TypeX_T x, TypeH_T h, Fn_T fn) const
      {
        // Finite difference
        auto const [a, b] = op(x, h, std::forward<Fn_T>(fn));
        auto const dx = a - b;

        // Numeric differentiation
        if constexpr (std::is_floating_point_v<TypeH_T>)
          return blaze::evaluate(dx / h);
        else
          return blaze::evaluate(dx / blaze::length(h));
      }

    private:
      static constexpr FDKernel_T op{};
    };

    using ForwardFD  = TwoPointFD<fdkernel::FD_ForwardDifference<>>;
    using BackwardFD = TwoPointFD<fdkernel::FD_BackwardDifference<>>;
    using CentralFD  = TwoPointFD<fdkernel::FD_CentralDifference<>>;




    template <typename = void>
    struct FivePointStencil {

      template <typename TypeX_T, typename TypeH_T, typename Fn_T>
      auto operator()(TypeX_T x, TypeH_T h, Fn_T fn) const
      {
        // For sample coefs: -2, -1, 0, 1, 2
        auto const f0 = -fn(x + 2 * h);
        auto const f1 = 8 * fn(x + h);
        //      auto const f2 =  TypeX_T(0);
        auto const f3 = -8 * fn(x - h);
        auto const f4 = fn(x - 2 * h);

        auto const dx = f0 + f1 /*+ f2*/ + f3 + f4;
        auto const total_step = 12 * h;

        //      auto const [a, b] = op(x, h, std::forward<Fn_T>(fn));
        //      auto const dx = a - b;

        // Numeric differentiation
        if constexpr (std::is_floating_point_v<TypeH_T>)
          return blaze::evaluate(dx / total_step);
        else
          return blaze::evaluate(dx / blaze::length(total_step));
      }
    };

  }   // namespace finitediff



//  template <typename FDKernel_T>
//  struct NumericDifference {

//    template <typename TypeX_T, typename TypeH_T, typename Fn_T>
//    auto operator()(TypeX_T x, TypeH_T h, Fn_T fn) const
//    {
//      auto const dx = op(x, h, std::forward<Fn_T>(fn));
//      if constexpr (std::is_floating_point_v<TypeH_T>)
//        return blaze::evaluate(dx / h);
//      else
//        return blaze::evaluate(dx / blaze::length(h));
//    }

//  private:
//    static constexpr FiniteDifference<FDKernel_T> op{};
//  };










//  template <typename FDKernel_T, gmc::MappingMappingObject>
//  struct DirectionalDerivative {

//    template <typename TypeX_T, typename TypeH_T, typename Fn_T>
//    auto operator()(TypeX_T x, TypeH_T h, Fn_T fn) const
//    {
//      return op(x, h, fn);
//    }

//  private:
//    static constexpr NumericDifference<FDKernel_T> op{};
//  };

  template <typename FDKernel_T, gmc::MappingObject MappingObject_T,
            gmc::MappingObject EmbedObject_T>
  requires requires
  {
    requires MappingObject_T::EmbedSpace::Dimension
      == EmbedObject_T::DomainSpace::Dimension;
  }
  struct DirectionalDerivativeForMappingMappingKernel {

    // Maybe it could be usefull to specify the return type; explicitly
    // This version enforces a return type equal to the return type of the
    // Fn_T kernel
    template <std::regular_invocable<typename MappingObject_T::Type> FnM_T,
              std::regular_invocable<typename EmbedObject_T::DPoint> FnE_T>
    //
    typename EmbedObject_T::VectorH
    //
    operator()(typename MappingObject_T::Type xm,
               typename MappingObject_T::Type hm, FnM_T&& fnm,
               FnE_T&& fne) const requires
      // < -- here is the clue
      gmc::ParametricCurve<MappingObject_T>and
      //
      gmc::TensorProductSurface<EmbedObject_T>and
      // -- >
      std::same_as<std::invoke_result_t<FnE_T, typename EmbedObject_T::DPoint>,
                   typename EmbedObject_T::VectorH>
    {
      typename EmbedObject_T::DVector const xe
        = blaze::subvector<0, EmbedObject_T::DVectorDim>(fnm(xm));
      auto const vm = op_m(xm, hm, std::forward<FnM_T>(fnm));

      /*! \todo Look at this computation it might be dangerous */
      auto const he
        = blaze::normalize(blaze::subvector<0, EmbedObject_T::DVectorDim>(vm))
          * hm;
      /* todo END */

//      static constexpr NumericDifference<FDKernel_T> op_e{};
      static constexpr FDKernel_T op_e{};
      auto const                                     ve
        = op_e(xe, he, std::forward<FnE_T>(fne)) * blaze::length(vm);

//      std::cout << "DDFMMK\n";
//      std::cout << "  xm: " << xm << '\n';
//      std::cout << "  xe: " << xe << '\n';

//      std::cout << "  hm: " << hm << '\n';
//      std::cout << "  he: " << he << '\n';

//      std::cout << "  vm: " << vm << '\n';
//      std::cout << "  ve: " << ve << '\n';

//      std::cout << std::endl;

      return ve;
    }



    // Maybe it could be usefull to specify the return type; explicitly
    // This version enforces a return type equal to the return type of the
    // Fn_T kernel
    template <std::regular_invocable<typename MappingObject_T::Type> FnM_T,
              std::regular_invocable<typename EmbedObject_T::DPoint> FnE_T>
    //
    typename EmbedObject_T::VectorH
    //
    operator()(typename MappingObject_T::Type xm,
               typename MappingObject_T::Type hm, FnM_T&& fnm,
               FnE_T&& fne) const requires
      // < -- here is the clue
      gmc::ParametricCurve<MappingObject_T>and
      //
      gmc::PolygonalSurface<EmbedObject_T>and
      // -- >
      std::same_as<std::invoke_result_t<FnE_T, typename EmbedObject_T::DPoint>,
                   typename EmbedObject_T::VectorH>
    {
      typename EmbedObject_T::DVector const xe
        = blaze::subvector<0, EmbedObject_T::DVectorDim>(fnm(xm));
      auto const vm = op_m(xm, hm, std::forward<FnM_T>(fnm));

      /*! \todo Look at this computation it might be dangerous */
      auto const he
        = blaze::normalize(blaze::subvector<0, EmbedObject_T::DVectorDim>(vm))
          * hm;
      /* todo END */

//      static constexpr NumericDifference<FDKernel_T> op_e{};
      static constexpr FDKernel_T op_e{};
      auto const                                     ve
        = op_e(xe, he, std::forward<FnE_T>(fne)) * blaze::length(vm);

//      std::cout << "DDFMMK\n";
//      std::cout << "  xm: " << xm << '\n';
//      std::cout << "  xe: " << xe << '\n';

//      std::cout << "  hm: " << hm << '\n';
//      std::cout << "  he: " << he << '\n';

//      std::cout << "  vm: " << vm << '\n';
//      std::cout << "  ve: " << ve << '\n';

//      std::cout << std::endl;

      return ve;
    }




    // Maybe it could be usefull to specify the return type; explicitly
    // This version enforces a return type equal to the return type of the
    // Fn_T kernel
    template <std::regular_invocable<typename MappingObject_T::DPoint> FnM_T,
              std::regular_invocable<typename EmbedObject_T::DPoint>   FnE_T>
    //
    typename EmbedObject_T::VectorH
    //
    operator()(typename MappingObject_T::DPoint  xm,
               typename MappingObject_T::DVector hm, FnM_T&& fnm,
               FnE_T&& fne) const requires
      // < -- here is the clue
      gmc::TensorProductSurface<MappingObject_T>and
      //
      gmc::TensorProductSurface<EmbedObject_T>and
      // -- >
      std::same_as<std::invoke_result_t<FnE_T, typename EmbedObject_T::DPoint>,
                   typename EmbedObject_T::VectorH>
    {
      typename EmbedObject_T::DVector const xe
        = blaze::subvector<0, EmbedObject_T::DVectorDim>(fnm(xm));
      auto const vm = op_m(xm, hm, std::forward<FnM_T>(fnm));

      /*! \todo Look at this computation it might be dangerous */
      const auto he
        = blaze::normalize(blaze::subvector<0, EmbedObject_T::DVectorDim>(vm))
          * blaze::length(hm);
      /* todo END */

//      static constexpr NumericDifference<FDKernel_T> op_e{};
      static constexpr FDKernel_T op_e{};
      auto const                                     ve
        = op_e(xe, he, std::forward<FnE_T>(fne)) * blaze::length(vm);

//      std::cout << "DDFMMK\n";
//      std::cout << "  xm: " << xm << '\n';
//      std::cout << "  xe: " << xe << '\n';

//      std::cout << "  hm: " << hm << '\n';
//      std::cout << "  he: " << he << '\n';

//      std::cout << "  vm: " << vm << '\n';
//      std::cout << "  ve: " << ve << '\n';

//      std::cout << std::endl;

      return ve;
    }




//    // Maybe it could be usefull to specify the return type; explicitly
//    // This version enforces a return type equal to the return type of the
//    // Fn_T kernel
//    template <std::regular_invocable<typename MappingObject_T::Type> FnM_T,
//              std::regular_invocable<typename EmbedObject_T::DPoint> FnE_T>
//    //
//    typename EmbedObject_T::VectorH
//    //
//    operator()(typename MappingObject_T::Type xm,
//               typename MappingObject_T::Type hm, FnM_T&& fnm,
//               FnE_T&& fne) const requires
//      // < -- here is the clue
//      gmc::ParametricCurve<MappingObject_T>and
//      //
//      gmc::MappingMappingObject<EmbedObject_T>and // <<<<<< ------- HERE
//      // -- >
//      std::same_as<std::invoke_result_t<FnE_T, typename EmbedObject_T::DPoint>,
//                   typename EmbedObject_T::VectorH>
//    {
//      typename EmbedObject_T::DVector const xe
//        = blaze::subvector<0, EmbedObject_T::DVectorDim>(fnm(xm));
//      auto const vm = blaze::eval(op_m(xm, hm, std::forward<FnM_T>(fnm)));
//      const auto he
//        = blaze::normalize(blaze::subvector<0, EmbedObject_T::DVectorDim>(vm))
//          * hm;
//      static constexpr NumericDifference<FDKernel_T> op_e{}; // <--- RECURSE
//      auto const ve = blaze::eval(op_e(xe, he, std::forward<FnE_T>(fne))
//                                  * blaze::length(vm));
//      return ve;
//    }

  private:
//    static constexpr NumericDifference<FDKernel_T> op_m{};
    static constexpr FDKernel_T op_m{};
  };





}   // namespace gm::parametric::numericdiff




#endif   // GM2_PARAMETRIC_UTILS_NUMERIC_DIFFERENTIATION_H
