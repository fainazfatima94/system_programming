#ifndef GM2_PARAMETRIC_UTILS_APPROXIMATION_H
#define GM2_PARAMETRIC_UTILS_APPROXIMATION_H


#include "../../core/divideddifference/finitedifference.h"
#include "../../core/gm2_blaze.h"


namespace gm::parametric::approximation
{

  template <typename FiniteDifferenceOperator_T
            = divideddifference::fd::CentralDifference>
  struct DirectionalDerivativeLocal {

    template <typename ParametricObject_T>
    auto operator()(const ParametricObject_T&                        pobj,
                    const typename ParametricObject_T::PSpacePoint&  pos,
                    const typename ParametricObject_T::PSpaceVector& dir) const
    {
      using PObjEvaluationCtrl = typename ParametricObject_T::PObjEvalCtrl;
      const auto fn            = [&pobj](const auto& par) {
        const auto res = pobj.evaluateLocal(par);
        return blaze::evaluate(PObjEvaluationCtrl::toPosition(res));
      };

      auto res = typename ParametricObject_T::VectorH();
      blaze::subvector<0UL, ParametricObject_T::VectorDim>(res)
        = blaze::evaluate(op(pos, dir, fn));
      res[ParametricObject_T::VectorDim] = typename ParametricObject_T::Unit(0);
      return res;
    }

    static constexpr FiniteDifferenceOperator_T op{};
  };


  template <typename FiniteDifferenceOperator_T
            = divideddifference::fd::CentralDifference>
  struct DirectionalDerivativeParent
    : DirectionalDerivativeLocal<FiniteDifferenceOperator_T> {
  private:
    using Base = DirectionalDerivativeLocal<FiniteDifferenceOperator_T>;

  public:
    template <typename ParametricObject_T>
    auto operator()(
      const ParametricObject_T&                                      pobj,
      const typename std::decay_t<ParametricObject_T>::PSpacePoint&  pos,
      const typename std::decay_t<ParametricObject_T>::PSpaceVector& dir) const
    {
      return blaze::evaluate(pobj.pSpaceFrameParent()
                             * Base::operator()(pobj, pos, dir));
    }
  };


  template <typename FiniteDifferenceOperator_T
            = divideddifference::fd::CentralDifference>
  struct SubObjectDirectionalDerivativeLocal {

    template <typename PSpaceObject_T, typename ParametricObject_T>
    auto
    operator()(const PSpaceObject_T& pspace_obj,
               const typename std::decay_t<PSpaceObject_T>::PSpacePoint&  pos,
               const typename std::decay_t<PSpaceObject_T>::PSpaceVector& dir,
               const ParametricObject_T& pobj) const
    {
      using PSpaceObject      = std::decay_t<PSpaceObject_T>;
      using ParametricObject  = std::decay_t<ParametricObject_T>;
      using PSpaceObjEvalCtrl = typename PSpaceObject::PObjEvalCtrl;

      static_assert(PSpaceObject::VectorDim
                      == ParametricObject::PSpaceVectorDim,
                    "'EmbedSpace dimension of the PSpaceObject must match the "
                    "PSpace dimension of the ParametricObjects' in " __FILE__);

      const auto DpspaceH = pspace_op(pspace_obj, pos, dir);
      const auto Dpspace
        = blaze::subvector<0UL, ParametricObject::PSpaceVectorDim>(DpspaceH);
      const auto Dp
        = op(pobj,
             PSpaceObjEvalCtrl::toPosition(pspace_obj.evaluateParent(
               pos, typename PSpaceObject::PSpaceSizeArray())),
             blaze::normalize(Dpspace) * blaze::length(dir));
      return std::pair(Dp, blaze::length(Dpspace));
    }

    static constexpr DirectionalDerivativeParent<FiniteDifferenceOperator_T>
      pspace_op{};
    static constexpr DirectionalDerivativeLocal<FiniteDifferenceOperator_T>
      op{};
  };


//  template <typename FiniteDifferenceOperator_T
//            = gm::divideddifference::fd::CentralDifference>
//  struct SubObjectDirectionalDerivativeChainLocal {

//    template <typename PSpaceObject_T, typename... ParametricObject_Ts>
//    auto
//    operator()(const PSpaceObject_T& pspace_obj,
//               const typename std::decay_t<PSpaceObject_T>::PSpacePoint&  pos,
//               const typename std::decay_t<PSpaceObject_T>::PSpaceVector& dir,
//               const ParametricObject_Ts&... pobjs) const
//    {
//      // ...
//    }
//    static constexpr DirectionalDerivativeParent<FiniteDifferenceOperator_T>
//                                                                             pspace_op{};
//    static constexpr DirectionalDerivativeParent<FiniteDifferenceOperator_T> ops{};
//    static constexpr DirectionalDerivativeLocal<FiniteDifferenceOperator_T>  op_las{};
//  };

}   // namespace gm::parametric::approximation




#endif   // GM2_PARAMETRIC_UTILS_APPROXIMATION_H
