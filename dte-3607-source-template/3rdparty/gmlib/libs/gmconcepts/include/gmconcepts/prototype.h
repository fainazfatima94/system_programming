#ifndef GMC_PROTOTYPE_H
#define GMC_PROTOTYPE_H


namespace gmc::proto {

  template <size_t Order_T, size_t ReqOrder_T>
  concept OrderEqualTo = requires
  {
    requires Order_T == ReqOrder_T;
  };

  template <size_t Order_T, size_t ReqOrderMin_T, size_t ReqOrderMax_T>
  concept OrderBoundTo = requires
  {
    requires Order_T >= ReqOrderMin_T;
    requires Order_T <= ReqOrderMax_T;
  };

  template <size_t Order_T, size_t ReqOrder_T>
  concept OrderAtLeast = requires
  {
    requires Order_T >= ReqOrder_T;
  };




}   // namespace gmc


#endif // GMC_PROTOTYPE_H
