#ifndef GM2_CORE_SPACES_EUCLEDIANSPACE_H
#define GM2_CORE_SPACES_EUCLEDIANSPACE_H


namespace gm::spaces::eucledianspace {


  template <size_t R_T>
  struct R {
    static auto constexpr Dim = R_T;
  };

  using R1 = R<1ul>;
  using R2 = R<2ul>;
  using R3 = R<3ul>;

}   // namespace gm::spaces::eucledianspace



#endif // GM2_CORE_SPACES_EUCLEDIANSPACE_H
