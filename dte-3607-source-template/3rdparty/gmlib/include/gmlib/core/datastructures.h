#ifndef GM2_CORE_DATASTRUCTURES_H
#define GM2_CORE_DATASTRUCTURES_H

#include "gm2_blaze.h"

namespace gm::datastructures
{


  enum class MatrixOrdering : bool { RowMajor = true, ColumnMajor = false };

  template <typename Element_T, MatrixOrdering MO_T = MatrixOrdering::RowMajor>
  struct std_vector2d {
    static constexpr auto MO = MO_T;

    explicit std_vector2d() {}
    explicit std_vector2d(size_t m1, size_t m2) { resize(m1, m2); }

    std_vector2d(const std_vector2d& other) = default;
    std_vector2d(std_vector2d&& other)      = default;
    ~std_vector2d()                         = default;

    std_vector2d& operator=(const std_vector2d& other) = default;
    std_vector2d& operator=(std_vector2d&& other) = default;

    constexpr size_t index(size_t i, size_t j) const
    {
      if constexpr (MO == MatrixOrdering::RowMajor)
        return i * m_columns + j;
      else
        return i * m_rows + j;
    }


    Element_T& operator()(size_t i, size_t j) { return m_data[index(i, j)]; }

    const Element_T& operator()(size_t i, size_t j) const
    {
      return m_data[index(i, j)];
    }

    size_t rows() const { return m_rows; }
    size_t columns() const { return m_columns; }

    auto const& data() const { return m_data; }

    void resize(size_t m1, size_t m2)
    {
      m_rows    = MO == MatrixOrdering::RowMajor ? m1 : m2;
      m_columns = MO == MatrixOrdering::RowMajor ? m2 : m1;
      m_data.resize(m1 * m2);
    }

  private:
    size_t                 m_rows{0};
    size_t                 m_columns{0};
    std::vector<Element_T> m_data;
  };






  namespace parametrics
  {

    namespace ppoint
    {
      template <typename Element_T = gm::VectorT<double, 4>>
      using EvaluationResult = Element_T;

      template <typename Element_T = gm::VectorT<double, 4>>
      using SamplingResult = Element_T;

    }   // namespace ppoint


    namespace curve
    {
      template <typename Element_T = gm::VectorT<double, 4>>
      using EvaluationResult = DVectorT<Element_T>;

      template <typename Element_T = gm::VectorT<double, 4>>
      using SamplingResult = std::vector<EvaluationResult<Element_T>>;

    }   // namespace curve


    namespace tensorproductsurface
    {
      template <typename Element_T = gm::VectorT<double, 4>>
      using EvaluationResult = DMatrixT<Element_T>;

      template <typename Element_T = gm::VectorT<double, 4>>
      using SamplingResult = std_vector2d<EvaluationResult<Element_T>>;

    }   // namespace tensorproductsurface


    namespace triangularsurface
    {
      template <typename Element_T = gm::VectorT<double, 4>>
      using EvaluationResult = DVectorT<Element_T>;

      template <typename Element_T = gm::VectorT<double, 4>>
      using SamplingResult = std::vector<EvaluationResult<Element_T>>;

    }   // namespace triangularsurface


    namespace polygonalsurface
    {

      /**
       * \deprecated
       */
      template <typename Element_T = gm::VectorT<double, 4>>
      using EvaluationResult = DMatrixT<Element_T>;

      /**
       * \deprecated
       */
      template <typename Element_T = gm::VectorT<double, 4>>
      using SamplingResult = std::vector<EvaluationResult<Element_T>>;

      /**
       * \todo refactor (rename) on *[-DVec] deprecation
       */
      template <typename Element_T = gm::VectorT<double, 4>>
      using EvaluationResultDVec = DVectorT<Element_T>;

      /**
       * \todo refactor (rename) on *[-DVec] deprecation
       */
      template <typename Element_T = gm::VectorT<double, 4>>
      using SamplingResultDVec = std::vector<EvaluationResultDVec<Element_T>>;


      template <typename Element_T = gm::VectorT<double, 2>>
      using SamplingPoints = std::vector<Element_T>;

    }   // namespace polygonalsurface




  }   // namespace parametrics

}   // namespace gm::datastructures

#endif   // GM2_CORE_DATASTRUCTURES_H
