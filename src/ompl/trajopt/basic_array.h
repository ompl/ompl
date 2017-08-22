#pragma once
#include <vector>

namespace util {

template<class T>
struct BasicArray {

  int m_nRow;
  int m_nCol;
  std::vector<T> m_data;

  BasicArray() :
      m_nRow(0), m_nCol(0) {
  }
  BasicArray(int nRow, int nCol) :
      m_nRow(nRow), m_nCol(nCol) {
    m_data.resize(m_nRow * m_nCol);
  }
  BasicArray(int nRow, int nCol, const T* data) :
    m_nRow(nRow), m_nCol(nCol), m_data(data, data+nRow*nCol) {
  }
  BasicArray(const BasicArray& x) :
      m_nRow(x.m_nRow), m_nCol(x.m_nCol), m_data(x.m_data) {
  }
  void resize(int nRow, int nCol) {
    m_nRow = nRow;
    m_nCol = nCol;
    m_data.resize(m_nRow * m_nCol);
  }

  int rows() const {
    return m_nRow;
  }
  int cols() const {
    return m_nCol;
  }
  int size() const {
    return m_data.size();
  }
  BasicArray block(int startRow, int startCol, int nRow, int nCol) const {
    BasicArray out;
    out.resize(nRow, nCol);
    for (int iRow = 0; iRow < nRow; ++iRow) {
      for (int iCol = 0; iCol < nCol; ++iCol) {
        out(iRow, iCol) = at(iRow + startRow, iCol + startCol);
      }
    }
    return out;
  }
  std::vector<T> rblock(int startRow, int startCol, int nCol) const {
    std::vector<T> out(nCol);
    for (int iCol = 0; iCol < nCol; ++iCol) {
      out[iCol] = at(startRow, iCol + startCol);
    }
    return out;
  }
  BasicArray middleRows(int start, int n) {
    BasicArray out;
    out.resize(n, m_nCol);
    for (int i = start; i < start + n; ++i) {
      for (int j = 0; j < m_nCol; ++j) {
        out(i, j) = at(i, j);
      }
    }
    return out;
  }
  BasicArray topRows(int n) {
    return middleRows(0, n);
  }
  BasicArray bottomRows(int n) {
    return middleRows(m_nRow - n, n);
  }

  const T& at(int row, int col) const {
    return m_data.at(row * m_nCol + col);
  }
  T& at(int row, int col) {
    return m_data.at(row * m_nCol + col);
  }
  const T& operator()(int row, int col) const {
    return m_data.at(row * m_nCol + col);
  }
  T& operator()(int row, int col) {
    return m_data.at(row * m_nCol + col);
  }

  std::vector<T> col(int col) {
    std::vector<T> out;
    out.reserve(m_nRow);
    for (int row = 0; row < m_nRow; row++)
      out.push_back(at(row, col));
    return out;
  }

  std::vector<T> row(int row) {
    std::vector<T> out;
    out.reserve(m_nCol);
    for (int col = 0; col < m_nCol; col++)
      out.push_back(at(row, col));
    return out;
  }

  std::vector<T> flatten() {
    return m_data;
  }

  T* data() {
    return m_data.data();
  }
  T* data() const {
    return m_data.data();
  }

};

}
