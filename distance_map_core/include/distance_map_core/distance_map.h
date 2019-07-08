/*
 * Copyright 2019 Jeremie Deray
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Jeremie Deray
 */

#ifndef _DISTANCE_MAP_CORE_DISTANCE_MAP_H_
#define _DISTANCE_MAP_CORE_DISTANCE_MAP_H_

#include <cmath>
#include <stdexcept>
#include <memory>

namespace distmap {

class DistanceMap
{
  /// Used to convert to grid index
  /// @note Forces 0.5 to round up
  static constexpr double eps = 1e-8;

public:

  /// @todo Use Scalar to 'easily' switch
  /// between floats & doubles
  //using Scalar = float;

  struct Dimension
  {
    Dimension(const std::size_t width, const std::size_t height);
    std::size_t width = 0, height = 0;
  };

  struct Origin
  {
    // see https://stackoverflow.com/a/17436088/9709397
    Origin()  {}//= default;
    ~Origin() = default;

    Origin(const double x, const double y,
           const double yaw);

    double x   = 0,
           y   = 0,
           yaw = 0;
  };

  struct Gradient
  {
    double dx = 0, dy = 0;

    Gradient operator*(double w)
    {
      Gradient g = *this;
      g.dx *= w; g.dy *= w;
      return g;
    }
  };

  DistanceMap(const Dimension& dimension,
                    const double resolution = 1,
                    const Origin& origin = Origin());

  DistanceMap(const DistanceMap& grid);
  DistanceMap(DistanceMap&& grid);

  DistanceMap& operator=(const DistanceMap& grid);
  DistanceMap& operator=(DistanceMap&& grid);

  ~DistanceMap();

  void resize(const std::size_t rows, const std::size_t cols);

  bool isCellValid(const std::size_t row, const std::size_t col) const noexcept;
  bool isPositionValid(const double x, const double y) const noexcept;

  void cellToPosition(const std::size_t row, const std::size_t col,
                      double& x, double& y) const;

  void positionToCell(const double x, const double y,
                      std::size_t& row, std::size_t& col) const;

  /**
   * @brief atCell
   * @param row
   * @param col
   * @return The distance to the closest black (0) cell, in cell unit.
   */
  double atCell(const std::size_t row, const std::size_t col) const;

  /**
   * @brief atCellSafe, if cell is out of bound, return 0 (obstacle).
   * @param row
   * @param col
   * @return The distance to the closest black (0) cell, in cell unit.
   */
  double atCellSafe(const std::size_t row, const std::size_t col) const;

  /**
   * @brief atPosition
   * @param x,
   * @param y
   * @return The distance to the closest black (0) cell, in m unit.
   */
  double atPosition(const double x, const double y,
                    const bool interpolate = false) const;

  /**
   * @brief atPositionSafe, if position is out of bound, return 0 (obstacle).
   * @param x
   * @param y
   * @return The distance to the closest black (0) cell, in m unit.
   */
  double atPositionSafe(const double x, const double y,
                        const bool interpolate = false) const;

  /**
   * @brief gradientAtCell
   * @param row
   * @param col
   * @return
   */
  Gradient gradientAtCell(std::size_t row, std::size_t col) const;

  /**
   * @brief gradientAtPosition
   * @param x
   * @param y
   * @return
   */
  Gradient gradientAtPosition(const double x, const double y,
                              const bool interpolate = false) const;

  /**
   * @brief gradientAtCellSafe
   * @param row
   * @param col
   * @return
   */
  Gradient gradientAtCellSafe(std::size_t row, std::size_t col) const;

  /**
   * @brief gradientAtPositionSafe
   * @param x
   * @param y
   * @return
   */
  Gradient gradientAtPositionSafe(const double x, const double y,
                                  const bool interpolate = false) const;

  // Setter/getter

  double* data();
  const double* data() const;

  //void setDimension(const Dimension& dimension);
  void setOrigin(const Origin& origin);
  void setResolution(const double resolution);

  const Dimension& getDimension()  const noexcept;
  const double&    getResolution() const noexcept;
  const Origin&    getOrigin()     const noexcept;

protected:

  bool initialized_ = false;

  Dimension dimension_; ///< @brief Dimension of the grid in cells.
  double resolution_ = 1;
  Origin origin_; ///< @brief The 2-D pose of the bottom-left pixel in the map.

  /// @brief 2D grid data. Col-major.
  double* data_;

  std::size_t getIndex(const std::size_t row, const std::size_t col) const;

  void assertIsValidCell(const std::size_t& row, const std::size_t& col) const;
  void assertIsValidPosition(const double& x, const double& y) const;
};

using DistanceFieldGridPtr = std::shared_ptr<DistanceMap>;
using DistanceFieldGridConstPtr = std::shared_ptr<const DistanceMap>;

inline double* DistanceMap::data()
{
  return data_;
}

inline const double* DistanceMap::data() const
{
  return data_;
}

inline void DistanceMap::setOrigin(const Origin& origin)
{
  origin_ = origin;
}

inline void DistanceMap::setResolution(const double resolution)
{
  if (resolution <= 0)
    throw std::runtime_error("Resolution can't be zero nor negative !");

  resolution_ = resolution;
}

inline const DistanceMap::Dimension&
DistanceMap::getDimension() const noexcept
{
  return dimension_;
}

inline const double&
DistanceMap::getResolution() const noexcept
{
  return resolution_;
}

inline const DistanceMap::Origin&
DistanceMap::getOrigin() const noexcept
{
  return origin_;
}

inline std::size_t
DistanceMap::getIndex(const std::size_t row, const std::size_t col) const
{
  return row * dimension_.width + col;
}

template <typename Stream>
Stream& operator<<(Stream& s, const DistanceMap& g)
{
  s << "DistanceMap:\n"
    << "\tdimension: " << g.getDimension().width << "x" << g.getDimension().height << "\n"
    << "\tresolution: " << g.getResolution() << "\n"
    << "\torigin: " << g.getOrigin().x << ", "
                    << g.getOrigin().y << ", "
                    << g.getOrigin().yaw
    << "\n";

  return s;
}

} // namespace distmap

#endif // _DISTANCE_MAP_CORE_DISTANCE_MAP_H_
