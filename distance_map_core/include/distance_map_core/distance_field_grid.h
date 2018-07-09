#ifndef _DISTANCE_MAP_CORE_DISTANCE_FIELD_GRID_H_
#define _DISTANCE_MAP_CORE_DISTANCE_FIELD_GRID_H_

#include <stdexcept>

namespace distmap {

class DistanceFieldGrid
{
public:

  struct Dimension
  {
    Dimension(const std::size_t width, const std::size_t height);
    std::size_t width, height;
  };

  struct Origin
  {
    Origin()  = default;
    ~Origin() = default;

    Origin(const double x, const double y,
           const double yaw);

    double x   = 0,
           y   = 0,
           yaw = 0;
  };

  struct Gradient
  {
    double dx, dy;
  };

  DistanceFieldGrid(const Dimension& dimension,
                    const double resolution,
                    const Origin& origin /*= Origin()*/);

  ~DistanceFieldGrid();

  void resize(const std::size_t row, const std::size_t col);

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
   * @brief atPosition
   * @param x,
   * @param y
   * @return The distance to the closest black (0) cell, in m unit.
   */
  double atPosition(const double x, const double y) const;

  Gradient gradientAtCell(std::size_t row, std::size_t col) const;
  Gradient gradientAtPosition(const double x, const double y) const;

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

  Dimension dimension_;
  double resolution_ = 1;
  Origin origin_;

  double* data_;

  std::size_t getIndex(const std::size_t row, const std::size_t col) const;

  void assertIsValidCell(const std::size_t& row, const std::size_t& col) const;
  void assertIsValidPosition(const double& x, const double& y) const;
};

using DistanceFieldGridPtr = std::shared_ptr<DistanceFieldGrid>;
using DistanceFieldGridConstPtr = std::shared_ptr<const DistanceFieldGrid>;

DistanceFieldGrid::Dimension::Dimension(const std::size_t width, const std::size_t height)
  : width(width), height(height)
{
  if (width == 0)
    throw std::runtime_error("Dimension x can't be zero !");

  if (height == 0)
    throw std::runtime_error("Dimension x can't be zero !");
}

DistanceFieldGrid::Origin::Origin(const double _x, const double _y,
                                  const double _yaw)
  : x(_x), y(_y), yaw(_yaw)
{
  //
}

DistanceFieldGrid::DistanceFieldGrid(const Dimension& dimension,
                                     const double resolution,
                                     const Origin& origin)
  : dimension_(dimension)
  , resolution_(resolution)
  , origin_(origin)
{
  if (resolution_ <= 0)
    throw std::runtime_error("Resolution can't be zero nor negative !");

  data_ = new double[dimension_.width*dimension_.height];
  initialized_ = true;
}

DistanceFieldGrid::~DistanceFieldGrid()
{
  if (initialized_)
    delete data_;
}

void DistanceFieldGrid::resize(const std::size_t row, const std::size_t col)
{
  if (initialized_)
    delete data_;

  dimension_ = Dimension(row, col);
  data_ = new double[dimension_.width*dimension_.height];
  initialized_ = true;
}

bool DistanceFieldGrid::isCellValid(const std::size_t row, const std::size_t col) const noexcept
{
  return (row < dimension_.width) && (col < dimension_.height);
}

bool DistanceFieldGrid::isPositionValid(const double x, const double y) const noexcept
{
  if (x < origin_.x || y < origin_.y)
    return false;

  std::size_t row, col;
  positionToCell(x,y,row,col);

  return isCellValid(row, col);
}

void DistanceFieldGrid::cellToPosition(const std::size_t row,
                                       const std::size_t col,
                                       double& x, double& y) const
{
  x = /*R*/ double(row) * resolution_ + origin_.x;
  y = /*R*/ double(col) * resolution_ + origin_.y;
}

void DistanceFieldGrid::positionToCell(const double x, const double y,
                                       std::size_t& row, std::size_t& col) const
{
  row = /*R^-1*/ (x - origin_.x) / resolution_;
  col = /*R^-1*/ (y - origin_.y) / resolution_;
}

double DistanceFieldGrid::atCell(const std::size_t row, const std::size_t col) const
{
  assertIsValidCell(row, col);
  return data_[getIndex(row, col)];
}

double DistanceFieldGrid::atPosition(const double x, const double y) const
{
  if (x < origin_.x || y < origin_.y)
    throw std::out_of_range("Position is out of range !");

  std::size_t row, col;
  positionToCell(x,y,row,col);

  return atCell(row,col) * resolution_;
}

DistanceFieldGrid::Gradient
DistanceFieldGrid::gradientAtCell(std::size_t row, std::size_t col) const
{
  assertIsValidCell(row, col);

  // handle borders
  row = std::max(row, std::size_t(1));
  col = std::max(col, std::size_t(1));

  row = std::min(row, dimension_.height-2);
  col = std::min(col, dimension_.width -2);

  Gradient grad;

  grad.dx = (atCell(row-1, col) - atCell(row+1, col)) / 2.;
  grad.dy = (atCell(row, col-1) - atCell(row, col+1)) / 2.;

  return grad;
}

DistanceFieldGrid::Gradient
DistanceFieldGrid::gradientAtPosition(const double x, const double y) const
{
  assertIsValidPosition(x,y);

  std::size_t row, col;
  positionToCell(x,y,row,col);

  auto grad = gradientAtCell(row,col);

  grad.dx *= resolution_;
  grad.dy *= resolution_;

  return grad;
}

// Setter/getter

inline double* DistanceFieldGrid::data()
{
  return data_;
}

inline const double* DistanceFieldGrid::data() const
{
  return data_;
}

inline void DistanceFieldGrid::setOrigin(const Origin& origin)
{
  origin_ = origin;
}

inline void DistanceFieldGrid::setResolution(const double resolution)
{
  if (resolution <= 0)
    throw std::runtime_error("Resolution can't be zero nor negative !");

  resolution_ = resolution;
}

inline const DistanceFieldGrid::Dimension&
DistanceFieldGrid::getDimension() const noexcept
{
  return dimension_;
}

inline const double&
DistanceFieldGrid::getResolution() const noexcept
{
  return resolution_;
}

inline const DistanceFieldGrid::Origin&
DistanceFieldGrid::getOrigin() const noexcept
{
  return origin_;
}

std::size_t DistanceFieldGrid::getIndex(const std::size_t row, const std::size_t col) const
{
  return col + (dimension_.width - row - 1) * dimension_.height;
}

void DistanceFieldGrid::assertIsValidCell(const std::size_t& row, const std::size_t& col) const
{
  if (!isCellValid(col,row))
    throw std::out_of_range("Cell index is out of range !");
}

void DistanceFieldGrid::assertIsValidPosition(const double& x, const double& y) const
{
  if (!isPositionValid(x,y))
    throw std::out_of_range("Position is out of range !");
}

} /* namespace distmap */

#endif /* _DISTANCE_MAP_CORE_DISTANCE_FIELD_GRID_H_ */
