#ifndef _DISTANCE_MAP_CORE_DISTANCE_FIELD_GRID_H_
#define _DISTANCE_MAP_CORE_DISTANCE_FIELD_GRID_H_

#include <cmath>
#include <stdexcept>
#include <memory>

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
    double dx, dy;
  };

  DistanceFieldGrid(const Dimension& dimension,
                    const double resolution = 1,
                    const Origin& origin = Origin());

  ~DistanceFieldGrid();

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
  double atPosition(const double x, const double y) const;

  /**
   * @brief atPositionSafe, if position is out of bound, return 0 (obstacle).
   * @param x
   * @param y
   * @return The distance to the closest black (0) cell, in m unit.
   */
  double atPositionSafe(const double x, const double y) const;

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
  Gradient gradientAtPosition(const double x, const double y) const;

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
  Gradient gradientAtPositionSafe(const double x, const double y) const;

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

void DistanceFieldGrid::resize(const std::size_t rows, const std::size_t cols)
{  
  if (rows*cols != dimension_.height*dimension_.width)
  {
    if (initialized_)
      delete data_;

    data_ = new double[rows*cols];
  }
  dimension_ = Dimension(cols, rows);
  initialized_ = true;
}

bool DistanceFieldGrid::isCellValid(const std::size_t row, const std::size_t col) const noexcept
{
  return (row < dimension_.height) && (col < dimension_.width);
}

bool DistanceFieldGrid::isPositionValid(const double x, const double y) const noexcept
{
  std::size_t row, col;
  positionToCell(x,y,row,col);

  return isCellValid(row, col);
}

void DistanceFieldGrid::cellToPosition(const std::size_t row,
                                       const std::size_t col,
                                       double& x, double& y) const
{
  /// To cartesian coor
  const double x_corner =  col;
  const double y_corner = -row + (dimension_.height-1);

  /* Sim2 * p */
  const double cos_yaw  = std::cos(origin_.yaw) * resolution_;
  const double sin_yaw  = std::sin(origin_.yaw) * resolution_;
  x = cos_yaw * x_corner - sin_yaw * y_corner + origin_.x;
  y = sin_yaw * x_corner + cos_yaw * y_corner + origin_.y;

//  std::cout << "cell: " << row << "," << col
//            << " (" << x_corner << "," << y_corner << ")"
//            << " to position:"
//            << x << "," << y << "\n";
}

void DistanceFieldGrid::positionToCell(const double x, const double y,
                                       std::size_t& row, std::size_t& col) const
{
  /* Sim2^-1 * p */
  // R^-1
  double cos_yaw = std::cos(origin_.yaw) * resolution_;
  double sin_yaw = std::sin(origin_.yaw) * resolution_;
  const double sq = cos_yaw*cos_yaw + sin_yaw*sin_yaw;
  cos_yaw =  cos_yaw / sq;
  sin_yaw = -sin_yaw / sq;

  // t^-1
  const double xo = -(cos_yaw * origin_.x - sin_yaw * origin_.y);
  const double yo = -(sin_yaw * origin_.x + cos_yaw * origin_.y);

  const double x_corner = ((cos_yaw * x - sin_yaw * y) + xo);
  const double y_corner = ((sin_yaw * x + cos_yaw * y) + yo);

  /// Convert to grid index
  /// @note Forces 0.5 to round up
  constexpr double eps = 1e-8;
  row = static_cast<std::size_t>(-y_corner + (dimension_.height-1) + eps);
  col = static_cast<std::size_t>( x_corner + eps);

//  std::cout << "position: " << x << "," << y
//            << "(" << x_corner << "," << y_corner << ")"
//            << " to cell:"
//            << row << "(<" << dimension_.height << ")"
//            << ","
//            << col << "(<" << dimension_.width << ")" << "\n";
}

double DistanceFieldGrid::atCell(const std::size_t row, const std::size_t col) const
{
  assertIsValidCell(row, col);
  return data_[getIndex(row, col)];
}

double DistanceFieldGrid::atCellSafe(const std::size_t row, const std::size_t col) const
{
  /// @todo return -dist_to_in_bound ?
  return isCellValid(row,col)? data_[getIndex(row,col)] : 0;
}

double DistanceFieldGrid::atPosition(const double x, const double y) const
{
  assertIsValidPosition(x,y);

  std::size_t row, col;
  positionToCell(x,y,row,col);

  return atCell(row,col) * resolution_;

//  const double lx = std::floor(x),
//               ly = std::floor(y);

//  const double hx = lx + resolution_,
//               hy = ly + resolution_;

//  std::size_t lxi,lyi,hxi,hyi;
//  positionToCell(lx,ly,lxi,lyi);

//  isPositionValid(hx, hy)?
//    positionToCell(hx,hy,hxi,hyi)    :
//    positionToCell(hx-resolution_,
//                   hy-resolution_,
//                   hxi,hyi);

//  std::cout << x << "," << y << "\n";
//  std::cout << lxi << "," << lyi << "\n";
//  std::cout << hxi << "," << hyi << "\n";
//  std::cout << (hx-x)*(hy-y) << "," << (x-lx)*(hy-y) << ","
//            << (hx-x)*(y-ly) << "," << (x-lx)*(y-ly) << "\n";
//  std::cout << "Q1 " << (hx+1-x)*(hy+1-y)*atCell(lxi, lyi) << "\n";
//  std::cout << "Q2 " << (x-lx)*(hy+1-y)*atCell(hxi, lyi) << "\n";
//  std::cout << "Q3 " << (hx+1-x)*(y-ly)*atCell(lxi, hyi) << "\n";
//  std::cout << "Q4 " << (x-lx)*(y-ly)*atCell(hxi, hyi) << "\n";

//  return (hx+1-x)*(hy+1-y)*atCell(lxi, lyi) +
//         (x-lx)*(hy+1-y)*atCell(hxi, lyi) +
//         (hx+1-x)*(y-ly)*atCell(lxi, hyi) +
//         (x-lx)*(y-ly)*atCell(hxi, hyi) ;

//  const double lx = std::floor(x),
//               ly = std::floor(y);

//  const double hx = lx + 1,
//               hy = ly + 1;

//  std::size_t lxi,lyi,hxi,hyi;
//  positionToCell(lx,ly,lxi,lyi);

//  if (isCellValid(lxi+1,lyi+1))
//  {
//    hxi = lxi+1;
//    hyi = lyi+1;
//  }
//  else
//  {
//    hxi = lxi;
//    hyi = lyi;
//  }

//  std::cout << x << "," << y << "\n";
//  std::cout << lxi << "," << lyi << "\n";
//  std::cout << hxi << "," << hyi << "\n";
//  std::cout << (hx-x)*(hy-y) << "," << (x-lx)*(hy-y) << ","
//            << (hx-x)*(y-ly) << "," << (x-lx)*(y-ly) << "\n";
//  std::cout << "Q1 " << (hx-x)*(hy-y)*atCell(lxi, lyi) << "\n";
//  std::cout << "Q2 " << (x-lx)*(hy-y)*atCell(hxi, lyi) << "\n";
//  std::cout << "Q3 " << (hx-x)*(y-ly)*atCell(lxi, hyi) << "\n";
//  std::cout << "Q4 " << (x-lx)*(y-ly)*atCell(hxi, hyi) << "\n";

//  return (hx-x)*(hy-y)*atCell(lxi, lyi) +
//         (x-lx)*(hy-y)*atCell(hxi, lyi) +
//         (hx-x)*(y-ly)*atCell(lxi, hyi) +
//         (x-lx)*(y-ly)*atCell(hxi, hyi) ;
}

double DistanceFieldGrid::atPositionSafe(const double x, const double y) const
{
  /// @todo return -dist_to_in_bound ?
  double d = 0;
  if (isPositionValid(x,y))
  {
    d = atPosition(x,y);
  }
  return d;
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

  grad.dx = (atCell(row, col-1) - atCell(row, col+1)) / 2.;
  grad.dy = (atCell(row-1, col) - atCell(row+1, col)) / 2.;

  return grad;
}

DistanceFieldGrid::Gradient
DistanceFieldGrid::gradientAtPosition(double x, double y) const
{
  assertIsValidPosition(x,y);
  assertIsValidPosition(x+1,y+1);

  const double lx = std::floor(x),
               ly = std::floor(y);
  const double hx = lx + 1.0,
               hy = ly + 1.0;

  Gradient grad;

  grad.dx = (hy-y) * (atPosition(hx, ly)-atPosition(lx, ly)) +
            (y-ly) * (atPosition(hx, hy)-atPosition(lx, hy));

  grad.dy = (hx-x) * (atPosition(lx, hy)-atPosition(lx, ly)) +
            (x-lx) * (atPosition(hx, hy)-atPosition(hx, ly));

  return grad;
}

DistanceFieldGrid::Gradient
DistanceFieldGrid::gradientAtCellSafe(std::size_t row, std::size_t col) const
{
  Gradient grad;
  if (isCellValid(row,col))
  {
    grad = gradientAtCell(row, col);
  }
  else
  {
    grad.dx = (row>dimension_.height-2)? row-dimension_.height-2 : 0;
    grad.dy = (col>dimension_.width -2)? col-dimension_.width -2 : 0;
  }

  return grad;
}

DistanceFieldGrid::Gradient
DistanceFieldGrid::gradientAtPositionSafe(const double x, const double y) const
{
  const double lx = std::floor(x),
               ly = std::floor(y);
  const double hx = lx + 1.0,
               hy = ly + 1.0;
  Gradient grad;

  grad.dx = (hy-y) * (atPositionSafe(hx, ly)-atPositionSafe(lx, ly)) +
            (y-ly) * (atPositionSafe(hx, hy)-atPositionSafe(lx, hy));
  grad.dy = (hx-x) * (atPositionSafe(lx, hy)-atPositionSafe(lx, ly)) +
            (x-lx) * (atPositionSafe(hx, hy)-atPositionSafe(hx, ly));

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
  return row * dimension_.width + col;
}

void DistanceFieldGrid::assertIsValidCell(const std::size_t& row, const std::size_t& col) const
{
  if (!isCellValid(row,col))
    throw std::out_of_range("Cell index " + std::to_string(row) +
                            ", " + std::to_string(col) + " is out of range !\n"
                            "Origin ["  + std::to_string(origin_.x) + ","
                                        + std::to_string(origin_.y) + ","
                                        + std::to_string(origin_.yaw) + "], "
                            "Dimension [" + std::to_string(dimension_.width) + ","
                                          + std::to_string(dimension_.height) + "],"
                            "Resolution " + std::to_string(resolution_) + ".");
}

void DistanceFieldGrid::assertIsValidPosition(const double& x, const double& y) const
{
  if (!isPositionValid(x,y))
    throw std::out_of_range("Position " + std::to_string(x) +
                            ", " + std::to_string(y) + " is out of range !\n"
                            "Origin ["  + std::to_string(origin_.x) + ","
                                        + std::to_string(origin_.y) + ","
                                        + std::to_string(origin_.yaw) + "], "
                            "Dimension [" + std::to_string(dimension_.width) + ","
                                          + std::to_string(dimension_.height) + "],"
                            "Resolution " + std::to_string(resolution_) + ".");
}

template <typename Stream>
Stream& operator<<(Stream& s, const DistanceFieldGrid& g)
{
  s << "DistanceFieldGrid:\n"
    << "\tdimension: " << g.getDimension().width << "x" << g.getDimension().height << "\n"
    << "\tresolution: " << g.getResolution() << "\n"
    << "\torigin: " << g.getOrigin().x << ", "
                    << g.getOrigin().y << ", "
                    << g.getOrigin().yaw
    << "\n";

  return s;
}

} /* namespace distmap */

#endif /* _DISTANCE_MAP_CORE_DISTANCE_FIELD_GRID_H_ */
