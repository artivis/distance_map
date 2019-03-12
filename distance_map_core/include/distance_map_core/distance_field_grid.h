#ifndef _DISTANCE_MAP_CORE_DISTANCE_FIELD_GRID_H_
#define _DISTANCE_MAP_CORE_DISTANCE_FIELD_GRID_H_

#include <cmath>
#include <stdexcept>
#include <memory>

namespace distmap {

class DistanceFieldGrid
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

  DistanceFieldGrid(const Dimension& dimension,
                    const double resolution = 1,
                    const Origin& origin = Origin());

  DistanceFieldGrid(const DistanceFieldGrid& grid);
  DistanceFieldGrid(DistanceFieldGrid&& grid);

  DistanceFieldGrid& operator=(const DistanceFieldGrid& grid);
  DistanceFieldGrid& operator=(DistanceFieldGrid&& grid);

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

DistanceFieldGrid::DistanceFieldGrid(const DistanceFieldGrid& grid)
  : dimension_(grid.dimension_)
{
  *this = grid;
}

DistanceFieldGrid::DistanceFieldGrid(DistanceFieldGrid&& grid)
  : dimension_(grid.dimension_)
{
  *this = grid;
}

DistanceFieldGrid& DistanceFieldGrid::operator=(const DistanceFieldGrid& grid)
{
  if (grid.initialized_)
  {
    resolution_ = grid.resolution_;
    origin_     = grid.origin_;

    if (dimension_.width  != grid.dimension_.width ||
        dimension_.height != grid.dimension_.height  ) {
      dimension_  = grid.dimension_;
      data_ = new double[dimension_.width*dimension_.height];
    }

    std::copy(grid.data_, grid.data_+(dimension_.width*dimension_.height), data_);

    initialized_ = grid.initialized_;
  }

  return *this;
}

DistanceFieldGrid& DistanceFieldGrid::operator=(DistanceFieldGrid&& grid)
{
  if (grid.initialized_)
  {
    resolution_ = grid.resolution_;
    origin_     = grid.origin_;

    if (dimension_.width  != grid.dimension_.width ||
        dimension_.height != grid.dimension_.height  ) {
      dimension_  = grid.dimension_;
      data_ = new double[dimension_.width*dimension_.height];
    }

    std::copy(grid.data_, grid.data_+(dimension_.width*dimension_.height), data_);

    grid.dimension_ = Dimension(1,1);
    grid.resolution_ = 1;
    grid.origin_ = Origin();
    delete grid.data_;
    grid.data_ = NULL;

    initialized_ = grid.initialized_;
    grid.initialized_ = false;
  }

  return *this;
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

  row = static_cast<std::size_t>(-y_corner + (dimension_.height-1) + eps);
  col = static_cast<std::size_t>( x_corner + eps);
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

double DistanceFieldGrid::atPosition(const double x, const double y,
                                     const bool interpolate) const
{
  assertIsValidPosition(x,y);

  if (!interpolate)
  {
    std::size_t row, col;
    positionToCell(x,y,row,col);

    return atCell(row,col) * resolution_;
  }

  // bilinear interpolation

  /// @todo we're interpolating on the grid,
  /// what if theta_grid != theta_origin ?

  std::size_t lxi, lyi;
  positionToCell(x,y,lyi,lxi);
  std::size_t hxi=lxi+1, hyi=lyi+1;

  double lx, ly;
  cellToPosition(lyi,lxi,lx,ly);

  double hx, hy;
  cellToPosition(hyi,hxi,hx,hy);

  if ((hx==lx && hy==ly)    ||
      !isCellValid(lyi,lxi) ||
      !isCellValid(lyi,hxi) ||
      !isCellValid(hyi,lxi) ||
      !isCellValid(hyi,hxi)   )
  {
    std::size_t row, col;
    positionToCell(x,y,row,col);

    return atCell(row,col) * resolution_;
  }

  const double w = 1./ (hx-lx)*(hy-ly);

  const double a=(hx-x)*w;
  const double b=(x-lx)*w;

  const double q11=atCell(lyi,lxi);
  const double q12=atCell(lyi,hxi);
  const double q21=atCell(hyi,lxi);
  const double q22=atCell(hyi,hxi);

  const double g=q11*a+q21*b;
  const double h=q12*a+q22*b;

  return (g*(hy-y)+h*(y-ly))/resolution_;
}

double DistanceFieldGrid::atPositionSafe(const double x, const double y,
                                         const bool interpolate) const
{
  /// @todo return -dist_to_in_bound ?
  double d = 0;
  if (isPositionValid(x,y))
  {
    d = atPosition(x,y,interpolate);
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

  const double dhdx = atCell(row, col+1);
  const double dldx = atCell(row, col-1);

  const double dhdy = atCell(row+1, col);
  const double dldy = atCell(row-1, col);

//  grad.dx = (dldx - dhdx) / 2.;
//  grad.dy = (dhdy - dldy) / 2.;
  grad.dx = (dhdx - dldx) / 2.;
  grad.dy = (dldy - dhdy) / 2.;

  return grad;
}

DistanceFieldGrid::Gradient
DistanceFieldGrid::gradientAtPosition(double x, double y,
                                      const bool interpolate) const
{
  if (!interpolate)
  {
//    std::size_t row, col;
//    positionToCell(x,y,row,col);
//    return gradientAtCell(row,col) * resolution_;

    /// @todo handle borders
    Gradient grad;

    const double dhdx = atPosition(x+resolution_+eps, y);
    const double dldx = atPosition(x-resolution_-eps, y);

    const double dhdy = atPosition(x, y+resolution_+eps);
    const double dldy = atPosition(x, y-resolution_-eps);

    grad.dx = (dhdx - dldx) / 2.;
    grad.dy = (dhdy - dldy) / 2.;

    return grad;
  }

  // bilinear interpolation

  /// @todo we're differentiating on the grid,
  /// what if theta_grid != theta_origin ?
  /// grad_pos = R.grad_cell ?

  std::size_t lxi, lyi;
  positionToCell(x,y,lyi,lxi);
  std::size_t hxi=lxi+1, hyi=lyi+1;

  double lx, ly;
  cellToPosition(lyi,lxi,lx,ly);

  double hx, hy;
  cellToPosition(hyi,hxi,hx,hy);

  if ((hx==lx && hy==ly)    ||
      !isCellValid(lyi,lxi) ||
      !isCellValid(lyi,hxi) ||
      !isCellValid(hyi,lxi) ||
      !isCellValid(hyi,hxi)   )
  {
    return gradientAtPosition(x,y,false);
  }

  const double w = 1./ (hx-lx)*(hy-ly);

  const double a=(hx-x)*w;
  const double b=(x-lx)*w;

  const auto q11=gradientAtCell(lyi,lxi);
  const auto q12=gradientAtCell(lyi,hxi);
  const auto q21=gradientAtCell(hyi,lxi);
  const auto q22=gradientAtCell(hyi,hxi);

  Gradient grad;
  {
    const double g=q11.dx*a+q21.dx*b;
    const double h=q12.dx*a+q22.dx*b;

    grad.dx = (g*(hy-y)+h*(y-ly));
  }

  {
    const double g=q11.dy*a+q21.dy*b;
    const double h=q12.dy*a+q22.dy*b;

    grad.dy = (g*(hy-y)+h*(y-ly));
  }

  return grad*(1./resolution_);

  //////////////////////////////////////

//  grad.dx = (atPosition(x+resolution_+eps, y) - atPosition(x-resolution_-eps, y)) / 2.;
//  grad.dy = (atPosition(x, y+resolution_+eps) - atPosition(x, y-resolution_-eps)) / 2.;

//  const double lx = x-resolution_-eps, ly = y-resolution_-eps;
//  const double hx = x+resolution_+eps, hy = y-resolution_-eps;

//  const double w = 1./ (hx-lx)*(hy-ly);

//  const double a=(hx-x)*w;
//  const double b=(x-lx)*w;

//  const double q11=atPositionSafe(ly,lx);
//  const double q12=atPositionSafe(ly,hx);
//  const double q21=atPositionSafe(hy,lx);
//  const double q22=atPositionSafe(hy,hx);

//  const double g=q11*a+q21*b;
//  const double h=q12*a+q22*b;

//  const double d=(g*(hy-y)+h*(y-ly));

  return grad;
}

DistanceFieldGrid::Gradient
DistanceFieldGrid::gradientAtCellSafe(std::size_t row, std::size_t col) const
{
  Gradient grad;
  if (isCellValid(row-1,col-1) && isCellValid(row+1,col+1))
  {
    grad = gradientAtCell(row, col);
  }
  else
  {
    /// @todo gradient to in bounds
    grad.dx = 0;
    grad.dy = 0;
  }

  return grad;
}

DistanceFieldGrid::Gradient
DistanceFieldGrid::gradientAtPositionSafe(const double x, const double y,
                                          const bool interpolate) const
{
  Gradient grad;
  if (isPositionValid(x-resolution_-eps,y-resolution_-eps) &&
      isPositionValid(x+resolution_+eps,y+resolution_+eps)   )
  {
    grad = gradientAtPosition(x,y,interpolate);
  }
  else
  {
    // @todo gradient to in bounds
    grad.dx = 0;
    grad.dy = 0;
  }

  return grad;

//  if (!interpolate)
//    return gradientAtCellSafe(row,col) * resolution_;

//  // bilinear interpolation

////  std::size_t lxi, lyi;
////  positionToCell(x,y,lyi,lxi);
////  std::size_t hxi=lxi+1, hyi=lyi+1;

////  double lx, ly;
////  cellToPosition(lyi,lxi,lx,ly);

////  double hx, hy;
////  cellToPosition(hyi,hxi,hx,hy);

//  const double lx = std::floor(x),
//               ly = std::floor(y);
//  const double hx = lx + resolution_ + 1e-8,
//               hy = ly + resolution_ + 1e-8;

//  if ((hx==lx && hy==ly) || !isPositionValid(hx, hy))
//  {
//    return gradientAtCellSafe(row,col) * resolution_;
//  }

//  Gradient grad;

//  grad.dx = (hy-y) * (atPositionSafe(hx, ly, interpolate)-atPositionSafe(lx, ly, interpolate)) +
//            (y-ly) * (atPositionSafe(hx, hy, interpolate)-atPositionSafe(lx, hy, interpolate));

//  grad.dy = (hx-x) * (atPositionSafe(lx, hy, interpolate)-atPositionSafe(lx, ly, interpolate)) +
//            (x-lx) * (atPositionSafe(hx, hy, interpolate)-atPositionSafe(hx, ly, interpolate));

//  return grad;

////  const double lx = std::floor(x),
////               ly = std::floor(y);
////  const double hx = lx + 1.0,
////               hy = ly + 1.0;
////  Gradient grad;

////  grad.dx = (hy-y) * (atPositionSafe(hx, ly)-atPositionSafe(lx, ly)) +
////            (y-ly) * (atPositionSafe(hx, hy)-atPositionSafe(lx, hy));
////  grad.dy = (hx-x) * (atPositionSafe(lx, hy)-atPositionSafe(lx, ly)) +
////            (x-lx) * (atPositionSafe(hx, hy)-atPositionSafe(hx, ly));

////  return grad;
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
