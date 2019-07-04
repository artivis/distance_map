#include "distance_map_core/distance_map.h"

namespace distmap {

DistanceMap::Dimension::Dimension(const std::size_t width, const std::size_t height)
  : width(width), height(height)
{
  if (width == 0)
    throw std::runtime_error("Dimension x can't be zero !");
  if (height == 0)
    throw std::runtime_error("Dimension x can't be zero !");
}

DistanceMap::Origin::Origin(const double _x, const double _y,
                                  const double _yaw)
  : x(_x), y(_y), yaw(_yaw)
{
  //
}

DistanceMap::DistanceMap(const Dimension& dimension,
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

DistanceMap::DistanceMap(const DistanceMap& grid)
  : dimension_(grid.dimension_)
{
  *this = grid;
}

DistanceMap::DistanceMap(DistanceMap&& grid)
  : dimension_(grid.dimension_)
{
  *this = grid;
}

DistanceMap& DistanceMap::operator=(const DistanceMap& grid)
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

DistanceMap& DistanceMap::operator=(DistanceMap&& grid)
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

DistanceMap::~DistanceMap()
{
  if (initialized_)
    delete data_;
}

void DistanceMap::resize(const std::size_t rows, const std::size_t cols)
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

bool DistanceMap::isCellValid(const std::size_t row, const std::size_t col) const noexcept
{
  return (row < dimension_.height) && (col < dimension_.width);
}

bool DistanceMap::isPositionValid(const double x, const double y) const noexcept
{
  std::size_t row, col;
  positionToCell(x,y,row,col);

  return isCellValid(row, col);
}

void DistanceMap::cellToPosition(const std::size_t row,
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

void DistanceMap::positionToCell(const double x, const double y,
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

double DistanceMap::atCell(const std::size_t row, const std::size_t col) const
{
  assertIsValidCell(row, col);
  return data_[getIndex(row, col)];
}

double DistanceMap::atCellSafe(const std::size_t row, const std::size_t col) const
{
  /// @todo return -dist_to_in_bound ?
  return isCellValid(row,col)? data_[getIndex(row,col)] : 0;
}

double DistanceMap::atPosition(const double x, const double y,
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

double DistanceMap::atPositionSafe(const double x, const double y,
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

DistanceMap::Gradient
DistanceMap::gradientAtCell(std::size_t row, std::size_t col) const
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

DistanceMap::Gradient
DistanceMap::gradientAtPosition(double x, double y,
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

DistanceMap::Gradient
DistanceMap::gradientAtCellSafe(std::size_t row, std::size_t col) const
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

DistanceMap::Gradient
DistanceMap::gradientAtPositionSafe(const double x, const double y,
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

void DistanceMap::assertIsValidCell(const std::size_t& row, const std::size_t& col) const
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

void DistanceMap::assertIsValidPosition(const double& x, const double& y) const
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



}
