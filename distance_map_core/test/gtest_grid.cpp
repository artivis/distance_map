#include <gtest/gtest.h>

#include <distance_map_core/distance_field_grid.h>

struct GridTest : public testing::Test
{
  GridTest()
    : default_grid(distmap::DistanceFieldGrid::Dimension(3,3))
    , grid(distmap::DistanceFieldGrid::Dimension(5,5),
           0.5,
           distmap::DistanceFieldGrid::Origin(1.23,3.45,0.17))

  {
    for (int i=0; i<3*3; ++i)
      *(default_grid.data()+i) = i;

    // pattern
    //
    // 1 1 1 2 3
    // 1 0 1 2 3
    // 1 1 1 2 3
    // 2 2 2 2 3
    // 3 3 3 3 3

    *(grid.data()+ 0) = 1;
    *(grid.data()+ 1) = 1;
    *(grid.data()+ 2) = 1;
    *(grid.data()+ 3) = 2;
    *(grid.data()+ 4) = 3;

    *(grid.data()+ 5) = 1;
    *(grid.data()+ 6) = 0;
    *(grid.data()+ 7) = 1;
    *(grid.data()+ 8) = 2;
    *(grid.data()+ 9) = 3;

    *(grid.data()+10) = 1;
    *(grid.data()+11) = 1;
    *(grid.data()+12) = 1;
    *(grid.data()+13) = 2;
    *(grid.data()+14) = 3;

    *(grid.data()+15) = 2;
    *(grid.data()+16) = 2;
    *(grid.data()+17) = 2;
    *(grid.data()+18) = 2;
    *(grid.data()+19) = 3;

    *(grid.data()+20) = 3;
    *(grid.data()+21) = 3;
    *(grid.data()+22) = 3;
    *(grid.data()+23) = 3;
    *(grid.data()+24) = 3;
  }

  distmap::DistanceFieldGrid default_grid;
  distmap::DistanceFieldGrid grid;
};

TEST_F(GridTest, TEST_GRID_DEFAULT)
{
  EXPECT_EQ(default_grid.getDimension().height, 3);
  EXPECT_EQ(default_grid.getDimension().width,  3);
  EXPECT_EQ(default_grid.getOrigin().x,   0);
  EXPECT_EQ(default_grid.getOrigin().y,   0);
  EXPECT_EQ(default_grid.getOrigin().yaw, 0);
  EXPECT_EQ(default_grid.getResolution(), 1);

  ASSERT_THROW(default_grid.setResolution(0),  std::runtime_error);
  ASSERT_THROW(default_grid.setResolution(-1), std::runtime_error);

  EXPECT_TRUE(default_grid.isCellValid(0,0));
  EXPECT_TRUE(default_grid.isCellValid(1,1));
  EXPECT_TRUE(default_grid.isCellValid(2,2));

  EXPECT_FALSE(default_grid.isCellValid(3,3));
  EXPECT_FALSE(default_grid.isCellValid(0,3));
  EXPECT_FALSE(default_grid.isCellValid(3,0));

  double x = -1, y = -1;
  default_grid.cellToPosition(0,0,x,y);
  EXPECT_DOUBLE_EQ(0, x);
  EXPECT_DOUBLE_EQ(0, y);
  default_grid.cellToPosition(1,1,x,y);
  EXPECT_DOUBLE_EQ(1, x);
  EXPECT_DOUBLE_EQ(1, y);
  default_grid.cellToPosition(2,2,x,y);
  EXPECT_DOUBLE_EQ(2, x);
  EXPECT_DOUBLE_EQ(2, y);

  default_grid.cellToPosition(0,1,x,y);
  EXPECT_DOUBLE_EQ(0, x);
  EXPECT_DOUBLE_EQ(1, y);
  default_grid.cellToPosition(0,2,x,y);
  EXPECT_DOUBLE_EQ(0, x);
  EXPECT_DOUBLE_EQ(2, y);
  default_grid.cellToPosition(1,2,x,y);
  EXPECT_DOUBLE_EQ(1, x);
  EXPECT_DOUBLE_EQ(2, y);

  default_grid.cellToPosition(1,0,x,y);
  EXPECT_DOUBLE_EQ(1, x);
  EXPECT_DOUBLE_EQ(0, y);
  default_grid.cellToPosition(2,0,x,y);
  EXPECT_DOUBLE_EQ(2, x);
  EXPECT_DOUBLE_EQ(0, y);
  default_grid.cellToPosition(2,1,x,y);
  EXPECT_DOUBLE_EQ(2, x);
  EXPECT_DOUBLE_EQ(1, y);

  default_grid.setResolution(0.5);

  default_grid.cellToPosition(0,0,x,y);
  EXPECT_DOUBLE_EQ(0./2, x);
  EXPECT_DOUBLE_EQ(0./2, y);
  default_grid.cellToPosition(1,1,x,y);
  EXPECT_DOUBLE_EQ(1./2, x);
  EXPECT_DOUBLE_EQ(1./2, y);
  default_grid.cellToPosition(2,2,x,y);
  EXPECT_DOUBLE_EQ(2./2, x);
  EXPECT_DOUBLE_EQ(2./2, y);

  default_grid.cellToPosition(0,1,x,y);
  EXPECT_DOUBLE_EQ(0./2, x);
  EXPECT_DOUBLE_EQ(1./2, y);
  default_grid.cellToPosition(0,2,x,y);
  EXPECT_DOUBLE_EQ(0./2, x);
  EXPECT_DOUBLE_EQ(2./2, y);
  default_grid.cellToPosition(1,2,x,y);
  EXPECT_DOUBLE_EQ(1./2, x);
  EXPECT_DOUBLE_EQ(2./2, y);

  default_grid.cellToPosition(1,0,x,y);
  EXPECT_DOUBLE_EQ(1./2, x);
  EXPECT_DOUBLE_EQ(0./2, y);
  default_grid.cellToPosition(2,0,x,y);
  EXPECT_DOUBLE_EQ(2./2, x);
  EXPECT_DOUBLE_EQ(0./2, y);
  default_grid.cellToPosition(2,1,x,y);
  EXPECT_DOUBLE_EQ(2./2, x);
  EXPECT_DOUBLE_EQ(1./2, y);

  EXPECT_DOUBLE_EQ(0, default_grid.atCell(0,0));
  EXPECT_DOUBLE_EQ(1, default_grid.atCell(0,1));
  EXPECT_DOUBLE_EQ(2, default_grid.atCell(0,2));

  ASSERT_THROW(default_grid.atCell(3,3), std::out_of_range);
  ASSERT_NO_THROW(default_grid.atCellSafe(3,3));

  EXPECT_DOUBLE_EQ(0, default_grid.atCellSafe(3,3));

//  EXPECT_TRUE(false);
}

TEST_F(GridTest, TEST_GRID)
{
  EXPECT_EQ(grid.getDimension().height, 5);
  EXPECT_EQ(grid.getDimension().width,  5);
  EXPECT_EQ(grid.getOrigin().x,   1.23);
  EXPECT_EQ(grid.getOrigin().y,   3.45);
  EXPECT_EQ(grid.getOrigin().yaw, 0.17);
  EXPECT_EQ(grid.getResolution(), 0.5);

  ASSERT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin(1,2,3));
    grid.setResolution(0.1);
  );

  EXPECT_EQ(grid.getOrigin().x,   1);
  EXPECT_EQ(grid.getOrigin().y,   2);
  EXPECT_EQ(grid.getOrigin().yaw, 3);
  EXPECT_EQ(grid.getResolution(), 0.1);

  ASSERT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin());
    grid.setResolution(1);
  );

  EXPECT_DOUBLE_EQ(1, grid.atCell(0,0));
  EXPECT_DOUBLE_EQ(1, grid.atCell(0,1));
  EXPECT_DOUBLE_EQ(1, grid.atCell(0,2));
  EXPECT_DOUBLE_EQ(2, grid.atCell(0,3));
  EXPECT_DOUBLE_EQ(3, grid.atCell(0,4));

  EXPECT_DOUBLE_EQ(1, grid.atCell(1,0));
  EXPECT_DOUBLE_EQ(0, grid.atCell(1,1));
  EXPECT_DOUBLE_EQ(1, grid.atCell(1,2));
  EXPECT_DOUBLE_EQ(2, grid.atCell(1,3));
  EXPECT_DOUBLE_EQ(3, grid.atCell(1,4));

  EXPECT_DOUBLE_EQ(1, grid.atCell(2,0));
  EXPECT_DOUBLE_EQ(1, grid.atCell(2,1));
  EXPECT_DOUBLE_EQ(1, grid.atCell(2,2));
  EXPECT_DOUBLE_EQ(2, grid.atCell(2,3));
  EXPECT_DOUBLE_EQ(3, grid.atCell(2,4));

  EXPECT_DOUBLE_EQ(2, grid.atCell(3,0));
  EXPECT_DOUBLE_EQ(2, grid.atCell(3,1));
  EXPECT_DOUBLE_EQ(2, grid.atCell(3,2));
  EXPECT_DOUBLE_EQ(2, grid.atCell(3,3));
  EXPECT_DOUBLE_EQ(3, grid.atCell(3,4));

  EXPECT_DOUBLE_EQ(3, grid.atCell(4,0));
  EXPECT_DOUBLE_EQ(3, grid.atCell(4,1));
  EXPECT_DOUBLE_EQ(3, grid.atCell(4,2));
  EXPECT_DOUBLE_EQ(3, grid.atCell(4,3));
  EXPECT_DOUBLE_EQ(3, grid.atCell(4,4));

  ///

  EXPECT_DOUBLE_EQ(1, grid.atPosition(0,0));
  EXPECT_DOUBLE_EQ(1, grid.atPosition(0,1));
  EXPECT_DOUBLE_EQ(1, grid.atPosition(0,2));
  EXPECT_DOUBLE_EQ(2, grid.atPosition(0,3));
  EXPECT_DOUBLE_EQ(3, grid.atPosition(0,4));

  EXPECT_DOUBLE_EQ(1, grid.atPosition(1,0));
  EXPECT_DOUBLE_EQ(0, grid.atPosition(1,1));
  EXPECT_DOUBLE_EQ(1, grid.atPosition(1,2));
  EXPECT_DOUBLE_EQ(2, grid.atPosition(1,3));
  EXPECT_DOUBLE_EQ(3, grid.atPosition(1,4));

  EXPECT_DOUBLE_EQ(1, grid.atPosition(2,0));
  EXPECT_DOUBLE_EQ(1, grid.atPosition(2,1));
  EXPECT_DOUBLE_EQ(1, grid.atPosition(2,2));
  EXPECT_DOUBLE_EQ(2, grid.atPosition(2,3));
  EXPECT_DOUBLE_EQ(3, grid.atPosition(2,4));

  EXPECT_DOUBLE_EQ(2, grid.atPosition(3,0));
  EXPECT_DOUBLE_EQ(2, grid.atPosition(3,1));
  EXPECT_DOUBLE_EQ(2, grid.atPosition(3,2));
  EXPECT_DOUBLE_EQ(2, grid.atPosition(3,3));
  EXPECT_DOUBLE_EQ(3, grid.atPosition(3,4));

  EXPECT_DOUBLE_EQ(3, grid.atPosition(4,0));
  EXPECT_DOUBLE_EQ(3, grid.atPosition(4,1));
  EXPECT_DOUBLE_EQ(3, grid.atPosition(4,2));
  EXPECT_DOUBLE_EQ(3, grid.atPosition(4,3));
  EXPECT_DOUBLE_EQ(3, grid.atPosition(4,4));

  ///

  ASSERT_NO_THROW(
    grid.setResolution(0.5);
  );

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0,0));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0,1./2));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0,2./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(0,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(0,4./2));

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2,0));
  EXPECT_DOUBLE_EQ(0./2, grid.atPosition(1./2.,1./2.));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2,2./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(1./2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(1./2,4./2));

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2,0));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2,1./2));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2,2./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(2./2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(2./2,4./2));

  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2,0));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2,1./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2,2./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(3./2,4./2));

  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,0));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,1./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,2./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,4./2));

  ///

  ASSERT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin(1.2,0,0));
    grid.setResolution(0.5);
  );

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,0));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,1./2));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,2./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(0+1.2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(0+1.2,4./2));

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2.+1.2,0));
  EXPECT_DOUBLE_EQ(0./2, grid.atPosition(1./2.+1.2,1./2.));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2.+1.2,2./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(1./2.+1.2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(1./2.+1.2,4./2));

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,0));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,1./2));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,2./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(2./2.+1.2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(2./2.+1.2,4./2));

  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,0));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,1./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,2./2));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(3./2.+1.2,4./2));

  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,0));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,1./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,2./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,3./2));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,4./2));

  ASSERT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin(1.2,4.7,0));
    grid.setResolution(0.5);
  );

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,0.+4.7));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,1./2.+4.7));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,2./2.+4.7));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(0+1.2,3./2.+4.7));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(0+1.2,4./2.+4.7));

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2.+1.2,0.+4.7));
  EXPECT_DOUBLE_EQ(0./2, grid.atPosition(1./2.+1.2,1./2.+4.7));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2.+1.2,2./2.+4.7));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(1./2.+1.2,3./2.+4.7));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(1./2.+1.2,4./2.+4.7));

  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,0.+4.7));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,1./2.+4.7));
  EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,2./2.+4.7));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(2./2.+1.2,3./2.+4.7));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(2./2.+1.2,4./2.+4.7));

  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,0.+4.7));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,1./2.+4.7));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,2./2.+4.7));
  EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,3./2.+4.7));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(3./2.+1.2,4./2.+4.7));

  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,0.+4.7));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,1./2.+4.7));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,2./2.+4.7));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,3./2.+4.7));
  EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,4./2.+4.7));

  ASSERT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin(3.17,-4.44,-M_PI/3));
    grid.setResolution(0.44);
  );

  const auto proj = [&](double& x, double& y)
  {
    //std::cout << "proj " << x << "," << y;

    const auto row = static_cast<std::size_t>(x);
    const auto col = static_cast<std::size_t>(y);
    grid.cellToPosition(row,col,x,y);

    //std::cout << " => " << x << "," << y << "\n";
  };

  double px, py;

  px=0; py=0; proj(px, py);
  EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py));
  px=0; py=1; proj(px, py);
  EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py));
  px=0; py=2; proj(px, py);
  EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py));
  px=0; py=3; proj(px, py);
  EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py));
  px=0; py=4; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));

  px=1; py=0; proj(px, py);
  EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py));
  px=1; py=1; proj(px, py);
  EXPECT_DOUBLE_EQ(0.* grid.getResolution(), grid.atPosition(px,py));
  px=1; py=2; proj(px, py);
  EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py));
  px=1; py=3; proj(px, py);
  EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py));
  px=1; py=4; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));

  px=2; py=0; proj(px, py);
  EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py));
  px=2; py=1; proj(px, py);
  EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py));
  px=2; py=2; proj(px, py);
  EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py));
  px=2; py=3; proj(px, py);
  EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py));
  px=2; py=4; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));

  px=3; py=0; proj(px, py);
  EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py));
  px=3; py=1; proj(px, py);
  EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py));
  px=3; py=2; proj(px, py);
  EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py));
  px=3; py=3; proj(px, py);
  EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py));
  px=3; py=4; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));

  px=4; py=0; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));
  px=4; py=1; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));
  px=4; py=2; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));
  px=4; py=3; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));
  px=4; py=4; proj(px, py);
  EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
