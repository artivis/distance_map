#include <gtest/gtest.h>

#include <distance_map_core/distance_field_grid.h>

struct GridTest : public testing::Test
{
  GridTest()
    : default_grid(distmap::DistanceFieldGrid::Dimension(3,3))
    , grid(distmap::DistanceFieldGrid::Dimension(7,5),
           0.5,
           distmap::DistanceFieldGrid::Origin(1.23,3.45,0.17))

  {
    for (int i=0; i<3*3; ++i)
      *(default_grid.data()+i) = i;

    // pattern
    //
    // 1 1 1 2 3 4 5
    // 1 0 1 2 3 4 5
    // 1 1 1 2 3 4 5
    // 2 2 2 2 3 4 5
    // 3 3 3 3 3 4 5

    *(grid.data()+ 0) = 1;
    *(grid.data()+ 1) = 1;
    *(grid.data()+ 2) = 1;
    *(grid.data()+ 3) = 2;
    *(grid.data()+ 4) = 3;
    *(grid.data()+ 5) = 4;
    *(grid.data()+ 6) = 5;

    *(grid.data()+ 7) = 1;
    *(grid.data()+ 8) = 0;
    *(grid.data()+ 9) = 1;
    *(grid.data()+10) = 2;
    *(grid.data()+11) = 3;
    *(grid.data()+12) = 4;
    *(grid.data()+13) = 5;

    *(grid.data()+14) = 1;
    *(grid.data()+15) = 1;
    *(grid.data()+16) = 1;
    *(grid.data()+17) = 2;
    *(grid.data()+18) = 3;
    *(grid.data()+19) = 4;
    *(grid.data()+20) = 5;

    *(grid.data()+21) = 2;
    *(grid.data()+22) = 2;
    *(grid.data()+23) = 2;
    *(grid.data()+24) = 2;
    *(grid.data()+25) = 3;
    *(grid.data()+26) = 4;
    *(grid.data()+27) = 5;

    *(grid.data()+28) = 3;
    *(grid.data()+29) = 3;
    *(grid.data()+30) = 3;
    *(grid.data()+31) = 3;
    *(grid.data()+32) = 3;
    *(grid.data()+33) = 4;
    *(grid.data()+34) = 5;
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
  ASSERT_NO_THROW(default_grid.setResolution(1));

  EXPECT_TRUE(default_grid.isCellValid(0,0));
  EXPECT_TRUE(default_grid.isCellValid(1,1));
  EXPECT_TRUE(default_grid.isCellValid(2,2));

  EXPECT_FALSE(default_grid.isCellValid(3,3));
  EXPECT_FALSE(default_grid.isCellValid(0,3));
  EXPECT_FALSE(default_grid.isCellValid(3,0));

  double x = -1, y = -1;
  default_grid.cellToPosition(0,0,x,y);
  EXPECT_DOUBLE_EQ(0, x);
  EXPECT_DOUBLE_EQ(2, y);
  default_grid.cellToPosition(1,1,x,y);
  EXPECT_DOUBLE_EQ(1, x);
  EXPECT_DOUBLE_EQ(1, y);
  default_grid.cellToPosition(2,2,x,y);
  EXPECT_DOUBLE_EQ(2, x);
  EXPECT_DOUBLE_EQ(0, y);

  default_grid.cellToPosition(0,1,x,y);
  EXPECT_DOUBLE_EQ(1, x);
  EXPECT_DOUBLE_EQ(2, y);
  default_grid.cellToPosition(0,2,x,y);
  EXPECT_DOUBLE_EQ(2, x);
  EXPECT_DOUBLE_EQ(2, y);
  default_grid.cellToPosition(1,2,x,y);
  EXPECT_DOUBLE_EQ(2, x);
  EXPECT_DOUBLE_EQ(1, y);

  default_grid.cellToPosition(1,0,x,y);
  EXPECT_DOUBLE_EQ(0, x);
  EXPECT_DOUBLE_EQ(1, y);
  default_grid.cellToPosition(2,0,x,y);
  EXPECT_DOUBLE_EQ(0, x);
  EXPECT_DOUBLE_EQ(0, y);
  default_grid.cellToPosition(2,1,x,y);
  EXPECT_DOUBLE_EQ(1, x);
  EXPECT_DOUBLE_EQ(0, y);

  default_grid.setResolution(0.5);

  default_grid.cellToPosition(0,0,x,y);
  EXPECT_DOUBLE_EQ(0./2, x);
  EXPECT_DOUBLE_EQ(2./2, y);
  default_grid.cellToPosition(1,1,x,y);
  EXPECT_DOUBLE_EQ(1./2, x);
  EXPECT_DOUBLE_EQ(1./2, y);
  default_grid.cellToPosition(2,2,x,y);
  EXPECT_DOUBLE_EQ(2./2, x);
  EXPECT_DOUBLE_EQ(0./2, y);

  default_grid.cellToPosition(0,1,x,y);
  EXPECT_DOUBLE_EQ(1./2, x);
  EXPECT_DOUBLE_EQ(2./2, y);
  default_grid.cellToPosition(0,2,x,y);
  EXPECT_DOUBLE_EQ(2./2, x);
  EXPECT_DOUBLE_EQ(2./2, y);
  default_grid.cellToPosition(1,2,x,y);
  EXPECT_DOUBLE_EQ(2./2, x);
  EXPECT_DOUBLE_EQ(1./2, y);

  default_grid.cellToPosition(1,0,x,y);
  EXPECT_DOUBLE_EQ(0./2, x);
  EXPECT_DOUBLE_EQ(1./2, y);
  default_grid.cellToPosition(2,0,x,y);
  EXPECT_DOUBLE_EQ(0./2, x);
  EXPECT_DOUBLE_EQ(0./2, y);
  default_grid.cellToPosition(2,1,x,y);
  EXPECT_DOUBLE_EQ(1./2, x);
  EXPECT_DOUBLE_EQ(0./2, y);

  EXPECT_DOUBLE_EQ(0, default_grid.atCell(0,0));
  EXPECT_DOUBLE_EQ(1, default_grid.atCell(0,1));
  EXPECT_DOUBLE_EQ(2, default_grid.atCell(0,2));

  ASSERT_THROW(default_grid.atCell(3,3), std::out_of_range);
  ASSERT_NO_THROW(default_grid.atCellSafe(3,3));

  EXPECT_DOUBLE_EQ(0, default_grid.atCellSafe(3,3));

  for (int r=0; r<3; ++r)
    for (int c=0; c<3; ++c)
    {
      double x = -1, y = -1;
      default_grid.cellToPosition(r,c,x,y);

      std::size_t row = 9, col = 9;
      default_grid.positionToCell(x,y,row,col);

      EXPECT_EQ(r, row);
      EXPECT_EQ(c, col);
    }

  EXPECT_NO_THROW(
    default_grid.setOrigin(distmap::DistanceFieldGrid::Origin(-23.17,-4.44,-M_PI/3));
    default_grid.setResolution(0.44);
  );

  for (int r=0; r<3; ++r)
    for (int c=0; c<3; ++c)
    {
      double x = -1, y = -1;
      default_grid.cellToPosition(r,c,x,y);

      std::size_t row = 9, col = 9;
      default_grid.positionToCell(x,y,row,col);

      EXPECT_EQ(r, row);
      EXPECT_EQ(c, col);
    }
}

TEST_F(GridTest, TEST_GRID)
{
  EXPECT_EQ(grid.getDimension().height, 5);
  EXPECT_EQ(grid.getDimension().width,  7);
  EXPECT_EQ(grid.getOrigin().x,   1.23);
  EXPECT_EQ(grid.getOrigin().y,   3.45);
  EXPECT_EQ(grid.getOrigin().yaw, 0.17);
  EXPECT_EQ(grid.getResolution(), 0.5);

  EXPECT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin(1,2,3));
    grid.setResolution(0.1);
  );

  EXPECT_EQ(grid.getOrigin().x,   1);
  EXPECT_EQ(grid.getOrigin().y,   2);
  EXPECT_EQ(grid.getOrigin().yaw, 3);
  EXPECT_EQ(grid.getResolution(), 0.1);

  EXPECT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin());
    grid.setResolution(1);
  );

  //// Testing Cell

  {
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atCell(0,0)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atCell(0,1)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atCell(0,2)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atCell(0,3)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(0,4)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atCell(0,5)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atCell(0,6)));

    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atCell(1,0)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(0, grid.atCell(1,1)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atCell(1,2)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atCell(1,3)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(1,4)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atCell(1,5)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atCell(1,6)));

    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atCell(2,0)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atCell(2,1)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atCell(2,2)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atCell(2,3)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(2,4)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atCell(2,5)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atCell(2,6)));

    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atCell(3,0)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atCell(3,1)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atCell(3,2)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atCell(3,3)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(3,4)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atCell(3,5)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atCell(3,6)));

    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(4,0)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(4,1)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(4,2)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(4,3)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atCell(4,4)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atCell(4,5)));
    EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atCell(4,6)));
  }

  //// Testing Position

  /// All default

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(0,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atPosition(0,1)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atPosition(0,2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atPosition(0,3)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atPosition(0,4)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(1,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atPosition(1,1)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atPosition(1,2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(0, grid.atPosition(1,3)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atPosition(1,4)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atPosition(2,1)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atPosition(2,2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atPosition(2,3)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1, grid.atPosition(2,4)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(3,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atPosition(3,1)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atPosition(3,2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atPosition(3,3)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2, grid.atPosition(3,4)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(4,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(4,1)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(4,2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(4,3)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3, grid.atPosition(4,4)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atPosition(5,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atPosition(5,1)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atPosition(5,2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atPosition(5,3)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4, grid.atPosition(5,4)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atPosition(6,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atPosition(6,1)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atPosition(6,2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atPosition(6,3)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5, grid.atPosition(6,4)));

  /// Resolution + Origin x-offset & y-offset

  EXPECT_NO_THROW(
    grid.setResolution(0.5);
  );

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(0,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(0,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(1./2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(1./2.,1./2.)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(0./2, grid.atPosition(1./2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(2./2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(2./2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(3./2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2,4./2)));

  /// Resolution + Origin x-offset

  EXPECT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin(1.2,0,0));
    grid.setResolution(0.5);
  );

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(0+1.2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(0+1.2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(1./2.+1.2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(1./2.+1.2,1./2.)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2.+1.2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(0./2, grid.atPosition(1./2.+1.2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2.+1.2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(2./2.+1.2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(2./2.+1.2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(3./2.+1.2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,4./2)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,0)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,1./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,2./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,3./2)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,4./2)));

  /// Resolution + Origin x-offset & y-offset

  EXPECT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin(1.2,4.7,0));
    grid.setResolution(0.5);
  );

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(0+1.2,0.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(0+1.2,1./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,2./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,3./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(0+1.2,4./2.+4.7)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(1./2.+1.2,0.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(1./2.+1.2,1./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2.+1.2,2./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(0./2, grid.atPosition(1./2.+1.2,3./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(1./2.+1.2,4./2.+4.7)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(2./2.+1.2,0.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(2./2.+1.2,1./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,2./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,3./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1./2, grid.atPosition(2./2.+1.2,4./2.+4.7)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(3./2.+1.2,0.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,1./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,2./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,3./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2./2, grid.atPosition(3./2.+1.2,4./2.+4.7)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,0.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,1./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,2./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,3./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3./2, grid.atPosition(4./2.+1.2,4./2.+4.7)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,0.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,1./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,2./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,3./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(4./2, grid.atPosition(5./2.+1.2,4./2.+4.7)));

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,0.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,1./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,2./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,3./2.+4.7)));
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(5./2, grid.atPosition(6./2.+1.2,4./2.+4.7)));

  /// Resolution + Origin x-offset & y-offset & yaw-offset

  EXPECT_NO_THROW(
    grid.setOrigin(distmap::DistanceFieldGrid::Origin(-23.17,-4.44,-M_PI/3));
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
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py)));
  px=0; py=1; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py)));
  px=0; py=2; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py)));
  px=0; py=3; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py)));
  px=0; py=4; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));

  px=1; py=0; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py)));
  px=1; py=1; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(0.* grid.getResolution(), grid.atPosition(px,py)));
  px=1; py=2; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py)));
  px=1; py=3; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py)));
  px=1; py=4; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));

  px=2; py=0; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py)));
  px=2; py=1; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py)));
  px=2; py=2; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(1.* grid.getResolution(), grid.atPosition(px,py)));
  px=2; py=3; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py)));
  px=2; py=4; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));

  px=3; py=0; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py)));
  px=3; py=1; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py)));
  px=3; py=2; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py)));
  px=3; py=3; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(2.* grid.getResolution(), grid.atPosition(px,py)));
  px=3; py=4; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));

  px=4; py=0; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));
  px=4; py=1; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));
  px=4; py=2; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));
  px=4; py=3; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));
  px=4; py=4; proj(px, py);
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(3.* grid.getResolution(), grid.atPosition(px,py)));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
