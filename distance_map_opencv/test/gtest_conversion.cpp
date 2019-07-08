#include <gtest/gtest.h>

#include "distance_map_opencv/distance_map_opencv.h"

struct GridTest : public testing::Test
{
  GridTest()
    : mat(5,7,CV_32FC1)
    , mat_converted(5,7,CV_8UC1)
    , grid(distmap::DistanceMap::Dimension(7,5),
           0.5,
           distmap::DistanceMap::Origin(1.23,3.45,0.17))

  {

    // pattern
    //
    // 1 1 1 2 3 4 5
    // 1 0 1 2 3 4 5
    // 1 1 1 2 3 4 5
    // 2 2 2 2 3 4 5
    // 3 3 3 3 3 4 5

    mat.at<float>(0,0) = 1;
    mat.at<float>(0,1) = 1;
    mat.at<float>(0,2) = 1;
    mat.at<float>(0,3) = 2;
    mat.at<float>(0,4) = 3;
    mat.at<float>(0,5) = 4;
    mat.at<float>(0,6) = 5;

    mat.at<float>(1,0) = 1;
    mat.at<float>(1,1) = 0;
    mat.at<float>(1,2) = 1;
    mat.at<float>(1,3) = 2;
    mat.at<float>(1,4) = 3;
    mat.at<float>(1,5) = 4;
    mat.at<float>(1,6) = 5;

    mat.at<float>(2,0) = 1;
    mat.at<float>(2,1) = 1;
    mat.at<float>(2,2) = 1;
    mat.at<float>(2,3) = 2;
    mat.at<float>(2,4) = 3;
    mat.at<float>(2,5) = 4;
    mat.at<float>(2,6) = 5;

    mat.at<float>(3,0) = 2;
    mat.at<float>(3,1) = 2;
    mat.at<float>(3,2) = 2;
    mat.at<float>(3,3) = 2;
    mat.at<float>(3,4) = 3;
    mat.at<float>(3,5) = 4;
    mat.at<float>(3,6) = 5;

    mat.at<float>(4,0) = 3;
    mat.at<float>(4,1) = 3;
    mat.at<float>(4,2) = 3;
    mat.at<float>(4,3) = 3;
    mat.at<float>(4,4) = 3;
    mat.at<float>(4,5) = 4;
    mat.at<float>(4,6) = 5;

    ///

    // mat_converted.at<uchar>(0,0) = 0;
    // mat_converted.at<uchar>(0,1) = 0;
    // mat_converted.at<uchar>(0,2) = 0;
    // mat_converted.at<uchar>(0,3) = 0;
    // mat_converted.at<uchar>(0,4) = 0;
    // mat_converted.at<uchar>(0,5) = 0;
    // mat_converted.at<uchar>(0,6) = 100;
    //
    // mat_converted.at<uchar>(1,0) = 0;
    // mat_converted.at<uchar>(1,1) = 0;
    // mat_converted.at<uchar>(1,2) = 1;
    // mat_converted.at<uchar>(1,3) = 2;
    // mat_converted.at<uchar>(1,4) = 3;
    // mat_converted.at<uchar>(1,5) = 4;
    // mat_converted.at<uchar>(1,6) = 100;
    //
    // mat_converted.at<uchar>(2,0) = 1;
    // mat_converted.at<uchar>(2,1) = 1;
    // mat_converted.at<uchar>(2,2) = 1;
    // mat_converted.at<uchar>(2,3) = 2;
    // mat_converted.at<uchar>(2,4) = 3;
    // mat_converted.at<uchar>(2,5) = 4;
    // mat_converted.at<uchar>(2,6) = 5;
    //
    // mat_converted.at<uchar>(3,0) = 2;
    // mat_converted.at<uchar>(3,1) = 2;
    // mat_converted.at<uchar>(3,2) = 2;
    // mat_converted.at<uchar>(3,3) = 2;
    // mat_converted.at<uchar>(3,4) = 3;
    // mat_converted.at<uchar>(3,5) = 4;
    // mat_converted.at<uchar>(3,6) = 5;
    //
    // mat_converted.at<uchar>(4,0) = 3;
    // mat_converted.at<uchar>(4,1) = 3;
    // mat_converted.at<uchar>(4,2) = 3;
    // mat_converted.at<uchar>(4,3) = 3;
    // mat_converted.at<uchar>(4,4) = 3;
    // mat_converted.at<uchar>(4,5) = 4;
    // mat_converted.at<uchar>(4,6) = 5;

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

    // occgrid.info.height = 5;
    // occgrid.info.width  = 7;
    //
    // occgrid.data.resize(7*5);
    //
    // *(occgrid.data.data()+ 0) = 0;
    // *(occgrid.data.data()+ 1) = 0;
    // *(occgrid.data.data()+ 2) = 0;
    // *(occgrid.data.data()+ 3) = 0;
    // *(occgrid.data.data()+ 4) = 0;
    // *(occgrid.data.data()+ 5) = 0;
    // *(occgrid.data.data()+ 6) = 100;
    //
    // *(occgrid.data.data()+ 7) = 0;
    // *(occgrid.data.data()+ 8) = 0;
    // *(occgrid.data.data()+ 9) = 1;
    // *(occgrid.data.data()+10) = 2;
    // *(occgrid.data.data()+11) = 3;
    // *(occgrid.data.data()+12) = 4;
    // *(occgrid.data.data()+13) = 100;
    //
    // *(occgrid.data.data()+14) = 1;
    // *(occgrid.data.data()+15) = 1;
    // *(occgrid.data.data()+16) = 1;
    // *(occgrid.data.data()+17) = 2;
    // *(occgrid.data.data()+18) = 3;
    // *(occgrid.data.data()+19) = 4;
    // *(occgrid.data.data()+20) = 100;
    //
    // *(occgrid.data.data()+21) = 2;
    // *(occgrid.data.data()+22) = 2;
    // *(occgrid.data.data()+23) = 2;
    // *(occgrid.data.data()+24) = 2;
    // *(occgrid.data.data()+25) = 3;
    // *(occgrid.data.data()+26) = 4;
    // *(occgrid.data.data()+27) = 100;
    //
    // *(occgrid.data.data()+28) = 3;
    // *(occgrid.data.data()+29) = 3;
    // *(occgrid.data.data()+30) = 3;
    // *(occgrid.data.data()+31) = 3;
    // *(occgrid.data.data()+32) = 3;
    // *(occgrid.data.data()+33) = 4;
    // *(occgrid.data.data()+34) = 100;
  }

  nav_msgs::OccupancyGrid occgrid;
  cv::Mat mat, mat_converted;
  distmap::DistanceMap grid;
};

/// @todo(artivis) fix test
// TEST_F(GridTest, TEST_OCCGRID2MAT_CONVERSION)
// {
//   cv::Mat matc;
//
//   ASSERT_NO_THROW(
//     matc = distmap::DistanceMapOpencv::occupancyGridToMat(occgrid)
//            );
//
//   EXPECT_EQ(matc.rows, occgrid.info.height);
//   EXPECT_EQ(matc.cols, occgrid.info.width);
//
//   // for (int row = 0; row<mat.rows; ++row)
//   //   for (int col = 0; col<mat.cols; ++col)
//   //   {
//   //     ASSERT_NO_THROW(
//   //       EXPECT_FLOAT_EQ((double)mat.at<uchar>(row,col),
//   //                       (double)matc.at<uchar>(row,col))
//   //         << "at " << row << "," << col);
//   //   }
//
// //  for (int row = 0; row<mat.rows; ++row)
// //    for (int col = 0; col<mat.cols; ++col)
// //    {
// //      ASSERT_NO_THROW(
// //        EXPECT_FLOAT_EQ(mat.at<float>(row,col), gridc.atCell(row,col))
// //          << "at " << row << "," << col);
// //    }
// }

TEST_F(GridTest, TEST_MAT2GRID_CONVERSION)
{
  distmap::DistanceMap gridc(
        distmap::DistanceMap::Dimension(2,2),
        0.5,
        distmap::DistanceMap::Origin(1,1,1));

  ASSERT_NO_THROW(distmap::DistanceMapOpencv::matToDistanceMap(
                    mat, 1, gridc));

  EXPECT_EQ(gridc.getDimension().height, mat.rows);
  EXPECT_EQ(gridc.getDimension().width, mat.cols);
  EXPECT_EQ(gridc.getResolution(), 1);

  EXPECT_EQ(gridc.getOrigin().x,   1);
  EXPECT_EQ(gridc.getOrigin().y,   1);
  EXPECT_EQ(gridc.getOrigin().yaw, 1);

  for (int row = 0; row<mat.rows; ++row)
    for (int col = 0; col<mat.cols; ++col)
    {
      ASSERT_NO_THROW(
        EXPECT_FLOAT_EQ(grid.atCell(row,col), gridc.atCell(row,col))
          << "at " << row << "," << col);
    }

  for (int row = 0; row<mat.rows; ++row)
    for (int col = 0; col<mat.cols; ++col)
    {
      ASSERT_NO_THROW(
        EXPECT_FLOAT_EQ(mat.at<float>(row,col), gridc.atCell(row,col))
          << "at " << row << "," << col);
    }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
