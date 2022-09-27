
#include "astar_gridmap.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>

using namespace cv;

int main()
{
    std::string filename = "../pairtest.png";

    Mat img = imread(filename, IMREAD_GRAYSCALE);
    if (img.empty())
    {
        std::cout << "Could not read the image: " << filename << std::endl;
        return 1;
    }

    std::vector<std::pair<size_t, size_t>> out;
    astar_gridmap::AstarGridMap2D<uchar> planner(img.data, img.rows, img.cols);
    bool success = planner.plan(0, 0, 10, 10, out, 0, true);

    if (success)
    {
        std::cout << "Success!" << out.size() << std::endl;
    }

    imshow("Display window", img);
    waitKey(0);

    return 0;
}