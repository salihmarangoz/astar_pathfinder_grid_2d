
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <chrono>

#include <boost/dynamic_bitset.hpp>

// dont define this in your code. use only for debugging accessed nodes on the grid
#define DEBUG_PATHFINDER_GRID_2D (200)

#include "pathfinder_grid_2d.h"

using namespace cv;

Mat load_maze(std::string filename)
{
    Mat img = imread(filename, IMREAD_GRAYSCALE);
    if (img.empty())
    {
        std::cout << "Could not read the image: " << filename << std::endl;
        exit(1);
    }

    return img;
}

void show_maze(Mat &img, const pathfinder_grid_2d::Path &path, std::string windowName)
{
    Mat img_color;
    cvtColor(img, img_color, COLOR_GRAY2BGR);

#ifdef DEBUG_PATHFINDER_GRID_2D
    for (int i=0; i<img.rows; i++)
    {
        for (int j=0; j<img.cols; j++)
        {
            if (img.at<uchar>(i,j) == DEBUG_PATHFINDER_GRID_2D)
            {
                Vec3b &color = img_color.at<Vec3b>(i, j);
                color[0] = 222;
                color[1] = 222;
                color[2] = 255;
            }

        }
    }
#endif

    for (int i = 0; i < path.size(); i++)
    {
        Vec3b &color = img_color.at<Vec3b>(path[i].i, path[i].j);
        color[0] = 0;
        color[1] = 0;
        color[2] = 255;
    }

    namedWindow(windowName, WINDOW_NORMAL);
    resizeWindow(windowName, 1024, 1024);
    imshow(windowName, img_color);
    waitKey(0);
}

void demo_maze(Mat img)
{
    const auto start = std::chrono::system_clock::now();

    // pathfinder_grid_2d::PathFinderGrid2D<uchar> planner(img.data, img.rows, img.cols); // stable algorithm
    pathfinder_grid_2d::FastPathFinderGrid2D<uchar> planner(img.data, img.rows, img.cols); // fast algorithm

    pathfinder_grid_2d::Path out;
    int start_i = 0, start_j = img.cols / 2;
    int end_i = img.rows - 1, end_j = img.cols / 2;

    bool success = planner.plan(start_i, start_j, end_i, end_j, out, 127, false);

    const auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printf("demo_maze timing: %ld ms\n", elapsed.count());

    if (success)
    {
        printf("demo_maze plan succesful!\n");
        show_maze(img, out, "demo_maze");
    }
    else
    {
        printf("demo_maze plan failed!\n");
        return;
    }
    
}

void demo_maze_bitset(Mat img)
{
    // convert image to bitset
    boost::dynamic_bitset<> img_bitset;
    img_bitset.reserve(img.rows*img.cols);
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            img_bitset.push_back(img.at<uchar>(i,j));
        }
    }

    const auto start = std::chrono::system_clock::now();

    //pathfinder_grid_2d::PathFinderGrid2D<bool> planner(&img_bitset, img.rows, img.cols);  // stable algorithm
    pathfinder_grid_2d::FastPathFinderGrid2D<bool> planner(&img_bitset, img.rows, img.cols); // fast algorithm

    pathfinder_grid_2d::Path out;
    int start_i = 0;
    int start_j = img.cols / 2;
    int end_i = img.rows - 1;
    int end_j = img.cols / 2;

    bool success = planner.plan(start_i, start_j, end_i, end_j, out, 127, false);

    const auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printf("demo_maze_bitset timing: %ld ms\n", elapsed.count());

    if (success)
    {
        printf("demo_maze_bitset plan succesful!\n");
        show_maze(img, out, "demo_maze_bitset");
    }
    else
    {
        printf("demo_maze_bitset plan failed!\n");
        return;
    }
}

int main()
{
    std::string maze_files[] = {"../assets/trap.png", "../assets/maze9.png", "../assets/maze99.png", "../assets/maze199.png"};

    for (auto f : maze_files)
    {
        demo_maze(load_maze(f));
        demo_maze_bitset(load_maze(f));
    }
    return 0;
}