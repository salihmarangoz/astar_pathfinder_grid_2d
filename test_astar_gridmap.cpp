
#include "astar_gridmap.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/binomial_heap.hpp> // TODO: segfault. sometimes
#include <boost/heap/pairing_heap.hpp> // TODO: segfault

using namespace cv;

struct NodeA
{
    int a;

    NodeA(int a)
    {
        this->a = a;
    }

    bool operator<(NodeA const &rhs) const
    {
        return a > rhs.a;
    }
};

void test_heap()
{
    boost::heap::binomial_heap<NodeA> heap;

    heap.push(NodeA(2));
    heap.push(NodeA(200));
    heap.push(NodeA(3));
    heap.push(NodeA(6));
    heap.push(NodeA(100));
    heap.push(NodeA(7));
    heap.push(NodeA(8));
    auto handle = heap.push(NodeA(1));
    heap.push(NodeA(500));

    (*handle).a = 5; // increase value from 1 to 5
    heap.decrease(handle); // correct!
    //heap.increase(handle); // incorrect!
    //heap.update(handle); // correct but slower.

    printf("heap size: %d\n", heap.size());

    int maxiter = heap.size();
    for (int i = 0; i < maxiter; i++)
    {
        auto a = heap.top();

        printf("popped element: %d\n", a.a);
        heap.pop();
    }
}

void test_maze()
{
    // std::string filename = "../maze_large.png";
    std::string filename = "../maze_1000_smooth.png";

    Mat img = imread(filename, IMREAD_GRAYSCALE);
    if (img.empty())
    {
        std::cout << "Could not read the image: " << filename << std::endl;
        return;
    }

    std::vector<std::pair<int, int>> out;
    astar_gridmap::AstarGridMap2D<uchar> planner(img.data, img.rows, img.cols);
    bool success = planner.plan(5, img.cols / 2, img.rows - 5, img.cols / 2, out, 127, true);

    if (success)
    {
        std::cout << "Success!" << out.size() << std::endl;
    }

    for (int i = 0; i < out.size(); i++)
    {
        img.at<uint8_t>(out[i].first, out[i].second) = 255;
    }

    namedWindow("Display window", WINDOW_NORMAL);
    resizeWindow("Display window", 1024, 1024);
    imshow("Display window", img);
    waitKey(0);
}

int main()
{
    //test_heap();
    test_maze();
    return 0;
}