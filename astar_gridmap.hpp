#pragma once

#include <memory>
#include <vector>
#include <cmath>
#include <boost/heap/fibonacci_heap.hpp>

namespace astar_gridmap
{

struct AstarPoint;
struct AstarPointPtr;
struct AstarPointPtrComparator;

struct AstarPoint
{
    size_t i, j;
    float dist_from_start, total_dist;
    AstarPoint *came_from;
    bool handle_available;
    //void *handle; // boost::heap::fibonacci_heap<AstarPointPtr, boost::heap::compare<AstarPointPtrComparator>>::handle_type

    AstarPoint() : AstarPoint(-1, -1, -1.0, -1.0, nullptr) {}
    AstarPoint(size_t i, size_t j, float dist_from_start, float dist_to_end, AstarPoint *came_from)
    {
        this->i = i;
        this->j = j;
        this->dist_from_start = dist_from_start;
        this->total_dist = dist_from_start + dist_to_end;
        this->came_from = came_from;
        this->handle_available = false;
    }
};

struct AstarPointPtr
{
    AstarPoint *ptr;
    AstarPointPtr(AstarPoint *ptr)
    {
        this->ptr = ptr;
    }
};

struct AstarPointPtrComparator
{
    bool operator()(const AstarPointPtr &p1, const AstarPointPtr &p2) const
    {
        return p1.ptr->total_dist > p2.ptr->total_dist;
    }
};

template <typename T>
class AstarGridMap2D
{
public:
    T const *m_gridmap;
    std::shared_ptr<const std::vector<T>> m_gridmap_ptr;
    size_t m_rows, m_cols;

    AstarGridMap2D(T const *gridmap, size_t rows, size_t cols)
        : m_gridmap(gridmap), m_rows(rows), m_cols(cols)
    {
    }

    AstarGridMap2D(std::shared_ptr<const std::vector<T>> gridmap_ptr, size_t rows, size_t cols)
        : m_gridmap_ptr(gridmap_ptr), m_rows(rows), m_cols(cols)
    {
        m_gridmap = m_gridmap_ptr->data();
    }

    bool isInside(size_t i, size_t j) const
    {
        if (i < m_rows && j < m_cols && i >= 0 && j >= 0)
            return true;
        else
            return false;
    }

    size_t at(size_t i, size_t j) const
    {
        return i * m_cols + j;
    }

    bool plan(size_t start_i, size_t start_j, size_t end_i, size_t end_j, std::vector<std::pair<size_t, size_t>> &path_out, T threshold, bool obstacleHasHigherValue)
    {
        boost::heap::fibonacci_heap<AstarPointPtr, boost::heap::compare<AstarPointPtrComparator>> min_heap;
        std::vector<AstarPoint> points;
        points.resize(m_rows * m_cols);

        float dist_start_to_end = std::hypot(start_i - end_i, start_j - end_j);
        AstarPoint p(start_i, start_j, 0.0, dist_start_to_end, nullptr);
        int idx = at(start_i, start_j);
        points[idx] = p;
        /*points[idx].handle = */min_heap.push(AstarPointPtr(&(points[idx])));
        printf("%d\n", min_heap.size());

        bool goal_reached = false;
        while (!goal_reached && min_heap.size() > 0)
        {
            // get AstarPoint with the lowest total distance value
            AstarPointPtr curr = min_heap.top();
            min_heap.pop();
            curr.ptr->handle_available = false;

            // check end condition
            if (curr.ptr->i == end_i && curr.ptr->j == end_j)
            {
                goal_reached = true;
                break;
            }

            printf("%d\n", min_heap.size());

            // add neighbors
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    // find neighbor positions
                    int n_i = curr.ptr->i + i;
                    int n_j = curr.ptr->j + j;
                    int n_idx = at(n_i, n_j);

                    if (i == 0 && j == 0)
                        continue; // if tries to add itself
                    if (!isInside(n_i, n_j))
                        continue; // if neighbor is outside of the costmap
                    if (m_gridmap[n_idx] < threshold)
                        continue; // if neighbor's cost is high

                    float dist_from_start = ((i == 0 || j == 0) ? (1.0) : (M_SQRT2)) + curr.ptr->dist_from_start;
                    float dist_to_end = std::hypot(n_i - end_i, n_j - end_j); // euclidian estimation
                    AstarPoint new_p(n_i, n_j, dist_from_start, dist_to_end, curr.ptr);

                    // if there is a neighbor previously evaluated
                    if (points[n_idx].i >= 0 && points[n_idx].j >= 0)
                    {
                        // overwrite the current neighbor if the current neighbor's distance_from_start is smaller
                        if (points[n_idx].dist_from_start > dist_from_start)
                        {
                            //auto handle = points[n_idx].handle;
                            points[n_idx] = new_p;
                            //points[n_idx].handle = handle;

                            if (points[n_idx].handle_available) // if neighbor still in the heap, update
                            {
                                //min_heap.update((boost::heap::fibonacci_heap<AstarPointPtr, boost::heap::compare<AstarPointPtrComparator>>::handle_type)(points[n_idx].handle));
                            }
                            else // if neighbor is previously evaluated and not in the heap, add to heap for new evaluation
                            {
                                /*points[n_idx].handle =*/ min_heap.push(AstarPointPtr(&(points[n_idx])));
                                //points[n_idx].handle_available = true;
                            }
                        }
                    }
                    else
                    {
                        points[n_idx] = new_p;
                        //points[n_idx].handle = min_heap.push(AstarPointPtr(&(points[n_idx])));
                        //points[n_idx].handle_available = true;
                    }
                }
            }
        }

        if (goal_reached)
        {
            path_out.clear();

            // backtrack
            int idx = end_i * m_cols + end_j;
            AstarPoint *cur = &points[idx];
            path_out.push_back(std::make_pair(end_i, end_j));
            while (cur->came_from != nullptr)
            {
                cur = cur->came_from;
                path_out.push_back(std::make_pair(cur->i, cur->j));
            }
            std::reverse(path_out.begin(), path_out.end());

            return true;
        }

        return false; // plan failed
    }
};

} // astar_gridmap