#pragma once

#include <memory>
#include <vector>
#include <cmath>

// https: // www.boost.org/doc/libs/1_80_0/doc/html/heap/data_structures.html
//#include <boost/heap/skew_heap.hpp> // not mutable by default
//#include <boost/heap/priority_queue.hpp> // not mutable by default
//#include <boost/heap/d_ary_heap.hpp> // not mutable by default
#include <boost/heap/fibonacci_heap.hpp> // fastest for big grids
#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/pairing_heap.hpp>

/* TODO
- selected_heap -> heap_algorithm
- T -> point_type
*/

namespace astar_gridmap
{

    struct GridCell
    {
        int i, j;

        GridCell()
        {
            this->i = -1;
            this->j = -1;
        }
        GridCell(int i, int j)
        {
            this->i = i;
            this->j = j;
        }
    };

    template <typename T>
    using selected_heap = boost::heap::binomial_heap<T>;

    struct AstarCellPtr;

    struct AstarCell
    {
        int i, j;
        float dist_from_start, total_dist;
        AstarCell *came_from;
        bool handle_available;
        selected_heap<AstarCellPtr>::handle_type handle;

        AstarCell() : AstarCell(-1, -1, -1.0, -1.0, nullptr) {}
        AstarCell(int i, int j, float dist_from_start, float dist_to_end, AstarCell *came_from)
        {
            this->i = i;
            this->j = j;
            this->dist_from_start = dist_from_start;
            this->total_dist = dist_from_start + dist_to_end;
            this->came_from = came_from;
            this->handle_available = false;
        }
    };

    struct AstarCellPtr
    {
        AstarCell *ptr;
        AstarCellPtr(AstarCell *ptr)
        {
            this->ptr = ptr;
        }

        bool operator<(AstarCellPtr const &rhs) const
        {
            return ptr->total_dist > rhs.ptr->total_dist;
        }
    };

    template <typename T>
    class AstarGridMap2D
    {
    public:
        T /*const*/ *m_gridmap;
        std::shared_ptr<const std::vector<T>> m_gridmap_ptr;
        int m_rows, m_cols;

        AstarGridMap2D(T /*const*/ *gridmap, int rows, int cols)
            : m_gridmap(gridmap), m_rows(rows), m_cols(cols)
        {
        }

        AstarGridMap2D(std::shared_ptr<const std::vector<T>> gridmap_ptr, int rows, int cols)
            : m_gridmap_ptr(gridmap_ptr), m_rows(rows), m_cols(cols)
        {
            m_gridmap = m_gridmap_ptr->data();
        }

        bool isInside(int i, int j) const
        {
            if (i < m_rows && j < m_cols && i >= 0 && j >= 0)
                return true;
            else
                return false;
        }

        int at(int i, int j) const
        {
            return i * m_cols + j;
        }

        bool findClosestTarget(int start_i, int start_j, const std::vector<std::pair<int, int>> targets, std::vector<std::pair<int, int>> &path_out, T threshold, bool obstacleHasHigherValue)
        {

        }

        bool smoothPlan(std::vector<std::pair<int, int>> &path)
        {

        }

        bool plan(int start_i, int start_j, int end_i, int end_j, std::vector<std::pair<int, int>> &path_out, T threshold, bool obstacleHasHigherValue)
        {
            selected_heap<AstarCellPtr> min_heap;
            std::vector<AstarCell> points;
            points.resize(m_rows * m_cols);

            float dist_start_to_end = std::hypot(start_i - end_i, start_j - end_j);
            AstarCell p(start_i, start_j, 0.0, dist_start_to_end, nullptr);
            int idx = at(start_i, start_j);
            points[idx] = p;
            points[idx].handle = min_heap.push(AstarCellPtr(&(points[idx])));

            bool goal_reached = false;
            while (!goal_reached && min_heap.size() > 0)
            {
                // get AstarCell with the lowest total distance value
                AstarCellPtr curr = min_heap.top();
                min_heap.pop();
                curr.ptr->handle_available = false;

                // check end condition
                if (curr.ptr->i == end_i && curr.ptr->j == end_j)
                {
                    goal_reached = true;
                    break;
                }

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

                        m_gridmap[n_idx] = 200;

                        float dist_from_start = ((i == 0 || j == 0) ? (1.0) : (M_SQRT2)) + curr.ptr->dist_from_start;
                        float dist_to_end = std::hypot(n_i - end_i, n_j - end_j); // euclidian estimation
                        AstarCell new_p(n_i, n_j, dist_from_start, dist_to_end, curr.ptr);

                        // if there is a neighbor previously evaluated
                        if (points[n_idx].i >= 0 && points[n_idx].j >= 0)
                        {
                            // overwrite the current neighbor if the current neighbor's distance_from_start is smaller
                            if (points[n_idx].dist_from_start > dist_from_start)
                            {
                                auto handle = points[n_idx].handle;
                                points[n_idx] = new_p;
                                points[n_idx].handle = handle;

                                if (points[n_idx].handle_available) // if neighbor still in the heap, update
                                {
                                    min_heap.decrease(points[n_idx].handle);
                                }
                                else // if neighbor is previously evaluated and not in the heap, add to heap for new evaluation
                                {
                                    points[n_idx].handle = min_heap.push(AstarCellPtr(&(points[n_idx])));
                                    points[n_idx].handle_available = true;
                                }
                                
                            }
                        }
                        else
                        {
                            points[n_idx] = new_p;
                            points[n_idx].handle = min_heap.push(AstarCellPtr(&(points[n_idx])));
                            points[n_idx].handle_available = true;
                        }
                    }
                }
            }

            if (goal_reached)
            {
                path_out.clear();

                // backtrack
                int idx = end_i * m_cols + end_j;
                AstarCell *cur = &points[idx];
                path_out.push_back(std::make_pair(end_i, end_j));
                while (cur->came_from != nullptr)
                {
                    cur = cur->came_from;
                    path_out.push_back(std::make_pair(cur->i, cur->j));
                }
                std::reverse(path_out.begin(), path_out.end());
            }

            min_heap.clear();
            points.clear();
            return goal_reached;
        }
    };

} // astar_gridmap