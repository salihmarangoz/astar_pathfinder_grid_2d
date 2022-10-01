// MIT License

// Copyright (c) 2022 Salih Marangoz

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __PATHFINDER_GRID_2D__
#define __PATHFINDER_GRID_2D__

// More info: https://www.boost.org/doc/libs/1_80_0/doc/html/heap/data_structures.html
// skew_heap, priority_queue, and d_ary_heap are not mutable by default. It is possible with the mutable_ interface but fibonacci_heap is probably will be faster.
#include <boost/heap/fibonacci_heap.hpp> // fastest for big grids. binomial_heap and pairing_heap also works but slow.
#include <boost/heap/priority_queue.hpp>
#include <boost/dynamic_bitset.hpp>
#include <memory>
#include <vector>
#include <cmath>

namespace pathfinder_grid_2d
{
    typedef int16_t idx_dtype; // always use signed integer types. if grid sizes are closer to 32768 change this type to int32_t
    typedef int32_t didx_dtype; // double size of idx_dtype

    struct Cell
    {
        idx_dtype i, j;
        Cell(idx_dtype i, idx_dtype j)
        {
            this->i = i;
            this->j = j;
        }
    };
    typedef std::vector<Cell> Path;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////// STABLE A-STAR GRID PATHFINDER ///////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename T> using pathfinder_heap_algorithm = boost::heap::fibonacci_heap<T>; // supports fibonacci_heap, binomial_heap, or pairing_heap. fibonacci_heap is faster!

    struct NodePtr_;
    struct Node_
    {
        idx_dtype i, j;
        float dist_from_start, total_dist;
        Node_ *came_from;
        bool handle_available;
        pathfinder_heap_algorithm<NodePtr_>::handle_type handle;

        Node_() : Node_(-1, -1, 0.0, 0.0, nullptr) {}
        Node_(idx_dtype i, idx_dtype j, float dist_from_start, float dist_to_end, Node_ *came_from)
        {
            this->i = i;
            this->j = j;
            this->dist_from_start = dist_from_start;
            this->total_dist = dist_from_start + dist_to_end;
            this->came_from = came_from;
            this->handle_available = false;
        }
    };

    struct NodePtr_
    {
        Node_ *ptr;
        NodePtr_(Node_ *ptr)
        {
            this->ptr = ptr;
        }

        bool operator<(NodePtr_ const &rhs) const
        {
            return ptr->total_dist > rhs.ptr->total_dist;
        }
    };

    template <typename T>
    class PathFinderGrid2D
    {
    public:
        T *m_gridmap;
        boost::dynamic_bitset<> *m_gridmap_bitset;
        idx_dtype m_rows, m_cols;
        bool gridmap_is_bitset;

        PathFinderGrid2D(T *gridmap, int rows, int cols)
            : m_gridmap(gridmap), m_rows(rows), m_cols(cols)
        {
            gridmap_is_bitset = false;
        }

        PathFinderGrid2D(boost::dynamic_bitset<> *gridmap_bitset, int rows, int cols)
            : m_gridmap_bitset(gridmap_bitset), m_rows(rows), m_cols(cols)
        {
            gridmap_is_bitset = true;
        }

        bool plan(int start_i, int start_j, int end_i, int end_j, Path &path_out, T threshold, bool obstacleHasHigherValue = false, bool isDijkstra = false, bool isFourNeighbors = false)
        {
            pathfinder_heap_algorithm<NodePtr_> min_heap;
            std::vector<Node_> points;
            points.resize(m_rows * m_cols);

            float dist_start_to_end = (isDijkstra) ? (0) : heuristic(start_i - end_i, start_j - end_j);
            Node_ p(start_i, start_j, 0.0, dist_start_to_end, nullptr);
            didx_dtype idx = at(start_i, start_j);
            points[idx] = p;
            points[idx].handle = min_heap.push(NodePtr_(&(points[idx])));

            bool goal_reached = false;
            while (min_heap.size() > 0)
            {
                // get Node_ with the lowest total distance value
                NodePtr_ curr = min_heap.top();
                min_heap.pop();
                curr.ptr->handle_available = false;

                // check end condition
                if (curr.ptr->i == end_i && curr.ptr->j == end_j)
                {
                    goal_reached = true;
                    break;
                }

                // add neighbors
                for (idx_dtype i = -1; i <= 1; i++)
                {
                    for (idx_dtype j = -1; j <= 1; j++)
                    {
                        if (i == 0 && j == 0)
                            continue; // if tries to add itself
                        if (isFourNeighbors && i != 0 && j != 0)
                            continue;

                        // find neighbor positions
                        idx_dtype n_i = curr.ptr->i + i;
                        idx_dtype n_j = curr.ptr->j + j;
                        didx_dtype n_idx = at(n_i, n_j);

                        if (!isInside(n_i, n_j))
                            continue; // if neighbor is outside of the costmap

                        if (gridmap_is_bitset)
                        {
                            if (obstacleHasHigherValue && m_gridmap_bitset->test(n_idx))
                                continue; // if neighbor's cost is high
                            if (!obstacleHasHigherValue && !m_gridmap_bitset->test(n_idx))
                                continue; // if neighbor's cost is high
                        }
                        else
                        {
                            if (obstacleHasHigherValue && m_gridmap[n_idx] >= threshold)
                                continue; // if neighbor's cost is high
                            if (!obstacleHasHigherValue && m_gridmap[n_idx] <= threshold)
                                continue; // if neighbor's cost is high
                            #ifdef DEBUG_PATHFINDER_GRID_2D
                            m_gridmap[n_idx] = DEBUG_PATHFINDER_GRID_2D;
                            #endif
                        }

                        float dist_from_start = curr.ptr->dist_from_start + ((i == 0 || j == 0) ? (1.0) : (M_SQRT2)); // optimized for 8-neighbor
                        float dist_to_end = (isDijkstra) ? (0) : heuristic(n_i - end_i, n_j - end_j);
                        Node_ new_p(n_i, n_j, dist_from_start, dist_to_end, curr.ptr);

                        // if there is a neighbor previously evaluated
                        if (points[n_idx].i >= 0 /* && points[n_idx].j >= 0 */)
                        {
                            
                            // overwrite the current neighbor if the current neighbor's distance_from_start is smaller
                            if (points[n_idx].dist_from_start > dist_from_start)
                            {
                                auto handle = points[n_idx].handle;
                                points[n_idx] = new_p;
                                points[n_idx].handle = handle;

                                if (points[n_idx].handle_available) // if neighbor still in the heap, update
                                {
                                    min_heap.update(points[n_idx].handle); // update or decrease doesn't matter for fibonacci. but for other methods decrease is faster
                                }
                                else // if neighbor is previously evaluated and not in the heap, add to heap for new evaluation
                                {
                                    points[n_idx].handle = min_heap.push(NodePtr_(&(points[n_idx])));
                                    points[n_idx].handle_available = true;
                                }
                                
                            }
                            
                        }
                        else
                        {
                            points[n_idx] = new_p;
                            points[n_idx].handle = min_heap.push(NodePtr_(&(points[n_idx])));
                            points[n_idx].handle_available = true;
                        }
                    }
                }
            }

            if (goal_reached)
            {
                // backtracking
                path_out.clear();
                Node_ *cur = &points[at(end_i, end_j)];
                path_out.push_back(Cell(end_i, end_j));
                while (cur->came_from != nullptr)
                {
                    cur = cur->came_from;
                    path_out.push_back(Cell(cur->i, cur->j));
                }
                std::reverse(path_out.begin(), path_out.end());
            }

            min_heap.clear();
            points.clear();
            return goal_reached;
        }

        bool isInside(idx_dtype i, idx_dtype j) const
        {
            if (i < m_rows && j < m_cols && i >= 0 && j >= 0)
                return true;
            else
                return false;
        }

        didx_dtype at(idx_dtype i, idx_dtype j) const
        {
            return i * m_cols + j;
        }

        float heuristic(float di, float dj)
        {
            return std::hypot(di, dj); // euclidean distance
            //return std::abs(di) + std::abs(dj); // manhattan distance. faster!
            // before putting another approximate heuristics here why dont you try FastPathFinderGrid2D if you need more speed instead of stable results?
        }
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////// FAST A-STAR GRID PATHFINDER /////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename T> using fast_pathfinder_heap_algorithm = boost::heap::priority_queue<T>; // since we will not update values priority_queue will be faster

    struct FastNodePtr_;
    struct FastNode_
    {
        idx_dtype i, j;
        uint32_t dist_from_start, total_dist;
        FastNode_ *came_from;

        FastNode_() : FastNode_(-1, -1, 0.0, 0.0, nullptr) {}
        FastNode_(idx_dtype i, idx_dtype j, uint32_t dist_from_start, uint32_t dist_to_end, FastNode_ *came_from)
        {
            this->i = i;
            this->j = j;
            this->dist_from_start = dist_from_start;
            this->total_dist = dist_from_start + dist_to_end;
            this->came_from = came_from;
        }
    };

    struct FastNodePtr_
    {
        FastNode_ *ptr;
        FastNodePtr_(FastNode_ *ptr)
        {
            this->ptr = ptr;
        }

        bool operator<(FastNodePtr_ const &rhs) const
        {
            return ptr->total_dist > rhs.ptr->total_dist;
        }
    };

    template <typename T>
    class FastPathFinderGrid2D
    {
    public:
        T *m_gridmap;
        boost::dynamic_bitset<> *m_gridmap_bitset;
        idx_dtype m_rows, m_cols;
        bool gridmap_is_bitset;

        FastPathFinderGrid2D(T *gridmap, int rows, int cols)
            : m_gridmap(gridmap), m_rows(rows), m_cols(cols)
        {
            gridmap_is_bitset = false;
        }

        FastPathFinderGrid2D(boost::dynamic_bitset<> *gridmap_bitset, int rows, int cols)
            : m_gridmap_bitset(gridmap_bitset), m_rows(rows), m_cols(cols)
        {
            gridmap_is_bitset = true;
        }

        bool plan(int start_i, int start_j, int end_i, int end_j, Path &path_out, T threshold, bool obstacleHasHigherValue = false, bool isDijkstra = false, bool isFourNeighbors = false)
        {
            fast_pathfinder_heap_algorithm<FastNodePtr_> min_heap;
            std::vector<FastNode_> points;
            points.resize(m_rows * m_cols);

            uint32_t dist_start_to_end = (isDijkstra) ? (0) : heuristic(start_i - end_i, start_j - end_j);
            FastNode_ p(start_i, start_j, 0, dist_start_to_end, nullptr);
            int idx = at(start_i, start_j);
            points[idx] = p;
            min_heap.push(FastNodePtr_(&(points[idx])));

            bool goal_reached = false;
            while (min_heap.size() > 0)
            {
                // get FastNode_ with the lowest total distance value
                FastNodePtr_ curr = min_heap.top();
                min_heap.pop();

                // check end condition
                if (curr.ptr->i == end_i && curr.ptr->j == end_j)
                {
                    goal_reached = true;
                    break;
                }

                // add neighbors
                for (idx_dtype i = -1; i <= 1; i++)
                {
                    for (idx_dtype j = -1; j <= 1; j++)
                    {
                        if (i == 0 && j == 0)
                            continue; // if tries to add itself
                        if (isFourNeighbors && i != 0 && j != 0)
                            continue;

                        // find neighbor positions
                        idx_dtype n_i = curr.ptr->i + i;
                        idx_dtype n_j = curr.ptr->j + j;
                        didx_dtype n_idx = at(n_i, n_j);

                        if (!isInside(n_i, n_j))
                            continue; // if neighbor is outside of the costmap

                        if (gridmap_is_bitset)
                        {
                            if (obstacleHasHigherValue && m_gridmap_bitset->test(n_idx))
                                continue; // if neighbor's cost is high
                            if (!obstacleHasHigherValue && !m_gridmap_bitset->test(n_idx))
                                continue; // if neighbor's cost is high
                        }
                        else
                        {
                            if (obstacleHasHigherValue && m_gridmap[n_idx] >= threshold)
                                continue; // if neighbor's cost is high
                            if (!obstacleHasHigherValue && m_gridmap[n_idx] <= threshold)
                                continue; // if neighbor's cost is high
#ifdef DEBUG_PATHFINDER_GRID_2D
                            m_gridmap[n_idx] = DEBUG_PATHFINDER_GRID_2D;
#endif
                        }

                        uint32_t dist_from_start = curr.ptr->dist_from_start + ((i == 0 || j == 0) ? (10) : (14)); // optimized for 8-neighbor
                        uint32_t dist_to_end = (isDijkstra) ? (0) : heuristic(n_i - end_i, n_j - end_j);
                        FastNode_ new_p(n_i, n_j, dist_from_start, dist_to_end, curr.ptr);

                        // if there is a neighbor previously evaluated
                        if (points[n_idx].i >= 0 /* && points[n_idx].j >= 0 */)
                        {
                            // overwrite the current neighbor if the current neighbor's distance_from_start is smaller
                            if (points[n_idx].dist_from_start > dist_from_start)
                            {
                                points[n_idx] = new_p;
                            }
                        }
                        else
                        {
                            points[n_idx] = new_p;
                            min_heap.push(FastNodePtr_(&(points[n_idx])));
                        }
                    }
                }
            }

            if (goal_reached)
            {
                // backtracking
                path_out.clear();
                FastNode_ *cur = &points[at(end_i, end_j)];
                path_out.push_back(Cell(end_i, end_j));
                while (cur->came_from != nullptr)
                {
                    cur = cur->came_from;
                    path_out.push_back(Cell(cur->i, cur->j));
                }
                std::reverse(path_out.begin(), path_out.end());
            }

            min_heap.clear();
            points.clear();
            return goal_reached;
        }

        bool isInside(idx_dtype i, idx_dtype j) const
        {
            if (i < m_rows && j < m_cols && i >= 0 && j >= 0)
                return true;
            else
                return false;
        }

        didx_dtype at(idx_dtype i, idx_dtype j) const
        {
            return i * m_cols + j;
        }

        uint32_t heuristic(idx_dtype di, idx_dtype dj)
        {
            //return (dj < di) ? (di + (dj >> 1)) : (dj + (di >> 1)); // euclidean distance approximation
            return 10 * (std::abs(di) + std::abs(dj)) + (-6) * std::min(std::abs(di), std::abs(dj)); // octagonal distance approximation
        }
    };

} // pathfinder_grid2d

#endif // __PATHFINDER_GRID_2D__
