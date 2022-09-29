#ifndef __PATHFINDER_GRID_2D__
#define __PATHFINDER_GRID_2D__

// More info: https://www.boost.org/doc/libs/1_80_0/doc/html/heap/data_structures.html
// skew_heap, priority_queue, and d_ary_heap are not mutable by default. It is possible with the mutable_ interface but fibonacci_heap is probably will be faster.
#include <boost/heap/fibonacci_heap.hpp> // fastest for big grids. binomial_heap and pairing_heap also works but slow.
#include <boost/dynamic_bitset.hpp>
#include <memory>
#include <vector>
#include <cmath>

namespace pathfinder_grid_2d
{
    template <typename T> using heap_algorithm = boost::heap::fibonacci_heap<T>; // supports fibonacci_heap, binomial_heap, or pairing_heap. fibonacci_heap is faster!
    typedef int16_t idx_dtype; // always use signed integer types. if grid sizes are closer to 32768 change this type to int32_t

    struct NodePtr_;
    struct Node_
    {
        idx_dtype i, j;
        float dist_from_start, total_dist;
        Node_ *came_from;
        bool handle_available;
        heap_algorithm<NodePtr_>::handle_type handle;

        Node_() : Node_(-1, -1, -1.0, -1.0, nullptr) {}
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

    template <typename T>
    class PathFinderGrid2D
    {
    public:
        T *m_gridmap;
        boost::dynamic_bitset<> *m_gridmap_bitset;
        int m_rows, m_cols;
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
            heap_algorithm<NodePtr_> min_heap;
            std::vector<Node_> points;
            points.resize(m_rows * m_cols);

            float dist_start_to_end = std::hypot(start_i - end_i, start_j - end_j);
            Node_ p(start_i, start_j, 0.0, dist_start_to_end, nullptr);
            int idx = at(start_i, start_j);
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
                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        if (i == 0 && j == 0)
                            continue; // if tries to add itself
                        if (isFourNeighbors && i != 0 && j != 0)
                            continue;

                        // find neighbor positions
                        int n_i = curr.ptr->i + i;
                        int n_j = curr.ptr->j + j;
                        int n_idx = at(n_i, n_j);

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
                        float dist_to_end = (isDijkstra) ? (0) : std::hypot(n_i - end_i, n_j - end_j); // euclidian estimation
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
                                    //min_heap.update(points[n_idx].handle); // update or decrease doesn't matter for fibonacci
                                    //min_heap.decrease(points[n_idx].handle); // but for other methods decrease is faster
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
    };

} // pathfinder_grid2d

#endif // __PATHFINDER_GRID_2D__