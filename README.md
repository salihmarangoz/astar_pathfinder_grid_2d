## PathFinder Grid 2D (A*)

Tested on Ubuntu 20.04.

## How to include in my project

You need the boost library. You can install it with this command:

```bash
$ sudo apt install libboost1.71-dev
```



## How to use path finding

```
```





## FAQ

- Why did you use Fibonacci Heap for stable path finder, and Priority Queue for fast path finder?
  - Priority Queue is very fast with push and pop operations. But if we want to implement A* correctly we need to update values in the heap. In this case, Fibonacci Heap comes to the rescue! For more: https://www.boost.org/doc/libs/1_80_0/doc/html/heap/data_structures.html
- Are A* heuristics that important?
  - Short answer: Yes. Selected heuristic affects the performance and the runtime of the algorithm significantly. For more: https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html




