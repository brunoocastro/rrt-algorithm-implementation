# Description

[RRT (Rapidly exploring random tree)](https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree#:~:text=A%20rapidly%20exploring%20random%20tree,building%20a%20space%2Dfilling%20tree.) is an algorithm designed to efficiently search nonconvex, high-dimensional spaces by randomly building a space-filling tree.

On this repository we will implement it in a different ways, using Python and C++

# Pseudo-code algorithm

For a general configuration space C, the algorithm in pseudocode is as follows:

```pseudocode
Algorithm BuildRRT
    Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
    Output: RRT graph G

    G.init(qinit)
    for k = 1 to K do
        qrand ← RAND_CONF()
        qnear ← NEAREST_VERTEX(qrand, G)
        qnew ← NEW_CONF(qnear, qrand, Δq)
        G.add_vertex(qnew)
        G.add_edge(qnear, qnew)
    return G
```

"←" denotes assignment. For instance, "largest ← item" means that the value of largest changes to the value of item.
"return" terminates the algorithm and outputs the following value.

# Implementations

We already implemented pure RRT in Python, as you can [see here](/python/README.md)

We also [got a C++ implementation](https://github.com/nikhilchandak/Rapidly-Exploring-Random-Trees) and changed it to solve our problem. You can [see it here](/c++/README.md)