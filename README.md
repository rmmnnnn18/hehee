# Graph Algorithms - Exam Ready Notes

## 1. Graph Terminology

### Core Definitions
- **Graph G = (V, E)**: Collection of vertices V and edges E
- **Vertex (Node)**: Fundamental unit representing an entity
- **Edge**: Connection between two vertices
- **Adjacent Vertices**: Two vertices directly connected by an edge
- **Degree of Vertex**: Number of edges incident to that vertex
  - **In-degree**: Incoming edges (directed graphs)
  - **Out-degree**: Outgoing edges (directed graphs)

### Graph Types
- **Directed Graph**: Edges have direction (one-way connections)
- **Undirected Graph**: Edges are bidirectional
- **Weighted Graph**: Edges have associated costs/weights
- **Unweighted Graph**: All edges have equal weight (usually 1)
- **Connected Graph**: Path exists between every pair of vertices
- **Sparse Graph**: Few edges relative to total possible edges
- **Dense Graph**: Many edges relative to total possible edges

### Additional Terms
- **Path**: Sequence of vertices connected by edges
- **Cycle**: Path that starts and ends at the same vertex
- **Loop**: Edge connecting vertex to itself
- **Parallel Edges**: Multiple edges between same pair of vertices

---

## 2. Graph Representations

### Adjacency Matrix
- **Structure**: 2D array of size V×V
- **Storage**: `adj[i][j] = 1` if edge exists between vertices i and j, `0` otherwise
- **Weighted**: Store weight instead of 1
- **Undirected**: Matrix is symmetric (adj[i][j] = adj[j][i])

**Properties:**
- **Space Complexity**: O(V²)
- **Time Complexity**: O(1) to check if edge exists
- **Best for**: Dense graphs, frequent edge queries

### Adjacency List
- **Structure**: Array of lists, size V
- **Storage**: `adjList[i]` contains list of vertices adjacent to vertex i
- **Weighted**: Store pairs (vertex, weight) in lists

**Properties:**
- **Space Complexity**: O(V + E)
- **Time Complexity**: O(degree) to check if edge exists
- **Best for**: Sparse graphs, memory efficiency

### Comparison Table
| Aspect | Adjacency Matrix | Adjacency List |
|--------|------------------|----------------|
| Space | O(V²) | O(V + E) |
| Add Edge | O(1) | O(1) |
| Remove Edge | O(1) | O(degree) |
| Edge Query | O(1) | O(degree) |
| Memory Usage | High for sparse | Low for sparse |

---

## 3. Breadth-First Search (BFS)

### Concept
- Explores graph **level by level** (breadth-first)
- Uses **queue** data structure
- Visits all neighbors before moving to next level
- Finds **shortest path** in unweighted graphs

### Pseudocode
```
BFS(Graph G, start_vertex s):
    visited = empty set
    queue = empty queue
    
    queue.enqueue(s)
    visited.add(s)
    
    while queue is not empty:
        current = queue.dequeue()
        process(current)
        
        for each neighbor v of current:
            if v not in visited:
                queue.enqueue(v)
                visited.add(v)
```

### Dry Run Example
**Graph**: 0-1-2, 0-3, 1-4
- Start: 0 → Queue: [0], Visited: {0}
- Visit 0 → Queue: [1,3], Visited: {0,1,3}
- Visit 1 → Queue: [3,2,4], Visited: {0,1,3,2,4}
- Visit 3 → Queue: [2,4], Visited: {0,1,3,2,4}
- Continue until queue empty

### Complexity
- **Time**: O(V + E) - visit each vertex once, check each edge once
- **Space**: O(V) - queue and visited set storage

### Applications
- Shortest path in unweighted graphs
- Level-order traversal
- Finding connected components
- Social network analysis (degrees of separation)

---

## 4. Depth-First Search (DFS)

### Concept
- Explores as **deep as possible** before backtracking
- Uses **stack** data structure (or recursion)
- Visits one path completely before trying alternatives

### Pseudocode (Recursive)
```
DFS(Graph G, vertex v):
    visited.add(v)
    process(v)
    
    for each neighbor u of v:
        if u not in visited:
            DFS(G, u)
```

### Pseudocode (Iterative)
```
DFS(Graph G, start_vertex s):
    visited = empty set
    stack = empty stack
    
    stack.push(s)
    
    while stack is not empty:
        current = stack.pop()
        if current not in visited:
            visited.add(current)
            process(current)
            
            for each neighbor v of current:
                if v not in visited:
                    stack.push(v)
```

### Complexity
- **Time**: O(V + E)
- **Space**: O(V) - recursion stack or explicit stack

### Applications
- Cycle detection
- Topological sorting
- Finding strongly connected components
- Maze solving
- Tree/forest identification

---

## 5. Kruskal's Algorithm (Minimum Spanning Tree)

### Concept
- **Greedy algorithm** for finding MST
- Sorts edges by weight, adds minimum weight edges avoiding cycles
- Uses **Union-Find** data structure for cycle detection
- Results in tree with V-1 edges connecting all vertices

### Steps
1. Sort all edges in ascending order of weight
2. Initialize empty MST
3. For each edge (u,v) in sorted order:
   - If adding edge doesn't create cycle: add to MST
   - Use Union-Find to detect cycles
4. Stop when MST has V-1 edges

### Pseudocode
```
KRUSKAL(Graph G):
    MST = empty set
    
    for each vertex v in G:
        MAKE-SET(v)
    
    sort G.edges by weight in ascending order
    
    for each edge (u,v) in sorted edges:
        if FIND-SET(u) ≠ FIND-SET(v):
            MST.add((u,v))
            UNION(u, v)
    
    return MST
```

### Union-Find Operations
- **MAKE-SET(v)**: Create singleton set {v}
- **FIND-SET(v)**: Return representative of set containing v
- **UNION(u,v)**: Merge sets containing u and v

### Example
**Graph**: Edges (A,B,4), (A,C,2), (B,C,1), (B,D,5), (C,D,3)
1. Sort: (B,C,1), (A,C,2), (C,D,3), (A,B,4), (B,D,5)
2. Add (B,C,1): MST = {(B,C)}
3. Add (A,C,2): MST = {(B,C), (A,C)}
4. Add (C,D,3): MST = {(B,C), (A,C), (C,D)}
5. Skip (A,B,4) - creates cycle
6. Result: MST weight = 1+2+3 = 6

### Complexity
- **Time**: O(E log E) - dominated by edge sorting
- **Space**: O(V) - Union-Find data structure

---

## 6. Dijkstra's Algorithm (Shortest Path)

### Concept
- Finds **shortest path from source to all vertices**
- Works only with **non-negative weights**
- **Greedy approach**: always picks closest unvisited vertex
- Uses **priority queue** (min-heap) for efficiency

### Steps
1. Initialize distances: source = 0, all others = ∞
2. Add all vertices to priority queue
3. While queue not empty:
   - Extract vertex u with minimum distance
   - For each neighbor v of u:
     - Calculate alt = distance[u] + weight(u,v)
     - If alt < distance[v]: update distance[v], previous[v]

### Pseudocode
```
DIJKSTRA(Graph G, source s):
    for each vertex v in G:
        distance[v] = INFINITY
        previous[v] = NULL
    
    distance[s] = 0
    Q = all vertices in G
    
    while Q is not empty:
        u = vertex in Q with minimum distance
        remove u from Q
        
        for each neighbor v of u:
            alt = distance[u] + weight(u,v)
            if alt < distance[v]:
                distance[v] = alt
                previous[v] = u
    
    return distance[], previous[]
```

### Table Method Example
**Graph**: A→B(4), A→C(2), B→C(1), B→D(5), C→D(8), source=A

| Step | Current | A | B | C | D | Queue |
|------|---------|---|---|---|---|-------|
| 0 | - | 0 | ∞ | ∞ | ∞ | A,B,C,D |
| 1 | A | 0 | 4 | 2 | ∞ | B,C,D |
| 2 | C | 0 | 3 | 2 | 10 | B,D |
| 3 | B | 0 | 3 | 2 | 8 | D |
| 4 | D | 0 | 3 | 2 | 8 | - |

### Priority Queue Implementation
- Use min-heap for O(log V) extract-min operations
- Store (distance, vertex) pairs
- Update distances by inserting new pairs (old entries ignored)

### Complexity
- **Time**: O((V + E) log V) with binary heap, O(V² + E) with array
- **Space**: O(V) for distance and previous arrays

### Applications
- GPS navigation systems
- Network routing protocols
- Social networks (shortest connection path)
- Game pathfinding

---

## 7. Algorithm Comparison Chart

| Algorithm | Purpose | Graph Type | Time Complexity | Space Complexity | Data Structure |
|-----------|---------|------------|----------------|------------------|----------------|
| **BFS** | Traversal/Shortest Path | Unweighted | O(V + E) | O(V) | Queue |
| **DFS** | Traversal/Cycle Detection | Any | O(V + E) | O(V) | Stack/Recursion |
| **Kruskal's** | Minimum Spanning Tree | Weighted, Undirected | O(E log E) | O(V) | Union-Find + Sort |
| **Dijkstra's** | Single-Source Shortest Path | Weighted (≥0) | O((V+E) log V) | O(V) | Priority Queue |

### When to Use
- **BFS**: Shortest path in unweighted graphs, level-wise processing
- **DFS**: Cycle detection, topological sort, connected components
- **Kruskal's**: Find minimum cost to connect all vertices (MST)
- **Dijkstra's**: Find shortest paths with weights (GPS, networking)

### Key Differences
- **BFS vs DFS**: BFS uses queue (breadth), DFS uses stack (depth)
- **Kruskal's vs Dijkstra's**: Kruskal's finds MST, Dijkstra's finds shortest paths
- **MST vs Shortest Path**: MST connects all vertices minimally, shortest path finds optimal routes

---

## Quick Reference Formulas

- **Complete Graph Edges**: E = V(V-1)/2 (undirected), V(V-1) (directed)
- **Tree Property**: Connected graph with V vertices has exactly V-1 edges
- **MST Property**: Any MST of graph with V vertices has exactly V-1 edges
- **Handshaking Lemma**: Sum of all vertex degrees = 2E (undirected graphs)

---

*Remember: Practice tracing algorithms by hand, understand when each algorithm applies, and memorize complexity formulas for exam success!*
