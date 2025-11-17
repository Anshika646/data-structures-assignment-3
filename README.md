# data-structures-assignment-3

#   BUILDING DATA CLASS BY ANSHIKA 24101420044


class Building:
    def __init__(self, BuildingID, BuildingName, LocationDetails):
        self.BuildingID = BuildingID     
        self.BuildingName = BuildingName
        self.LocationDetails = LocationDetails


#   BINARY SEARCH TREE (BST)
class BSTNode:
    def __init__(self, data):
        self.data = data      
        self.left = None
        self.right = None
class BinarySearchTree:
    def __init__(self):
        self.root = None
    # Insert building in BST
    def insertBuilding(self, data):
        self.root = self._insert(self.root, data)
    def _insert(self, root, data):
        if root is None:
            return BSTNode(data)

        if data.BuildingID < root.data.BuildingID:
            root.left = self._insert(root.left, data)
        else:
            root.right = self._insert(root.right, data)

        return root
    # Search
    def search(self, key):
        return self._search(self.root, key)

    def _search(self, root, key):
        if root is None:
            return None
        if key == root.data.BuildingID:
            return root.data
        if key < root.data.BuildingID:
            return self._search(root.left, key)
        else:
            return self._search(root.right, key)

    # Traversals
    def inorder(self, root):
        if root:
            self.inorder(root.left)
            print(root.data.BuildingName, end=" → ")
            self.inorder(root.right)

    def preorder(self, root):
        if root:
            print(root.data.BuildingName, end=" → ")
            self.preorder(root.left)
            self.preorder(root.right)

    def postorder(self, root):
        if root:
            self.postorder(root.left)
            self.postorder(root.right)
            print(root.data.BuildingName, end=" → ")

#   AVL TREE
class AVLNode:
    def __init__(self, data):
        self.data = data
        self.left = None
        self.right = None
        self.height = 1

class AVLTree:
    def __init__(self):
        self.root = None

    def insertBuilding(self, data):
        self.root = self._insert(self.root, data)

    # Height helper
    def getHeight(self, root):
        if not root:
            return 0
        return root.height

    # Balance factor
    def getBalance(self, root):
        if not root:
            return 0
        return self.getHeight(root.left) - self.getHeight(root.right)

    # Right rotate (LL case)
    def rightRotate(self, z):
        y = z.left
        T3 = y.right

        y.right = z
        z.left = T3

        z.height = 1 + max(self.getHeight(z.left), self.getHeight(z.right))
        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))

        print("LL Rotation (Right Rotate)")
        return y

    # Left rotate (RR case)
    def leftRotate(self, z):
        y = z.right
        T2 = y.left

        y.left = z
        z.right = T2

        z.height = 1 + max(self.getHeight(z.left), self.getHeight(z.right))
        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))

        print("RR Rotation (Left Rotate)")
        return y

    def _insert(self, root, data):
        if not root:
            return AVLNode(data)

        if data.BuildingID < root.data.BuildingID:
            root.left = self._insert(root.left, data)
        else:
            root.right = self._insert(root.right, data)

        root.height = 1 + max(self.getHeight(root.left), self.getHeight(root.right))

        balance = self.getBalance(root)

        # 4 Cases
        if balance > 1 and data.BuildingID < root.left.data.BuildingID:
            return self.rightRotate(root)  # LL

        if balance < -1 and data.BuildingID > root.right.data.BuildingID:
            return self.leftRotate(root)   # RR

        if balance > 1 and data.BuildingID > root.left.data.BuildingID:
            root.left = self.leftRotate(root.left)  # LR
            print("LR Rotation")
            return self.rightRotate(root)

        if balance < -1 and data.BuildingID < root.right.data.BuildingID:
            root.right = self.rightRotate(root.right)  # RL
            print("RL Rotation")
            return self.leftRotate(root)

        return root

    # Traversal for display
    def inorder(self, root):
        if root:
            self.inorder(root.left)
            print(root.data.BuildingName, end=" → ")
            self.inorder(root.right)

#   GRAPH (Adjacency Matrix + List)
class Graph:
    def __init__(self, n):
        self.n = n
        self.matrix = [[0]*n for _ in range(n)]     
        self.adjList = [[] for _ in range(n)]       

    # Add edge
    def addEdge(self, u, v, w):
        self.matrix[u][v] = w
        self.matrix[v][u] = w
        self.adjList[u].append((v, w))
        self.adjList[v].append((u, w))

    # BFS
    def BFS(self, start):
        visited = [False]*self.n
        queue = [start]
        visited[start] = True
        print("\nBFS Traversal:", end=" ")

        while queue:
            node = queue.pop(0)
            print(node, end=" ")

            for neigh, _ in self.adjList[node]:
                if not visited[neigh]:
                    visited[neigh] = True
                    queue.append(neigh)

    # DFS
    def DFS(self, start):
        visited = [False]*self.n
        print("\nDFS Traversal:", end=" ")

        def dfs(node):
            visited[node] = True
            print(node, end=" ")
            for neigh, _ in self.adjList[node]:
                if not visited[neigh]:
                    dfs(neigh)

        dfs(start)

# ============================
#   DIJKSTRA – SHORTEST PATH
# ============================
import heapq
def dijkstra(graph, start):
    dist = [float('inf')] * graph.n
    dist[start] = 0
    pq = [(0, start)]

    while pq:
        d, u = heapq.heappop(pq)

        for v, w in graph.adjList[u]:
            if dist[v] > d + w:
                dist[v] = d + w
                heapq.heappush(pq, (dist[v], v))

    print("\nDijkstra Distance from", start, ":", dist)


# ============================
#   KRUSKAL – MINIMUM SPANNING TREE
# ============================

def kruskal(graph):
    edges = []
    for u in range(graph.n):
        for v, w in graph.adjList[u]:
            if u < v:
                edges.append((w, u, v))

    edges.sort()
    parent = [i for i in range(graph.n)]

    def find(x):
        while x != parent[x]:
            x = parent[x]
        return x

    def union(a, b):
        parent[find(a)] = find(b)

    mst = []
    for w, u, v in edges:
        if find(u) != find(v):
            mst.append((u, v, w))
            union(u, v)

    print("\nKruskal MST:", mst)


# ============================
#   EXPRESSION TREE (ENERGY BILL)
# ============================

class ExpNode:
    def __init__(self, val):
        self.val = val
        self.left = None
        self.right = None


def evaluateExpression(root):
    if root.val.isdigit():
        return int(root.val)
    left = evaluateExpression(root.left)
    right = evaluateExpression(root.right)
    if root.val == '+': return left + right
    if root.val == '-': return left - right
    if root.val == '*': return left * right
    if root.val == '/': return left / right



#   MAIN – COMPLETE WORKING
if __name__ == "__main__":
    print("\n=== PART 1: BST + AVL ===")
    bst = BinarySearchTree()
    avl = AVLTree()
    buildings = [
        Building(10, "Admin Block", "Central Campus"),
        Building(5, "Library", "North Wing"),
        Building(15, "Computer Dept", "East Block"),
        Building(12, "Science Dept", "South Block")
    ]
    for b in buildings:
        bst.insertBuilding(b)
        avl.insertBuilding(b)

    print("\nBST Inorder: ", end="")
    bst.inorder(bst.root)

    print("\nAVL Inorder: ", end="")
    avl.inorder(avl.root)

    print("\n\n=== PART 2: GRAPH ===")

    g = Graph(4)
    g.addEdge(0, 1, 10)
    g.addEdge(1, 2, 5)
    g.addEdge(2, 3, 2)
    g.addEdge(0, 3, 15)

    g.BFS(0)
    g.DFS(0)

    dijkstra(g, 0)
    kruskal(g)

    print("\n\n=== EXPRESSION TREE ===")

    # Expression: (3 + 2) * 4
    root = ExpNode('*')
    root.left = ExpNode('+')
    root.right = ExpNode('4')
    root.left.left = ExpNode('3')
    root.left.right = ExpNode('2')

    print("Energy Bill Expression Result:", evaluateExpression(root))
