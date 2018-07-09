'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          bfs_engine.py
   By:            Qin Chen
   Last Updated:  6/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
import sys
import Queue

class BFS(object):
    def __init__(self, graph):
        self.graph = graph
        return

    ######################################################
    # this function returns the shortest path for given start and goal nodes
    ######################################################
    def bfs_shortest_path(self, start, goal):
        parents = {}
        closed = []
        q = Queue.Queue()
        node = start
        path = []
        depth = 0
        counter = 0

        
        while node != goal and counter < 500:
            counter += 1
            # print("Doing node: " + str(node))
            # print("Closed: " + str(closed))
            if not node in closed: 
                closed.append(node)
                
                # print(path)
                for cNode in self.graph[node]:
                    q.put(cNode)
                    self.graph[node] = self.graph[node] - set(node)
                    if cNode not in parents.keys():
                        parents[cNode] = node
            # parent = node
            node = q.get()
            # print("Node: " + str(node) + '\t'+ "Parent: " + str(parent))
            # parents[node] = parent
        # print(parents)
        pNode = goal
        path.append(pNode)
        if counter >= 499 or goal not in parents.keys():
            return []
        while pNode != start:
            pNode = parents[pNode]
            path = [pNode] + path

        return path


    ######################################################
    # this function returns all paths for given start and goal nodes
    ######################################################
    def bfs_paths(self, start, goal):
        parents = {}
        closed = []
        q = Queue.Queue()
        node = start
        path = []
        depth = 0
        counter = 0
        
        while (not q.empty() or counter < 4) and counter < 500:
            
            counter += 1
            # print("Doing node: " + str(node))
            # print("Closed: " + str(closed))
            if not node in closed: 
                closed.append(node)
                
                # print(path)
                for cNode in self.graph[node]:
                    if cNode not in closed:
                        q.put(cNode)
                    self.graph[node] = self.graph[node] - set(node)
                    if cNode in parents.keys():
                        parents[cNode].append(node)
                    else:
                        parents[cNode] = [node]
            # parent = node
            node = q.get()
            # print("Node: " + str(node) + '\t'+ "Parent: " + str(parent))
            # parents[node] = parent
        # print(parents)
        print(parents)
        pNode = goal
        path.append(pNode)
        if counter >= 499 or goal not in parents.keys():
            return []

        paths = []
        pQ = Queue.Queue()
        pQ.put([goal])
        counter = 0
        while not pQ.empty() and counter < 500:
            counter += 1
            # print("PATH WHILE LOOP")
            possPath = pQ.get()
            cNode = possPath[0]
            print(possPath)
            if cNode[0] == start:
                paths.append(possPath)
                continue

            for ppNode in parents[cNode]:
                # print("PPNODE LOOP")
                if ppNode not in possPath:
                    pQ.put([ppNode] + possPath)

        return paths
                
    #########################################################
    # This function returns the shortest paths for given list of paths
    #########################################################
    def shortest(self, paths):
        shortest = paths[0]
        for path in paths:
            if len(path) < len(shortest):
                shortest = path

        return shortest

    #########################################################
    # THis function traverses the graph from given start node
    # return order of nodes visited
    #########################################################
    def bfs(self, start):
        q = Queue.Queue()
        node = start
        path = []
        q.put(node)
        counter = 0
        while not q.empty() and counter < 500:
            counter += 1
            node = q.get()
            # print("Doing node: " + str(node))
            path.append(node)
            # print(path)
            for cNode in self.graph[node]:
                
                q.put(cNode)
                if cNode in self.graph.keys():
                    if node in self.graph[cNode]:
                        self.graph[cNode].remove(node)
        
        return path

def main():
    graph = {'A': set(['B', 'C']),
         'B': set(['A', 'E', 'D']),
         'C': set(['A', 'F', 'G']),
         'D': set(['B', 'H']),
         'E': set(['B','I', 'J']),
         'F': set(['C','K']),
         'G': set(['C']),
         'H': set(['D']),
         'I': set(['E']),
         'J': set(['E']),
         'K': set(['F','J'])}

    bfs = BFS(graph)

    start_node = 'A'
    end_node = ''

    # order = bfs.bfs(start_node)
    # print "\n##########traverse order:", order

    # bfs.graph = graph
    # path = bfs.bfs_shortest_path('A', 'E')
    # print(path)

    paths = bfs.bfs_paths('A', 'J')
    # print(paths)
    # print(bfs.shortest(paths))
    return


def test():
    graph = {'A': set(['B', 'C']),
        'B': set(['A', 'C', 'D', 'E']),
        'C': set(['A', 'B', 'D', 'G']),
        'D': set(['B', 'C', 'E', 'G']),
        'E': set(['B', 'D', 'F', 'G']),
        'F': set(['E','G']),
        'G': set(['C', 'D', 'E', 'F'])}
    bfs = BFS(graph)
    start_node = 'A'
    end_node = 'G'

    p = bfs.bfs_shortest_path(start_node, end_node)
    print "\n++++++++++Shortest path from %s to %s: %s\n" % (start_node, end_node, p)
    
    #find all the paths returned by bfs_paths()
    paths = list(bfs.bfs_paths(start_node, end_node)) # [['A', 'C', 'F'], ['A', 'B', 'E', 'F']]
    print "\n==========paths from %s to %s: %s\n" % (start_node, end_node, paths)
    print len(paths)
    print "\n----------shortest path: %s\n" % bfs.shortest(paths)

    # order holds traverse order of the all the nodes
    order = bfs.bfs(start_node)
    print "\n##########traverse order:", order

if __name__ == "__main__":
    sys.exit(test())