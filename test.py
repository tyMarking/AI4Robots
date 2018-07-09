from starter_bfs import *

def main():
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
    main()
