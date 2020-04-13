import networkx as nx

G = nx.Graph()
G.add_node(1)
G.add_nodes_from([2, 3])

H = nx.path_graph(10)

G.add_nodes_from(H)

G.add_node(H)

G.add_edge(1, 2)

e = (2, 3)
G.add_edge(*e)

G.add_edges_from([(1, 2), (1, 3)])

G.add_edge(4, 5, weight=[2, 8])

# print(G.number_of_edges(), G.number_of_nodes())
# print(list(G.nodes))
# print(list(G.edges))
# print(G.degree[1])

node0 = 4
node1 = 5
low_int = 0
high_int = 1
print(G[node0][node1])
# print(G[node0][node1]['object'][high_int])
