from .hospital_graph_class import HospitalGraph
from random import randint
from .interval_cust import Interval as cust_interval
import networkx as nx
import interval

num_rooms = 10
num_halls = 9

extra_doors = None
extra_door_hall_links = None
# Associates rooms (door 0) with the adjacent hall - additional doors handled further down
hall0_rooms = [0, 1, 2, 3, 4]
hall1_rooms = []
hall2_rooms = []
hall3_rooms = []
hall4_rooms = []
hall5_rooms = []
hall6_rooms = []
hall7_rooms = []
hall8_rooms = [5, 6, 7, 8, 9]

hall_door_links = [hall0_rooms, hall1_rooms, hall2_rooms, hall3_rooms, hall4_rooms,
                   hall5_rooms, hall6_rooms, hall7_rooms, hall8_rooms]

connected_halls = [('h00', 'h01'), ('h00', 'h02'), ('h00', 'h03'),
                   ('h01', 'h04'), ('h02', 'h04'), ('h03', 'h04'),
                   ('h05', 'h04'), ('h06', 'h04'), ('h07', 'h04'),
                   ('h05', 'h08'), ('h06', 'h08'), ('h07', 'h08')]

hospital = HospitalGraph(num_rooms, num_halls, extra_doors, hall_door_links, extra_door_hall_links, connected_halls)
# hospital.plot_graph()

for (n1, n2) in hospital.G.edges():
    edge_weight = int(n1[1:3]) + int(n2[1:3])
    hospital.G[n1][n2]['weight'] = cust_interval(edge_weight, edge_weight+5)
    hospital.G[n1][n2]['lower'] = 1 / (edge_weight + 1)


# hospital.plot_graph()

print(nx.dijkstra_path(hospital.G, 'r03', 'r05'))
print(nx.dijkstra_path(hospital.G, 'r03', 'r05', weight='lower'))
