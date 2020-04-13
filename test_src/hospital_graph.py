import networkx as nx
import matplotlib.pyplot as plt
import math

G = nx.Graph()

num_rooms = 22
num_halls = 5

# Creates rooms
rooms = ['r' + str(i) for i in range(num_rooms)]

# Creates one door per room
doors = [rm + '_d0' for rm in rooms]
# Add extra doors to rooms with more than one
doors.insert(8, 'r7_d1')
doors.insert(18, 'r16_d1')
doors.insert(24, 'r21_d2')
doors.insert(24, 'r21_d1')

# Creates 5 hallways
halls = ['h' + str(j) for j in range(num_halls)]

G.add_nodes_from(rooms)
G.add_nodes_from(doors)
G.add_nodes_from(halls)

for door in doors:
    for room in rooms:
        # Makes comparison easier so r1 does not get included in r10_d0, for example
        room_ = room + "_"
        if room_ in door:
            G.add_edge(room, door)

hall0_rooms = [0, 13, 14, 15]
hall1_rooms = [1, 2, 3, 4, 5, 16, 17]
hall2_rooms = [6, 7]
hall3_rooms = [8, 9, 10, 11, 12, 20]
hall4_rooms = [18, 19, 21]
halls = [hall0_rooms, hall1_rooms, hall2_rooms, hall3_rooms, hall4_rooms]

for num in range(5):
    hall_str = 'h' + str(num)
    for rm_num in halls[num]:
        door_to_add = 'r' + str(rm_num) + '_d0'
        G.add_edge(hall_str, door_to_add)

G.add_edge('r7_d1', 'h2')
G.add_edge('r16_d1', 'h3')
G.add_edge('r21_d1', 'h4')
G.add_edge('r21_d2', 'h3')

G.add_edges_from([('h0', 'h1'), ('h0', 'h3'),
                  ('h1', 'h2'), ('h1', 'h3'), ('h1', 'h4'),
                  ('h2', 'h3'),
                  ('h3', 'h4')])

nx.spring_layout(G, k=5/math.sqrt(G.order()))  #, with_labels=True)
nx.draw_spring(G, with_labels=True)
plt.show()
