import networkx as nx
from networkx.drawing.nx_agraph import graphviz_layout
import matplotlib.pyplot as plt
import numpy as np
import pickle


#################################################
#################################################
### DO NOT UPDATE THIS ONE - ONLY FOR TESTING ###
#################################################
#################################################

"""
Rooms are in the format 'r##'
Doors are 'r##_d##a' room, room #, door, door #, node A or B
Halls are 'h##'
"""

class HospitalGraph:
    def __init__(self, num_rooms=None, num_halls=None, extra_doors=None,
                 hall_door_links=None, hall_room_extra=None, connected_halls=None):
        """

        Args:
            num_rooms: Integer for number of rooms
            num_halls: Integer for number of halls
            extra_doors: List of rooms with extra doors - list a room once for each extra door, e.g. ['r2', 'r2', 'r3']
            hall_door_links: List of which primary doors (d00) connect to which hall in tuple pairs, e.g. [('r02_d00b', 'h2')]
            hall_room_extra: List of which additional doors (d01+) connect to which halls in tuple pairs
            connected_halls: List of which halls connect together, e.g. [('h01', 'h02')]
        """
        self.G = nx.Graph()
        self.num_rooms = num_rooms              # Integer
        self.num_halls = num_halls              # Integer
        self.extra_doors = extra_doors          # List of rooms with extra doors - list a room once for each extra door
        self.rooms = None                       # Contains string representations of all room nodes
        self.halls = None                       # Contains string representations of all hall nodes
        self.doors = []                         # Contains string representations of all door nodes
        self.hall_room_links = hall_door_links
        self.hall_room_extra = hall_room_extra
        self.connected_halls = connected_halls
        self.draw_nodes()
        self.draw_edges()

    def draw_nodes(self):
        # Creates rooms in the format r## - numbers below 10 have a 0 appended in front for easier comparison
        self.rooms = ['r' + '%02d' % i for i in range(self.num_rooms)]

        # Creates one door with two nodes (node a & b)
        for rm in self.rooms:
            doors_a = rm + '_d00a'
            doors_b = rm + '_d00b'
            self.doors.append(doors_a)
            self.doors.append(doors_b)
            # self.G.add_edge(doors_a, doors_b)

        # Add extra doors to rooms with more than one
        if self.extra_doors:
            # Keep track of how many doors a room already has
            num_doors_per_rm = np.zeros(self.num_rooms)
            for room in self.extra_doors:
                rm = int(room[1:3])  # Get the room number
                num_doors_per_rm[rm] = num_doors_per_rm[rm] + 1  # Add additional door
                door_num = num_doors_per_rm[rm]  # Get door number
                new_nodes = ['r' + '%02d' % rm + '_d' + '%02d' % door_num + 'a',
                             'r' + '%02d' % rm + '_d' + '%02d' % door_num + 'b']  # Add two nodes to the door
                self.doors.extend(new_nodes)  # Add new nodes to the doors list
                self.G.add_edge(new_nodes[0], new_nodes[1])

        # Creates 5 hallways
        self.halls = ['h' + '%02d' % k for k in range(self.num_halls)]

        # Create nodes
        self.G.add_nodes_from(self.rooms)
        self.G.add_nodes_from(self.doors)
        self.G.add_nodes_from(self.halls)

    def draw_edges(self):

        for door in self.doors:

            # Add edges between interior ('a') door node and the associated room
            for r in range(self.num_rooms):
                room = 'r' + '%02d' % r
                # weight = r
                if room in door and 'a' in door:
                    self.G.add_edge(room, door)  #, weight=r)

            # Add edges between interior ('a') and exterior ('b') doors
            for door_b in self.doors:
                if door_b[:-1] in door and door_b is not door:
                    if not self.G.has_edge(door, door_b):
                        self.G.add_edge(door, door_b)

        # Add edges between halls and door 0 of adjacent rooms
        for num in range(self.num_halls):  # Loop through list of hall links
            hall_str = 'h' + '%02d' % num  # Get the string for the specific hallway
            for rm_num in self.hall_room_links[num]:
                door_to_add = 'r' + '%02d' % rm_num + '_d00' + 'b'
                self.G.add_edge(hall_str, door_to_add)  #, weight=num+rm_num)

        # Add edges for additional doors
        if self.hall_room_extra:
            self.G.add_edges_from(self.hall_room_extra)

        # Add edges between connected hallways
        if self.connected_halls:
            for (h0, h1) in self.connected_halls:
                hall_weight = int(h0[-2:]) + int(h1[-2:])
                self.G.add_edge(h0, h1)  #, weight=hall_weight)

    def plot_graph(self):
        # Plot the graph
        pos = graphviz_layout(self.G)
        nx.draw_networkx(self.G, pos, with_labels=False)
        labels = nx.get_edge_attributes(self.G, 'weight')
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=labels)
        # nx.draw(self.G, with_labels=True)

        plt.show()


def pickle_it(obj_to_pickle, filename):
    pickle.dump(obj_to_pickle, open(filename, 'wb'))


def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled


if __name__ == "__main__":
    # Define number of rooms and halls
    num_rooms = 3
    num_halls = 3

    extra_doors = ['r2']  # List room once for each additional set of doors.

    # Associates rooms (door 0) with the adjacent hall - additional doors handled further down
    hall0_rooms = [0]
    hall1_rooms = [1]
    hall2_rooms = [2]
    hall_door_links = [hall0_rooms, hall1_rooms, hall2_rooms]

    extra_door_hall_links = [('r02_d01b', 'h02')]  # Yes it's obnoxious notation but I couldn't think of a better way

    connected_halls = [('h00', 'h01'), ('h01', 'h02'), ('h02', 'h00')]  # Order irrelevant

    hospital = HospitalGraph(num_rooms, num_halls, extra_doors, hall_door_links, extra_door_hall_links, connected_halls)
    hospital.plot_graph()
