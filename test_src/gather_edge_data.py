from hospital_graph_class import HospitalGraph
from two_rooms_parameters import TwoRoomsParameters
import interval_cust


class RoboInfo:
    def __init__(self, p):
        self.current_loc = None
        self.previous_node = None
        self.p = p
        self.building = HospitalGraph(p.num_rooms, p.num_halls, p.extra_doors,
                                      p.hall_door_links, p.extra_door_hall_links, p.connected_halls)

def what_node(nodes_dict, loc):
    """
   Door nodes are checked first - they are within the bounds of rooms / halls so doors must supersede other areas

    Args:
        p: parameters
        loc: current x, y location to check

    Returns:
        node the robot is currently in

    """

    # Loop through once for doors
    for node in nodes_dict:
        if 'd' in node:
            if loc[0] in nodes_dict[node][0] and loc[1] in nodes_dict[node][1]:
                return node

    # Loop through a second time for all other areas
    for node in nodes_dict:
        if 'd' in node:
            continue
        else:
            if loc[0] in nodes_dict[node][0] and loc[1] in nodes_dict[node][1]:
                return node

    raise ValueError('Robot is out of this world')


if __name__ == "__main__":

    r = RoboInfo(p=TwoRoomsParameters)
    r.current_loc = [2, -3]
    print(what_node(TwoRoomsParameters.nodes_dict, r.current_loc))

    r.current_loc = [-2, -2]
    print(what_node(TwoRoomsParameters.nodes_dict, r.current_loc))

    r.current_loc = [2, 2]
    print(what_node(TwoRoomsParameters.nodes_dict, r.current_loc))



