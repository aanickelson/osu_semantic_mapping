import interval_cust


class TwoRoomsParameters:
    # "node name": [interval of x range, interval y range]
    nodes_dict = {"'r00'": [interval_cust.Interval(-10.4, -5.9), interval_cust.Interval(0, -5)],
                  "'r01'": [interval_cust.Interval(0, 5), interval_cust.Interval(0, -5)],
                  "'h00'": [interval_cust.Interval(-5.9, 0), interval_cust.Interval(-1.5, -3.2)],
                  "'r00_d00a'": [interval_cust.Interval(-6.1, -5.9), interval_cust.Interval(-2, -2.9)],
                  "'r00_d00b'": [interval_cust.Interval(-5.9, -5.7), interval_cust.Interval(-2, -2.9)],
                  "'r01_d00a'": [interval_cust.Interval(0, 0.2), interval_cust.Interval(-2, -2.9)],
                  "'r01_d00b'": [interval_cust.Interval(-0.2, 0), interval_cust.Interval(-2, -2.9)]}

    num_rooms = 2
    num_halls = 1

    extra_doors = None
    extra_door_hall_links = None
    # Associates rooms (door 0) with the adjacent hall - additional doors handled further down
    hall0_rooms = [0, 1]
    hall_door_links = [hall0_rooms]

    connected_halls = None
