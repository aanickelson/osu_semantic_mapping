import interval_cust


class TwoRoomsParameters:
    # "node name": [interval of x range, interval y range]
    nodes_dict = {"'r00'": [interval_cust.Interval(-1.0, 4.0), interval_cust.Interval(-1.0, 4.0)],
                  "'r01'": [interval_cust.Interval(8.0, 12.5), interval_cust.Interval(-1.0, 4.0)],
                  "'h00'": [interval_cust.Interval(4.3, 7.3), interval_cust.Interval(0.7, 2.2)],
                  "'r00_d00a'": [interval_cust.Interval(3.2, 3.7), interval_cust.Interval(0.7, 2.2)],
                  "'r00_d00b'": [interval_cust.Interval(3.7, 4.3), interval_cust.Interval(0.7, 2.2)],
                  "'r01_d00a'": [interval_cust.Interval(7.3, 7.8), interval_cust.Interval(0.7, 2.2)],
                  "'r01_d00b'": [interval_cust.Interval(7.8, 8.3), interval_cust.Interval(0.7, 2.2)]}

    num_rooms = 2
    num_halls = 1

    extra_doors = None
    extra_door_hall_links = None
    # Associates rooms (door 0) with the adjacent hall - additional doors handled further down
    hall0_rooms = [0, 1]
    hall_door_links = [hall0_rooms]

    connected_halls = None
