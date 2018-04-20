# https://developers.google.com/optimization/routing/tsp/tsp
import math
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


class CreateMatrixDistanceCallback(object):
    def __init__(self):
        self.matrix = [
            [   0, 2451,  713, 1018, 1631, 1374, 2408,  213, 2571,  875, 1420, 2145, 1972], # New York
            [2451,    0, 1745, 1524,  831, 1240,  959, 2596,  403, 1589, 1374,  357,  579], # Los Angeles
            [ 713, 1745,    0,  355,  920,  803, 1737,  851, 1858,  262,  940, 1453, 1260], # Chicago
            [1018, 1524,  355,    0,  700,  862, 1395, 1123, 1584,  466, 1056, 1280,  987], # Minneapolis
            [1631,  831,  920,  700,    0,  663, 1021, 1769,  949,  796,  879,  586,  371], # Denver
            [1374, 1240,  803,  862,  663,    0, 1681, 1551, 1765,  547,  225,  887,  999], # Dallas
            [2408,  959, 1737, 1395, 1021, 1681,    0, 2493,  678, 1724, 1891, 1114,  701], # Seattle
            [ 213, 2596,  851, 1123, 1769, 1551, 2493,    0, 2699, 1038, 1605, 2300, 2099], # Boston
            [2571,  403, 1858, 1584,  949, 1765,  678, 2699,    0, 1744, 1645,  653,  600], # San Francisco
            [ 875, 1589,  262,  466,  796,  547, 1724, 1038, 1744,    0,  679, 1272, 1162], # St. Louis
            [1420, 1374,  940, 1056,  879,  225, 1891, 1605, 1645,  679,    0, 1017, 1200], # Houston
            [2145,  357, 1453, 1280,  586,  887, 1114, 2300,  653, 1272, 1017,    0,  504], # Phoenix
            [1972,  579, 1260,  987,  371,  999,  701, 2099,  600, 1162,  1200,  504,   0]] # Salt Lake City

    def distance(self, from_node, to_node):
        return int(self.matrix[from_node][to_node])


class CreateHaversineDistanceCallback(object):
    def __init__(self):
        # Latitudes and longitudes of selected U.S. cities.
        locations = [[40.71, -74.01],  # New York
                     [34.05, -118.24],  # Los Angeles
                     [41.88, -87.63],  # Chicago
                     [44.98, -93.27],  # Minneapolis
                     [39.74, -104.99],  # Denver
                     [32.78, -96.89],  # Dallas
                     [47.61, -122.33],  # Seattle
                     [42.36, -71.06],  # Boston
                     [37.77, -122.42],  # San Francisco
                     [38.63, -90.20],  # St. Louis
                     [29.76, -95.37],  # Houston
                     [33.45, -112.07],  # Phoenix
                     [40.76, -111.89]]  # Salt Lake City

        # Create the distance matrix.
        size = len(locations)
        self.matrix = {}

        for from_node in range(size):
            self.matrix[from_node] = {}
            for to_node in range(size):
                if from_node == to_node:
                    self.matrix[from_node][to_node] = 0
                else:
                    x1 = locations[from_node][0]
                    y1 = locations[from_node][1]
                    x2 = locations[to_node][0]
                    y2 = locations[to_node][1]
                    self.matrix[from_node][to_node] = lat_long_dist(x1, y1, x2, y2)

    def distance(self, from_node, to_node):
        return int(self.matrix[from_node][to_node])


def haversine(angle):
    h = math.sin(angle / 2) ** 2
    return h


def lat_long_dist(lat1, long1, lat2, long2):
    # Mean radius of Earth in miles
    radius_earth = 3959

    # Convert latitude and longitude to spherical coordinates in radians.
    degrees_to_radians = math.pi / 180.0
    phi1 = lat1 * degrees_to_radians
    phi2 = lat2 * degrees_to_radians
    lambda1 = long1 * degrees_to_radians
    lambda2 = long2 * degrees_to_radians
    dphi = phi2 - phi1
    dlambda = lambda2 - lambda1

    a = haversine(dphi) + math.cos(phi1) * math.cos(phi2) * haversine(dlambda)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = radius_earth * c

    return d


def main():
    city_names = ["New York", "Los Angeles", "Chicago", "Minneapolis", "Denver", "Dallas", "Seattle",
                  "Boston", "San Francisco", "St. Louis", "Houston", "Phoenix", "Salt Lake City"]

    tsp_size = len(city_names)
    num_routes = 1  # The number of routes, which is 1 in the TSP.
    depot = 0  # The depot is the starting node of the route.

    # Create routing model
    routing = pywrapcp.RoutingModel(tsp_size, num_routes, depot)
    search_parameters = pywrapcp.RoutingModel_DefaultSearchParameters()

    # Use guided local search optimizer with time limit (this can be omitted)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit_ms = 5000

    # Create the distance callback, which takes two arguments (the from and to node indices)
    # and returns the distance between these nodes.
    dist_between_nodes = CreateHaversineDistanceCallback()
    dist_callback = dist_between_nodes.distance
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

    # Solve, returns a solution if any.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
        # Solution cost.
        print('Total distance: ' + str(assignment.ObjectiveValue()) + ' miles\n')
        # Inspect solution.
        # Only one route here; otherwise iterate from 0 to routing.vehicles() - 1
        route_number = 0
        index = routing.Start(route_number)  # Index of the variable for the starting node.
        route = ''
        while not routing.IsEnd(index):
            # Convert variable indices to node indices in the displayed route.
            route += str(city_names[routing.IndexToNode(index)]) + ' -> '
            index = assignment.Value(routing.NextVar(index))
        route += str(city_names[routing.IndexToNode(index)])
        print('Route:\n\n' + route)
    else:
        print('No solution found.')


if __name__ == "__main__":
    main()
