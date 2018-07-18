class DataProblem():
    """Stores the data for the problem"""
    def __init__(self, num_vehicles, depot, orders, maximum_distance, maximum_parcels, distance_calculation, max_cod):
        """Initializes the data for the problem"""
        self._num_vehicles = num_vehicles
        self._maximum_distance = maximum_distance
        self._maximum_parcels = maximum_parcels
        
        # flatten the orders into a number of pairs
        # order:[pickup, deli1, deli2]
        # -> locations: [pickup, deli1, pickup, deli2]
        self._locations = depot
        # a dictionary map the index in locations to the original "orders" array
        self._orders_index = {}
        for index, pair in enumerate(orders):
            for i in range(1, len(pair)):
                self._locations.append(pair[0][0:2] + [pair[i][2]])
                self._orders_index[len(self._locations) - 1] = str(index) + "-" + str(0)
                self._locations.append(pair[i])
                self._orders_index[len(self._locations) - 1] = str(index) + "-" + str(i)
        self._depot = 0
        self._distance_calculation = distance_calculation
        self._max_cod = max_cod

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def locations(self):
        return self._locations

    @property
    def orders_index(self):
        return self._orders_index

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self._locations)

    @property
    def depot(self):
        """Gets depot location index"""
        return self._depot

    @property
    def maximum_distance(self):
        return self._maximum_distance

    @property
    def maximum_parcels(self):
        return self._maximum_parcels

    @property
    def distance_calculation(self):
        return self._distance_calculation

    @property
    def max_cod(self):
        return self._max_cod