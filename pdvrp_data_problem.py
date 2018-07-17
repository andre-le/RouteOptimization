class DataProblem():
    """Stores the data for the problem"""
    def __init__(self, num_vehicles, depot, orders, maximum_distance, maximum_parcels, distance_calculation):
        """Initializes the data for the problem"""
        self._num_vehicles = num_vehicles
        self._maximum_distance = maximum_distance
        self._maximum_parcels = maximum_parcels
        
        # flatten the orders into a number of pairs
        # order:[pickup, deli1, deli2]
        # -> pairs: [pickup, deli1], [pickup, deli2]
        self._pairs = [depot]
        for pair in orders:
            for i in range(1, len(pair)):
                self._pairs.append([pair[0], pair[i]])
        print(self._pairs)
        self._depot = 0
        self._distance_calculation = distance_calculation

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def locations(self):
        locations = []
        for arr in self._pairs:
            for loc in arr:
                locations.append(loc)
        return locations

    @property
    def pairs(self):
        return self._pairs

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self.locations)

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