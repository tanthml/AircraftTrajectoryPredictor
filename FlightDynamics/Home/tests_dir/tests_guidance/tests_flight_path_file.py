import unittest

from Home.Guidance.FlightPathFile import FlightPath

"""
Meter2Feet = 3.2808  # one meter equals 3.28 feet
Meter2NauticalMiles = 0.000539956803  # One Meter = 0.0005 nautical miles
NauticalMiles2Meter = 1852
"""
Knots2MetersPerSecond = 0.514444444


class TestFlightPathFile(unittest.TestCase):

    def setUp(self) -> None:
        route_a320_charles_de_gaulle_lisbonne = \
            'ADEP/LFPG/26R-LAIGLE-ROLEN-PEPON-KURIS-TERPO-ERIGA-INBAB-ATLEN-' \
            'DEVAR-ASTURIAS-KUVAN-BISMU-BARKO-FATIMA-ADES/LPPT/03'
        self.str_route = route_a320_charles_de_gaulle_lisbonne

    def test_flight_path(self):
        flight_path = FlightPath(
            route=self.str_route,
            aircraftICAOcode='A320',
            RequestedFlightLevel=330,
            cruiseMach=0.82,
            takeOffMassKilograms=68000.0,
            windSpeedMetersPerSecond=25*Knots2MetersPerSecond,
            windDirectionDegrees=25
        )

        expectation = []

        self.assertEqual(flight_path, expectation)

    def test_flight_path_compute_flight(self):
        flight_path = FlightPath(
            route=self.str_route,
            aircraftICAOcode='A320',
            RequestedFlightLevel=330,
            cruiseMach=0.82,
            takeOffMassKilograms=68000.0,
            windSpeedMetersPerSecond=25*Knots2MetersPerSecond,
            windDirectionDegrees=25
        )

        # If we want to generate the whole flight using the old code, use the .computeFlight function
        flight_path.computeFlight(deltaTimeSeconds=1.0)
        flight_path.createFlightOutputFiles()

    def test_flight_path_fly(self):
        flyAndGenerateConfig = FlightPath(
            route=self.str_route,
            aircraftICAOcode='A320',
            RequestedFlightLevel=330,
            cruiseMach=0.82,
            takeOffMassKilograms=68000.0,
            windSpeedMetersPerSecond=25 * Knots2MetersPerSecond,
            windDirectionDegrees=25
        )

        flyAndGenerateConfig.fly()

    def test_flight_path_simulate_fly(self):
        updateFlightPath = FlightPath(
            route=self.str_route,
            aircraftICAOcode='A320',
            RequestedFlightLevel=330,
            cruiseMach=0.82,
            takeOffMassKilograms=68000.0,
            windSpeedMetersPerSecond=25 * Knots2MetersPerSecond,
            windDirectionDegrees=25
        )

        # fly from index 4 for the new route
        updateFlightPath.simulateFly(4, self.str_route)
        # fly from index 4 for the new route, with speed 260 and altitude 10000
        updateFlightPath.simulateFly(4, self.str_route, 260, 10000.0)


if __name__ == '__main__':
    unittest.main()
