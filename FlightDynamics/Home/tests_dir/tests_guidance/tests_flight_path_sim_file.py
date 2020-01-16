import unittest

from Home.Guidance.FlightPathSimFile import FlightPathSim


Knots2MetersPerSecond = 0.514444444


class TestFlightPathSimFile(unittest.TestCase):

    def setUp(self) -> None:
        route_a320_charles_de_gaulle_lisbonne = \
            'ADEP/LFPG/26R-LAIGLE-ROLEN-PEPON-KURIS-TERPO-ERIGA-INBAB-ATLEN-' \
            'DEVAR-ASTURIAS-KUVAN-BISMU-BARKO-FATIMA-ADES/LPPT/03'
        waypoints = 'ROLEN-PEPON-KURIS-TERPO-ERIGA-INBAB'
        waypoints = 'ROLEN-TERPO-ATLEN-DEVAR-BISMU-FATIMA-ADES/LPPT/03'
        waypoints = 'LAMSO-EVELI-BASNO-PAMPUS-IVLUT-LUNIX-RENDI-EDUPO-NAPRO-DEPAD-AMOSU-MISGO-COLA-ROLIS-ADES/EDDF/25C'
        waypoints = 'LAMSO-EVELI-BASNO-PAMPUS-ADES/EDDF/25C'
        self.str_route = waypoints

    def test_flight_path(self):
        flight_path = FlightPathSim(
            route=self.str_route,
            aircraftICAOcode='A320',
            RequestedFlightLevel=330,
            cruiseMach=0.82,
            takeOffMassKilograms=68000.0,
            windSpeedMetersPerSecond=25*Knots2MetersPerSecond,
            windDirectionDegrees=25
        )

        self.assertEqual(type(flight_path), FlightPathSim)

    def test_flight_path_compute_flight(self):
        flight_path = FlightPathSim(
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


if __name__ == '__main__':
    unittest.main()
