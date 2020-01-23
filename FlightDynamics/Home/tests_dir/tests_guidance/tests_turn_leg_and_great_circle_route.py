import unittest

from Home.BadaAircraftPerformance.BadaAircraftDatabaseFile import \
    BadaAircraftDatabase
from Home.BadaAircraftPerformance.BadaAircraftFile import BadaAircraft
from Home.Environment.Atmosphere import Atmosphere
from Home.Environment.Earth import Earth
from Home.Environment.WayPointsDatabaseFile import WayPointsDatabase
from Home.Guidance.TurnLegFile import TurnLeg
from Home.Guidance.GreatCircleRouteFile import GreatCircleRoute


class TestTurnLegGreatCircleRoute(unittest.TestCase):

    def setUp(self) -> None:
        self.atmosphere = Atmosphere()
        self.earth = Earth()
        self.acBd = BadaAircraftDatabase()
        self.aircraftICAOcode = 'A320'
        self.deltaTimeSeconds = 1.0

        assert self.acBd.read()
        assert self.acBd.aircraftExists(self.aircraftICAOcode)
        assert self.acBd.aircraftPerformanceFileExists(self.aircraftICAOcode)

        self.aircraft = BadaAircraft(
            ICAOcode=self.aircraftICAOcode,
            aircraftFullName=self.acBd.getAircraftFullName(
                self.aircraftICAOcode),
            badaPerformanceFilePath=self.acBd.getAircraftPerformanceFile(
                self.aircraftICAOcode),
            atmosphere=self.atmosphere,
            earth=self.earth,
            windSpeedMetersPerSecond=None,
            windDirectionDegrees=None
        )

    def test_turn_leg_fly_by_waypoint(self):
        wayPointsDb = WayPointsDatabase()
        assert (wayPointsDb.read())
        # Exona = wayPointsDb.getWayPoint('EXONA')
        # Rosal = wayPointsDb.getWayPoint('ROSAL')
        # Santa = wayPointsDb.getWayPoint('SANTA')

        p1 = wayPointsDb.getWayPoint('LAMSO')
        p2 = wayPointsDb.getWayPoint('EVELI')
        # p2 = wayPointsDb.getWayPoint('BASNO')
        # p3 = wayPointsDb.getWayPoint('PAMPUS')
        # ret = wayPointsDb.insertWayPoint('ENKOS', "N31°40'58.50" + '"', "N31°40'58.50" + '"')
        # ret = wayPointsDb.insertWayPoint('ENKOS', "N52°40'41.26" + '"', "E5°14'35.75" + '"')
        # if wayPointsDb.hasWayPoint('ENKOS'):
        #     self.assertFalse(ret, 'insertion not done')
        #     exit(1)
        # else:
        #     self.assertTrue(ret, 'insertion correct')
        p3 = wayPointsDb.getWayPoint('ENKOS')

        print(p1)
        print(p2)
        print(p3)
        # print(p4)

        self.aircraft.aircraftCurrentConfiguration = 'cruise'

        self.aircraft.initStateVector(
            elapsedTimeSeconds=0.0,
            trueAirSpeedMetersSecond=70.0,
            airportFieldElevationAboveSeaLevelMeters=10000.0)

        self.aircraft.setTargetCruiseFlightLevel(
            RequestedFlightLevel=310,
            departureAirportAltitudeMSLmeters=0.0
        )

        greatCircle = GreatCircleRoute(
            initialWayPoint=p1,
            finalWayPoint=p2,
            aircraft=self.aircraft
        )

        distanceStillToFlyMeters = p1.getDistanceMetersTo(p2)
        greatCircle.computeGreatCircle(
            deltaTimeSeconds=self.deltaTimeSeconds,
            elapsedTimeSeconds=p1.getElapsedTimeSeconds(),
            distanceStillToFlyMeters=distanceStillToFlyMeters,
            distanceToLastFixMeters=distanceStillToFlyMeters
        )
        finalRoute = greatCircle

        initialHeadingDegrees = p1.getBearingDegreesTo(p2)
        turn_leg = TurnLeg(initialWayPoint=p2,
                              finalWayPoint=p3,
                              initialHeadingDegrees=initialHeadingDegrees,
                              aircraft=self.aircraft,
                              reverse=False)

        last_gc_vertex = finalRoute.getLastVertex().getWeight()
        distance_to_fly = last_gc_vertex.getDistanceMetersTo(p3)
        turn_leg.buildTurnLeg(
            deltaTimeSeconds=self.deltaTimeSeconds,
            elapsedTimeSeconds=last_gc_vertex.getElapsedTimeSeconds(),
            distanceStillToFlyMeters=distance_to_fly,
            distanceToLastFixMeters=distance_to_fly
        )
        finalRoute.addGraph(turn_leg)

        last_turn_leg_vertex = turn_leg.getLastVertex().getWeight()
        # last_turn_leg_vertex = p2
        greatCircle2 = GreatCircleRoute(
            initialWayPoint=last_turn_leg_vertex,
            finalWayPoint=p3,
            aircraft=self.aircraft
        )
        distanceStillToFlyMeters = last_turn_leg_vertex.getDistanceMetersTo(p3)
        greatCircle2.computeGreatCircle(
            deltaTimeSeconds=self.deltaTimeSeconds,
            elapsedTimeSeconds=last_turn_leg_vertex.getElapsedTimeSeconds(),
            distanceStillToFlyMeters=distanceStillToFlyMeters,
            distanceToLastFixMeters=distanceStillToFlyMeters
        )
        finalRoute.addGraph(greatCircle2)

        finalRoute.createKmlOutputFile()
        finalRoute.createXlsxOutputFile()
        self.aircraft.createStateVectorOutputFile(filePrefix='turn-leg-great-circle')

        # export aircraft state history
        # import pandas as pd
        # data = self.aircraft.StateVector.aircraftStateHistory
        # std_output = []
        # for dt in data:
        #     key = list(dt.keys())[0]
        #     print(key)
        #     std_output.append(dt[key])
        #     # exit(0)
        #
        # headers = [
        #     'altitudeMeanSeaLevelMeters',
        #     'trueAirSpeedMetersPerSecond',
        #     'totalDistanceFlownMeters',
        #     'distanceStillToFlyMeters',
        #     'aircraftMassKilograms',
        #     'flightPathAngleDegrees',
        #     'thrustNewtons',
        #     'dragNewtons',
        #     'liftNewtons'
        # ]
        # df = pd.DataFrame(std_output, columns=headers)
        # print(df.head())
        # df.to_csv("aircraft_state_history.csv")


if __name__ == '__main__':
    unittest.main()
