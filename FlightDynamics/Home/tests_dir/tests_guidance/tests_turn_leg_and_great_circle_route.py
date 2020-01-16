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
        self.aircraft.setCurrentAltitudeSeaLevelMeters(
            elapsedTimeSeconds=0.0,
            altitudeMeanSeaLevelMeters=10000.0,
            lastAltitudeMeanSeaLevelMeters=10000.0,
            targetCruiseAltitudeMslMeters=10000.0)

        self.aircraft.initStateVector(
            elapsedTimeSeconds=0.0,
            trueAirSpeedMetersSecond=70.0,
            airportFieldElevationAboveSeaLevelMeters=152.0)

        self.aircraft.setTargetCruiseFlightLevel(
            RequestedFlightLevel=310,
            departureAirportAltitudeMSLmeters=10000.0
        )

        greatCircle = GreatCircleRoute(
            initialWayPoint=p1,
            finalWayPoint=p2,
            aircraft=self.aircraft
        )

        distanceStillToFlyMeters = p1.getDistanceMetersTo(p2)
        greatCircle.computeGreatCircle(
            deltaTimeSeconds=1.0,
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

        distance_to_fly = p2.getDistanceMetersTo(p3)
        turn_leg.buildTurnLeg(
            deltaTimeSeconds=self.deltaTimeSeconds,
            elapsedTimeSeconds=p2.getElapsedTimeSeconds(),
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
            elapsedTimeSeconds=0.0,
            distanceStillToFlyMeters=distanceStillToFlyMeters,
            distanceToLastFixMeters=distanceStillToFlyMeters
        )
        finalRoute.addGraph(greatCircle2)

        # turn_leg.addGraph(greatCircle2)
        # turn_leg.createKmlOutputFile()
        # turn_leg.createXlsxOutputFile()


        finalRoute.createKmlOutputFile()
        finalRoute.createXlsxOutputFile()


if __name__ == '__main__':
    unittest.main()
