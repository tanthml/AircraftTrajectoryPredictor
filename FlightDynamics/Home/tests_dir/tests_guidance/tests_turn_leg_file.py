import time
import unittest

from Home.BadaAircraftPerformance.BadaAircraftDatabaseFile import \
    BadaAircraftDatabase
from Home.BadaAircraftPerformance.BadaAircraftFile import BadaAircraft
from Home.Environment.AirportDatabaseFile import AirportsDatabase
from Home.Environment.Atmosphere import Atmosphere
from Home.Environment.Earth import Earth
from Home.Environment.RunWaysDatabaseFile import RunWayDataBase
from Home.Environment.WayPointsDatabaseFile import WayPointsDatabase
from Home.Guidance.DescentGlideSlopeFile import DescentGlideSlope
from Home.Guidance.GroundRunLegFile import GroundRunLeg
from Home.Guidance.TurnLegFile import TurnLeg


class TestTurnLeg(unittest.TestCase):

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

    def test_turn_leg(self):
        print(
            '==================== Turn Leg ==================== ' + time.strftime(
                "%c"))



        print(
            '==================== aircraft found  ==================== ' + time.strftime(
                "%c"))

        self.aircraft.dump()

        print(
            '==================== Get Airport ==================== ' + time.strftime(
                "%c"))
        airportsDB = AirportsDatabase()
        assert airportsDB.read()

        print(
            '==================== Get Arrival Airport ==================== ' + time.strftime(
                "%c"))
        Lisbonne = airportsDB.getAirportFromICAOCode('LPPT')
        print(Lisbonne)

        print(
            '====================  find the run-ways ==================== ' + time.strftime(
                "%c"))
        runWaysDatabase = RunWayDataBase()
        if runWaysDatabase.read():
            print('runways DB correctly read')

        print(
            '====================  take off run-way ==================== ' + time.strftime(
                "%c"))
        arrivalRunway = runWaysDatabase.getFilteredRunWays(
            airportICAOcode='LPPT',
            runwayName='')
        print(arrivalRunway)

        print(
            '==================== Ground run ==================== ' + time.strftime(
                "%c"))
        groundRun = GroundRunLeg(runway=arrivalRunway,
                                 aircraft=self.aircraft,
                                 airport=Lisbonne)

        touchDownWayPoint = groundRun.computeTouchDownWayPoint()
        print(touchDownWayPoint)
        groundRun.buildDepartureGroundRun(deltaTimeSeconds=1.0,
                                          elapsedTimeSeconds=0.0,
                                          distanceStillToFlyMeters=0.0,
                                          distanceToLastFixMeters=0.0)
        print(
            '==================== Climb Ramp ==================== ' + time.strftime(
                "%c"))

        initialWayPoint = groundRun.getLastVertex().getWeight()

        descentGlideSlope = DescentGlideSlope(runway=arrivalRunway,
                                              aircraft=self.aircraft,
                                              arrivalAirport=Lisbonne,
                                              descentGlideSlopeDegrees=3.0)
        ''' if there is a fix nearer to 5 nautics of the touch-down then limit size of simulated glide slope '''

        descentGlideSlope.buildSimulatedGlideSlope(
            descentGlideSlopeSizeNautics=5.0)
        descentGlideSlope.createKmlOutputFile()

        firstGlideSlopeWayPoint = descentGlideSlope.getVertex(v=0).getWeight()

        print(
            '==================== Climb Ramp ==================== ' + time.strftime(
                "%c"))
        initialWayPoint = groundRun.getLastVertex().getWeight()

        print(' ================== turn leg start =============== ')
        wayPointsDb = WayPointsDatabase()
        assert (wayPointsDb.read())
        Exona = wayPointsDb.getWayPoint('EXONA')
        Rosal = wayPointsDb.getWayPoint('ROSAL')

        print(Rosal.getBearingDegreesTo(Exona))
        initialHeadingDegrees = arrivalRunway.getTrueHeadingDegrees()

        lastTurnLeg = TurnLeg(initialWayPoint=firstGlideSlopeWayPoint,
                              finalWayPoint=Exona,
                              initialHeadingDegrees=initialHeadingDegrees,
                              aircraft=self.aircraft,
                              reverse=True)
        deltaTimeSeconds = 1.0
        lastTurnLeg.buildNewSimulatedArrivalTurnLeg(
            deltaTimeSeconds=deltaTimeSeconds,
            elapsedTimeSeconds=0.0,
            distanceStillToFlyMeters=0.0,
            simulatedAltitudeSeaLevelMeters=firstGlideSlopeWayPoint.getAltitudeMeanSeaLevelMeters(),
            flightPathAngleDegrees=3.0)
        lastTurnLeg.createKmlOutputFile()
        descentGlideSlope.addGraph(lastTurnLeg)
        descentGlideSlope.createXlsxOutputFile()
        descentGlideSlope.createKmlOutputFile()

        print(' ================== turn leg end =============== ')


if __name__ == '__main__':
    unittest.main()
