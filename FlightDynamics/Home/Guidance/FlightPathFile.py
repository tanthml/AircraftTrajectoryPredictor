'''
Created on 3 february 2015

@author: PASTOR Robert

        Written By:
                Robert PASTOR
                @Email: < robert [--DOT--] pastor0691 (--AT--) orange [--DOT--] fr >

        @http://trajectoire-predict.monsite-orange.fr/
        @copyright: Copyright 2015 Robert PASTOR

        This program is free software; you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation; either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.

@ TODO: create a flight list class (with only fixes) , a flight plan with Lat-Long and the flight Path

manage a flight path built from a flight plan
ensure a decoupling between the lateral path and the vertical path.

To simplify the prediction process, the horizontal and vertical components of the trajectory are de-coupled.

The flight legs are consisting of a series of straight-line (great-circle) segments connected by constant radius turns.
The aircraft speed is used to calculate a turn radius.
'''

import math
import json

from Home.Environment.Atmosphere import Atmosphere
from Home.Environment.Earth import Earth

from Home.Guidance.ConstraintsFile import analyseConstraint
from Home.Environment.WayPointsDatabaseFile import WayPointsDatabase
from Home.Environment.AirportDatabaseFile import AirportsDatabase
from Home.Environment.RunWaysDatabaseFile import RunWayDataBase
from Home.Guidance.WayPointFile import WayPoint
from Home.Guidance.GraphFile import Graph, Edge
from Home.Guidance.FlightPlanFile import FlightPlan
from Home.Guidance.GroundRunLegFile import GroundRunLeg
from Home.Guidance.ClimbRampFile import ClimbRamp
from Home.Guidance.TurnLegFile import TurnLeg
from Home.Guidance.GreatCircleRouteFile import GreatCircleRoute
from Home.Guidance.DescentGlideSlopeFile import DescentGlideSlope
from Home.Guidance.WayPointFile import Airport
from Home.Guidance.ConstraintsFile import ArrivalRunWayTouchDownConstraint, TargetApproachConstraint

from Home.BadaAircraftPerformance.BadaAircraftDatabaseFile import BadaAircraftDatabase
from Home.BadaAircraftPerformance.BadaAircraftFile import BadaAircraft

Meter2Feet = 3.2808  # one meter equals 3.28 feet
Meter2NauticalMiles = 0.000539956803  # One Meter = 0.0005 nautical miles
NauticalMiles2Meters = 1852.0
Kilogram2Pounds = 2.20462262  # 1 kilogram = 2.204 lbs

atmosphere = Atmosphere()
earth = Earth()
aircraftDb = BadaAircraftDatabase()
assert aircraftDb.read()


def pairwise(iterable):
    it = iter(iterable)
    a = next(it, None)

    for b in it:
        yield (a, b)
        a = b


def scan(f, state, it):
    for x in it:
        state = f(state, x)
        yield state


class FlightPath(FlightPlan):
    flightPlan = None
    aircraftICAOcode = ''

    def __init__(self,
                 route,
                 aircraftICAOcode='A320',
                 RequestedFlightLevel=330.0,
                 cruiseMach=0.8,
                 takeOffMassKilograms=62000.0,
                 windSpeedMetersPerSecond=None,
                 windDirectionDegrees = None,
    ):

        self.className = self.__class__.__name__
        ''' init mother class '''
        FlightPlan.__init__(self, route)
        ''' first bad and incomplete flight length '''
        ''' missing last turn and glide slope '''
        self.flightLengthMeters = self.computeLengthMeters()
        print(self.flightLengthMeters)
        self.windSpeedMetersPerSecond = windSpeedMetersPerSecond
        self.windDirectionDegrees=windDirectionDegrees
        self.aircraftICAOcode = aircraftICAOcode

        self.aircraft = None
        self.getAircraft()
        assert isinstance(self.aircraft, BadaAircraft) and not (self.aircraft is None)
        self.takeOffMassKilograms = takeOffMassKilograms
        self.RequestedFlightLevel = RequestedFlightLevel
        self.cruiseMach = cruiseMach
        self.aircraft.setAircraftMassKilograms(takeOffMassKilograms)
        self.deltaTimeSeconds = 1
        assert RequestedFlightLevel >= 15.0 and RequestedFlightLevel <= 450.0
        self.aircraft.setTargetCruiseFlightLevel(RequestedFlightLevel=RequestedFlightLevel,
                                                 departureAirportAltitudeMSLmeters=self.getDepartureAirport().getFieldElevationAboveSeaLevelMeters())
        print(cruiseMach)
        self.aircraft.setTargetCruiseMach(cruiseMachNumber=cruiseMach)

        self.arrivalAirport = self.getArrivalAirport()
        if (self.arrivalAirport is None):
            print(self.className + ': there is no arrival airport => flight is out-bound !!!')
        # assert isinstance(self.arrivalAirport, Airport) and not(self.arrivalAirport is None)

        self.departureAirport = self.getDepartureAirport()
        assert isinstance(self.departureAirport, Airport) and not (self.departureAirport is None)

    def convertStrRouteToFixList(self, strRoute):
        '''
        from the route build a fix list and from the fix list build a way point list
        '''
        wayPointsDict = {}
        wayPointsDb = WayPointsDatabase()
        assert (wayPointsDb.read())

        airportsDb = AirportsDatabase()
        assert airportsDb.read()

        runwaysDb = RunWayDataBase()
        assert runwaysDb.read()

        # print self.className + ': ================ get Fix List ================='
        fixList = []
        constraintsList = []
        index = 0
        for fix in strRoute.split('-'):
            fix = str(fix).strip()
            if str(fix).startswith('ADEP'):
                if index == 0:
                    if len(str(fix).split('/')) >= 2:
                        departureAirportIcaoCode = str(fix).split('/')[1]
                        departureAirport = airportsDb.getAirportFromICAOCode(ICAOcode=departureAirportIcaoCode)

                    departureRunwayName = ''
                    if len(str(fix).split('/')) >= 3:
                        departureRunwayName = str(fix).split('/')[2]

                    if not (self.departureAirport is None):
                        departureRunway = runwaysDb.getFilteredRunWays(airportICAOcode=departureAirportIcaoCode,
                                                                       runwayName=departureRunwayName)
                else:
                    raise ValueError(self.className + ': ADEP must be the first fix in the route!!!')


            elif str(fix).startswith('ADES'):
                if index == (len(strRoute.split('-')) - 1):
                    if len(str(fix).split('/')) >= 2:
                        arrivalAirportIcaoCode = str(fix).split('/')[1]
                        arrivalAirport = airportsDb.getAirportFromICAOCode(ICAOcode=arrivalAirportIcaoCode)
                    arrivalRunwayName = ''
                    if len(str(fix).split('/')) >= 3:
                        arrivalRunwayName = str(fix).split('/')[2]

                    if not (arrivalAirport is None):
                        arrivalRunway = runwaysDb.getFilteredRunWays(airportICAOcode=arrivalAirportIcaoCode,
                                                                     runwayName=arrivalRunwayName)

                else:
                    raise ValueError(self.classeName + ': ADES must be the last fix of the route!!!')

            else:
                ''' do not take the 1st one (ADEP) and the last one (ADES) '''
                constraintFound, levelConstraint, speedConstraint = analyseConstraint(index, fix)
                # print self.className + ': constraint found= {0}'.format(constraintFound)
                if constraintFound == True:
                    constraint = {}
                    constraint['fixIndex'] = index
                    constraint['level'] = levelConstraint
                    constraint['speed'] = speedConstraint
                    constraintsList.append(constraint)
                else:
                    fixList.append(fix)
                    wayPoint = wayPointsDb.getWayPoint(fix)
                    if not (wayPoint is None):
                        # print wayPoint
                        wayPointsDict[fix] = wayPoint
                    else:
                        ''' do not insert way point names when there is no known latitude - longitude '''
                        fixList.pop()

            index += 1

        return departureAirport, departureRunway, arrivalAirport, arrivalRunway, constraintsList, fixList, wayPointsDict

    def getAircraft(self):

        print(self.className + ': ================ get aircraft =================')
        atmosphere = Atmosphere()
        earth = Earth()
        acBd = BadaAircraftDatabase()
        assert acBd.read()

        if (acBd.aircraftExists(self.aircraftICAOcode) and
                acBd.aircraftPerformanceFileExists(self.aircraftICAOcode)):

            print(self.className + ': performance file= {0}'.format(
                acBd.getAircraftPerformanceFile(self.aircraftICAOcode)))
            self.aircraft = BadaAircraft(ICAOcode=self.aircraftICAOcode,
                                         aircraftFullName=acBd.getAircraftFullName(self.aircraftICAOcode),
                                         badaPerformanceFilePath=acBd.getAircraftPerformanceFile(self.aircraftICAOcode),
                                         atmosphere=atmosphere,
                                         earth=earth,
                                         windSpeedMetersPerSecond=None,
                                         windDirectionDegrees = None
            )
            self.aircraft.dump()
        else:
            raise ValueError(self.className + ': aircraft not found= ' + self.aircraftICAOcode)

    def printPassedWayPoint(self, finalWayPoint):

        minutes, seconds = divmod(finalWayPoint.getElapsedTimeSeconds(), 60)
        distanceFlownNautics = self.finalRoute.getLengthMeters() * Meter2NauticalMiles
        strMsg = ': passing waypoint: {0} - alt= {1:.2f} meters - alt= {2:.2f} feet - distance= {3:.2f} nautics'.format(
            finalWayPoint.getName(),
            finalWayPoint.getAltitudeMeanSeaLevelMeters(),
            finalWayPoint.getAltitudeMeanSeaLevelMeters() * Meter2Feet,
            distanceFlownNautics)
        elapsedTimeSeconds = finalWayPoint.getElapsedTimeSeconds()
        if elapsedTimeSeconds >= 60.0 and elapsedTimeSeconds < 3600.0:
            minutes, seconds = divmod(elapsedTimeSeconds, 60)

            strMsg += ' - real time = {0:.2f} seconds - {1:.2f} minutes {2:.2f} seconds'.format(elapsedTimeSeconds,
                                                                                                minutes, seconds)
        else:
            minutes, seconds = divmod(elapsedTimeSeconds, 60)
            hours, minutes = divmod(minutes, 60)
            strMsg += ' - real time = {0:.2f} seconds - {1:.2f} hours {2:.2f} minutes {3:.2f} seconds'.format(
                elapsedTimeSeconds, hours, minutes, seconds)

        print(self.className + strMsg)

    # compute distances between a consecutive pair in flight plan, in order
    def computePairwiseDistances(self, fixes):
        pairwiseDistances = []

        for (prev, current) in pairwise(fixes):
            origin, destination = self.wayPointsDict[prev], self.wayPointsDict[current]
            distance = origin.getDistanceMetersTo(destination)
            pairwiseDistances.append(distance)

        return pairwiseDistances

    def buildAircaft(self):
        if (aircraftDb.aircraftExists(self.aircraftICAOcode) and
                aircraftDb.aircraftPerformanceFileExists(self.aircraftICAOcode)):
            aircraft = BadaAircraft(ICAOcode=self.aircraftICAOcode,
                                    aircraftFullName=aircraftDb.getAircraftFullName(self.aircraftICAOcode),
                                    badaPerformanceFilePath=aircraftDb.getAircraftPerformanceFile(
                                        self.aircraftICAOcode),
                                    atmosphere=atmosphere,
                                    earth=earth,
                                    windSpeedMetersPerSecond=self.windSpeedMetersPerSecond,
                                    windDirectionDegrees=self.windDirectionDegrees
            )
            aircraft.setAircraftMassKilograms(self.takeOffMassKilograms)
            assert self.RequestedFlightLevel >= 15.0 and self.RequestedFlightLevel <= 450.0
            aircraft.setTargetCruiseFlightLevel(RequestedFlightLevel=self.RequestedFlightLevel,
                                                departureAirportAltitudeMSLmeters=self.getDepartureAirport().getFieldElevationAboveSeaLevelMeters())
            aircraft.setTargetCruiseMach(cruiseMachNumber=self.cruiseMach)

            return aircraft

        raise Exception("Invalid aircraft")

    def depart(self, aircraft, fixes, deltaTimeSeconds=1):
        wayPoints = [self.wayPointsDict[f] for f in fixes]
        distances = self.computePairwiseDistances(fixes)

        # DEPATURE
        elapsedTimeSeconds = 0
        finalRoute = GroundRunLeg(runway=self.departureRunway,
                                  aircraft=aircraft,
                                  airport=self.departureAirport)

        distanceToLastFixMeters = self.departureAirport.getDistanceMetersTo(wayPoints[1]) + sum(distances[1:])
        estimatedFlightLength = self.departureAirport.getDistanceMetersTo(wayPoints[0]) + sum(distances) + wayPoints[
            -1].getDistanceMetersTo(self.arrivalAirport)
        distanceStillToFlyMeters = estimatedFlightLength - finalRoute.getLengthMeters()

        finalRoute.buildDepartureGroundRun(deltaTimeSeconds=deltaTimeSeconds,
                                           elapsedTimeSeconds=elapsedTimeSeconds,
                                           distanceStillToFlyMeters=distanceStillToFlyMeters,
                                           distanceToLastFixMeters=distanceToLastFixMeters)
        distanceStillToFlyMeters = estimatedFlightLength - finalRoute.getLengthMeters()

        initialWayPoint = finalRoute.getLastVertex().getWeight()
        distanceToFirstFixNautics = initialWayPoint.getDistanceMetersTo(self.getFirstWayPoint()) * Meter2NauticalMiles

        climbRamp = ClimbRamp(initialWayPoint=initialWayPoint,
                              runway=self.departureRunway,
                              aircraft=aircraft,
                              departureAirport=self.departureAirport)

        climbRampLengthNautics = min(distanceToFirstFixNautics / 2.0, 5.0)
        climbRamp.buildClimbRamp(deltaTimeSeconds=deltaTimeSeconds,
                                 elapsedTimeSeconds=initialWayPoint.getElapsedTimeSeconds(),
                                 distanceStillToFlyMeters=distanceStillToFlyMeters,
                                 distanceToLastFixMeters=distanceToLastFixMeters,
                                 climbRampLengthNautics=climbRampLengthNautics)
        finalRoute.addGraph(climbRamp)

        initialWayPoint = finalRoute.getLastVertex().getWeight()
        lastLeg = finalRoute.getLastEdge()
        initialHeadingDegrees = lastLeg.getBearingTailHeadDegrees()

        return finalRoute, initialWayPoint, initialHeadingDegrees

    def simulateArrival(self, aircraft, fixes, deltaTimeSeconds=1):
        wayPoints = [self.wayPointsDict[f] for f in self.fixList]
        distances = self.computePairwiseDistances(self.fixList)
        constraints = []

        # ARRIVAL
        arrivalGroundRun = GroundRunLeg(runway=self.arrivalRunway,
                                        aircraft=aircraft,
                                        airport=self.arrivalAirport)

        touchDownWayPoint = arrivalGroundRun.computeTouchDownWayPoint()
        constraints.append(ArrivalRunWayTouchDownConstraint(touchDownWayPoint))

        distanceToLastFixNautics = touchDownWayPoint.getDistanceMetersTo(wayPoints[-1]) * Meter2NauticalMiles
        descentGlideSlope = DescentGlideSlope(runway=self.arrivalRunway,
                                              aircraft=aircraft,
                                              arrivalAirport=self.arrivalAirport,
                                              descentGlideSlopeDegrees=3.0)

        descentGlideSlopeSizeNautics = min(distanceToLastFixNautics / 2.0, 5.0)
        descentGlideSlope.buildSimulatedGlideSlope(descentGlideSlopeSizeNautics)
        firstGlideSlopeWayPoint = descentGlideSlope.getVertex(v=0).getWeight()
        lastFixListWayPoint = wayPoints[-1]
        initialHeadingDegrees = self.arrivalRunway.getTrueHeadingDegrees()

        lastTurnLeg = TurnLeg(initialWayPoint=firstGlideSlopeWayPoint,
                              finalWayPoint=lastFixListWayPoint,
                              initialHeadingDegrees=initialHeadingDegrees,
                              aircraft=aircraft,
                              reverse=True)
        lastTurnLeg.buildNewSimulatedArrivalTurnLeg(deltaTimeSeconds=deltaTimeSeconds,
                                                    elapsedTimeSeconds=0.0,
                                                    distanceStillToFlyMeters=0.0,
                                                    simulatedAltitudeSeaLevelMeters=firstGlideSlopeWayPoint.getAltitudeMeanSeaLevelMeters(),
                                                    flightPathAngleDegrees=3.0,
                                                    bankAngleDegrees=5.0)
        descentGlideSlope.addGraph(lastTurnLeg)

        beginOfLastTurnLeg = lastTurnLeg.getVertex(v=0).getWeight()
        constraints.append(TargetApproachConstraint(beginOfLastTurnLeg))

        return beginOfLastTurnLeg, touchDownWayPoint, descentGlideSlope.getLengthMeters()

    def flyFrom(self, aircraft, fixes, flightIndex, anticipatedTurnWayPoint, distanceStillToFlyMeters, lastLegEdge,
                elapsedTimeSeconds, initialHeadingDegrees,
                deltaTimeSeconds=1):
        route = Graph()
        route.addVertex(lastLegEdge)
        justRoute = Graph()

        endOfSimulation = False
        last = None
        finalHeadingDegrees = None

        if (anticipatedTurnWayPoint is None):
            fix = fixes[flightIndex]
            tailWayPoint = self.wayPointsDict[fix]
        else:
            tailWayPoint = anticipatedTurnWayPoint

        tailWayPoint.setElapsedTimeSeconds(elapsedTimeSeconds)
        print(self.fixList, flightIndex)
        if (flightIndex + 1) < len(fixes):
            headWayPoint = self.wayPointsDict[fixes[flightIndex + 1]]
            turnLeg = TurnLeg(initialWayPoint=tailWayPoint,
                              finalWayPoint=headWayPoint,
                              initialHeadingDegrees=initialHeadingDegrees,
                              aircraft=aircraft,
                              reverse=False)

            distanceToLastFixMeters = self.computeDistanceToLastFixMeters(currentPosition=tailWayPoint,
                                                                          fixListIndex=flightIndex)
            endOfSimulation = turnLeg.buildTurnLeg(deltaTimeSeconds=deltaTimeSeconds,
                                                   elapsedTimeSeconds=tailWayPoint.getElapsedTimeSeconds(),
                                                   distanceStillToFlyMeters=distanceStillToFlyMeters,
                                                   distanceToLastFixMeters=distanceToLastFixMeters)
            route.addGraph(turnLeg)
            justRoute.addGraph(turnLeg)

            if (endOfSimulation == False):
                endOfTurnLegWayPoint = route.getLastVertex().getWeight()
                lastLeg = route.getLastEdge()

                anticipatedTurnWayPoint = None
                if (flightIndex + 2) < len(fixes):
                    firstAngleDegrees = endOfTurnLegWayPoint.getBearingDegreesTo(headWayPoint)
                    secondAngleDegrees = headWayPoint.getBearingDegreesTo(self.wayPointsDict[fixes[flightIndex + 2]])
                    firstAngleRadians = math.radians(firstAngleDegrees)
                    secondAngleRadians = math.radians(secondAngleDegrees)

                    angleDifferenceDegrees = math.degrees(math.atan2(math.sin(secondAngleRadians - firstAngleRadians),
                                                                     math.cos(secondAngleRadians - firstAngleRadians)))

                    tasMetersPerSecond = aircraft.getCurrentTrueAirSpeedMetersSecond()
                    radiusOfTurnMeters = (tasMetersPerSecond * tasMetersPerSecond) / (
                            9.81 * math.tan(math.radians(15.0)))

                    anticipatedTurnStartMeters = radiusOfTurnMeters * math.tan(
                        math.radians((180.0 - abs(angleDifferenceDegrees)) / 2.0))

                    if ((endOfTurnLegWayPoint.getDistanceMetersTo(headWayPoint) > (1.1 * anticipatedTurnStartMeters)
                         and abs(angleDifferenceDegrees) > 30.)):
                        bearingDegrees = math.fmod(firstAngleDegrees + 180.0, 360.0)
                        anticipatedTurnWayPoint = headWayPoint.getWayPointAtDistanceBearing(
                            Name='Anticipated-Turn-' + headWayPoint.getName(),
                            DistanceMeters=anticipatedTurnStartMeters,
                            BearingDegrees=bearingDegrees)
                        headWayPoint = anticipatedTurnWayPoint

                greatCircle = GreatCircleRoute(initialWayPoint=endOfTurnLegWayPoint,
                                               finalWayPoint=headWayPoint,
                                               aircraft=aircraft)

                distanceToLastFixMeters = self.computeDistanceToLastFixMeters(currentPosition=endOfTurnLegWayPoint,
                                                                              fixListIndex=flightIndex)

                endOfSimulation = greatCircle.computeGreatCircle(deltaTimeSeconds=deltaTimeSeconds,
                                                                 elapsedTimeSeconds=endOfTurnLegWayPoint.getElapsedTimeSeconds(),
                                                                 distanceStillToFlyMeters=distanceStillToFlyMeters - route.getLengthMeters(),
                                                                 distanceToLastFixMeters=distanceToLastFixMeters)
                route.addGraph(greatCircle)
                justRoute.addGraph(greatCircle)

                finalWayPoint = route.getLastVertex().getWeight()
                lastLeg = route.getLastEdge()
                finalHeadingDegrees = lastLeg.getBearingTailHeadDegrees()
                distanceStillToFlyMeters -= route.getLengthMeters()
                elapsedTimeSeconds = finalWayPoint.getElapsedTimeSeconds()
                last = route.getLastVertex().getWeight()

                print("Initial heading degrees:", finalHeadingDegrees, lastLeg)
        return endOfSimulation, anticipatedTurnWayPoint, distanceStillToFlyMeters, last, elapsedTimeSeconds, finalHeadingDegrees, justRoute

    def arrive(self, aircraft, initialHeadingDegrees, endOfLastGreatCircleWayPoint, touchDownWayPoint, distanceStillToFlyMeters, flownDistance):
        route = Graph()
        route.addVertex(endOfLastGreatCircleWayPoint)
        print(endOfLastGreatCircleWayPoint)
        finalHeadingDegrees = self.arrivalRunway.getTrueHeadingDegrees()
        finalHeadingDegrees = math.fmod(finalHeadingDegrees + 180.0, 360.0)

        turnLeg = TurnLeg(initialWayPoint=endOfLastGreatCircleWayPoint,
                          finalWayPoint= touchDownWayPoint,
                          initialHeadingDegrees=initialHeadingDegrees,
                          aircraft= aircraft,
                          reverse=False)
        distanceToLastFixMeters  = distanceStillToFlyMeters
        deltaTimeSeconds = 0.1
        turnLeg.buildTurnLeg(deltaTimeSeconds=deltaTimeSeconds,
                             elapsedTimeSeconds=endOfLastGreatCircleWayPoint.getElapsedTimeSeconds(),
                             distanceStillToFlyMeters=distanceStillToFlyMeters,
                             distanceToLastFixMeters=distanceToLastFixMeters,
                             finalHeadingDegrees=finalHeadingDegrees,
                             lastTurn=True,
                             bankAngleDegrees=5.0)
        route.addGraph(turnLeg)

        endOfTurnLegWayPoint = route.getLastVertex().getWeight()
        descentGlideSlope = DescentGlideSlope(runway=self.arrivalRunway,
                                              aircraft=aircraft,
                                              arrivalAirport=self.arrivalAirport,
                                              descentGlideSlopeDegrees=3.0)

        flownDistanceMeters = flownDistance + route.getLengthMeters()

        distanceStillToFlyMeters = distanceStillToFlyMeters - route.getLengthMeters()
        distanceToLastFixMeters = distanceStillToFlyMeters

        descentGlideSlope.buildGlideSlope(deltaTimeSeconds=self.deltaTimeSeconds,
                                          elapsedTimeSeconds=endOfTurnLegWayPoint.getElapsedTimeSeconds(),
                                          initialWayPoint=endOfTurnLegWayPoint,
                                          flownDistanceMeters=flownDistanceMeters,
                                          distanceStillToFlyMeters=distanceStillToFlyMeters,
                                          distanceToLastFixMeters=distanceToLastFixMeters)
        route.addGraph(descentGlideSlope)
        endOfDescentGlideSlope = route.getLastVertex().getWeight()

        arrivalGroundRun = GroundRunLeg(runway=self.arrivalRunway,
                                        aircraft=aircraft,
                                        airport=self.arrivalAirport)
        arrivalGroundRun.buildArrivalGroundRun(deltaTimeSeconds=self.deltaTimeSeconds,
                                               elapsedTimeSeconds=endOfDescentGlideSlope.getElapsedTimeSeconds(),
                                               initialWayPoint=endOfDescentGlideSlope)
        route.addGraph(arrivalGroundRun)
        return route

    def simulateFly(self, startFrom, flightPath, updatedSpeed=None, updatedAltitude=None):
        _, _, _, _, _, fixes, wayPointsDict = self.convertStrRouteToFixList(flightPath)

        with open('configurations.json', 'r') as f:
            configurations = json.load(f)
            configuration = configurations[startFrom]

            aircraftConfig = configuration['aircraft']
            departedAircraft = self.buildAircaft()
            finalRoute, initialWayPoint, initialHeadingDegrees = self.depart(departedAircraft, self.fixList,
                                                                             deltaTimeSeconds=1)
            beginOfLastTurnLeg, touchDownWayPoint, descentGlideSlopeLength = self.simulateArrival(departedAircraft,
                                                                                                  self.fixList)
            fixes = [initialWayPoint.Name] + fixes + [beginOfLastTurnLeg.Name]
            wayPointsDict[initialWayPoint.Name] = initialWayPoint
            wayPointsDict[beginOfLastTurnLeg.Name] = beginOfLastTurnLeg

            # Recover the aircraft
            aircraft = self.buildAircaft()
            elapsedTimeSeconds = list(aircraftConfig.keys())[0]
            state = aircraftConfig[elapsedTimeSeconds]
            aircraft.updateAircraftStateVector(elapsedTimeSeconds,
                                               updatedSpeed or state[1],
                                               updatedAltitude or state[0],
                                               state[2],
                                               state[3],
                                               state[4],
                                               state[5],
                                               state[6],
                                               state[7],
                                               state[8],
                                               False)
            aircraft.setAircraftMassKilograms(state[4])
            aircraft.setTargetApproachWayPoint(beginOfLastTurnLeg)
            aircraft.setArrivalRunwayTouchDownWayPoint(touchDownWayPoint)
            aircraft.aircraftCurrentConfiguration = 'cruise'
            last = None

            if configuration['previousVertex']:
                lastVertex = configuration['previousVertex']
                last = WayPoint(
                    Name=lastVertex['Name'],
                    LatitudeDegrees=lastVertex['LatitudeDegrees'],
                    LongitudeDegrees=lastVertex['LongitudeDegrees'],
                    AltitudeMeanSeaLevelMeters=lastVertex['AltitudeMeanSeaLevelMeters']
                )
            self.wayPointsDict = wayPointsDict
            self.fixList = fixes
            fRoute = Graph()
            wayPoints = [wayPointsDict[f] for f in fixes]
            distances = self.computePairwiseDistances(fixes)
            flightLength = self.departureAirport.getDistanceMetersTo(wayPoints[0]) + sum(distances) + wayPoints[
                -1].getDistanceMetersTo(self.arrivalAirport) + descentGlideSlopeLength
            distanceStillToFlyMeters = flightLength - configuration['flownDistance']
            elapsedTimeSeconds = configuration['elapsedTimeSeconds']
            initialHeadingDegrees = configuration['initialHeadingDegrees']
            endOfSimulation = False
            flightIndex = startFrom
            anticipatedTurnWayPoint = None

            while (endOfSimulation == False) and (flightIndex < len(fixes) - 1):
                endOfSimulation, anticipatedTurnWayPoint, distanceStillToFlyMeters, last, elapsedTimeSeconds, initialHeadingDegrees, route = self.flyFrom(
                    aircraft, fixes, flightIndex, anticipatedTurnWayPoint, distanceStillToFlyMeters, last,
                    elapsedTimeSeconds, initialHeadingDegrees
                )
                fRoute.addGraph(route)
                finalRoute.addGraph(route)
                flightIndex += 1

            route = self.arrive(aircraft, initialHeadingDegrees, fRoute.getLastVertex().getWeight(), touchDownWayPoint, distanceStillToFlyMeters, flightLength)
            fRoute.addGraph(route)
            fRoute.createXlsxOutputFile()

    def fly(self, deltaTimeSeconds=1):
        configurations = []

        constraints = []
        fixes = self.fixList
        aircraft = self.buildAircaft()
        finalRoute, initialWayPoint, initialHeadingDegrees = self.depart(aircraft, fixes, deltaTimeSeconds)
        beginOfLastTurnLeg, touchDownWayPoint, descentGlideSlopeLength = self.simulateArrival(aircraft, fixes,
                                                                                              deltaTimeSeconds)

        self.insert(position='begin', wayPoint=initialWayPoint)
        self.insert(position='end', wayPoint=beginOfLastTurnLeg)
        aircraft.setTargetApproachWayPoint(beginOfLastTurnLeg)
        aircraft.setArrivalRunwayTouchDownWayPoint(touchDownWayPoint)

        wayPoints = [self.wayPointsDict[f] for f in self.fixList]
        distances = self.computePairwiseDistances(self.fixList)
        flightLength = self.departureAirport.getDistanceMetersTo(wayPoints[0]) + sum(distances) + wayPoints[
            -1].getDistanceMetersTo(self.arrivalAirport) + descentGlideSlopeLength

        anticipatedTurnWayPoint = None
        elapsedTimeSeconds = initialWayPoint.getElapsedTimeSeconds()
        flightIndex = 0
        endOfSimulation = False
        last = finalRoute.getLastVertex().getWeight()
        distanceStillToFlyMeters = flightLength - finalRoute.getLengthMeters()

        while (endOfSimulation == False) and (flightIndex < len(self.fixList)):
            previousVertex = {
                'Name': last.Name,
                'LatitudeDegrees': last.LatitudeDegrees,
                'LongitudeDegrees': last.LongitudeDegrees,
                'AltitudeMeanSeaLevelMeters': last.AltitudeMeanSeaLevelMeters
            } if last else None
            anticipatedTurnWayPointObj = {
                'Name': anticipatedTurnWayPoint.Name,
                'LatitudeDegrees': anticipatedTurnWayPoint.LatitudeDegrees,
                'LongitudeDegrees': anticipatedTurnWayPoint.LongitudeDegrees,
                'AltitudeMeanSeaLevelMeters': anticipatedTurnWayPoint.AltitudeMeanSeaLevelMeters
            } if anticipatedTurnWayPoint else None
            configurations.append({
                'aircraft': aircraft.StateVector.aircraftStateHistory[-1],
                'index': flightIndex,
                'anticipatedTurnWayPoint': anticipatedTurnWayPointObj,
                'previousVertex': previousVertex,
                'elapsedTimeSeconds': elapsedTimeSeconds,
                'initialHeadingDegrees': initialHeadingDegrees,
                'flownDistance': flightLength - distanceStillToFlyMeters
            })

            endOfSimulation, anticipatedTurnWayPoint, distanceStillToFlyMeters, last, elapsedTimeSeconds, initialHeadingDegrees, _ = self.flyFrom(
                aircraft, self.fixList, flightIndex, anticipatedTurnWayPoint, distanceStillToFlyMeters, last,
                elapsedTimeSeconds, initialHeadingDegrees,
                deltaTimeSeconds=1
            )

            flightIndex += 1

        with open('configurations.json', 'w') as f:
            json.dump(configurations, f)

    def turnAndFly(self,
                   tailWayPoint,
                   headWayPoint,
                   initialHeadingDegrees,
                   headWayPointIndex):
        '''
        execute a turn to align true heading and then fly a great circle
        '''
        print(' ================== one Turn Leg for each fix in the list =============== ')
        turnLeg = TurnLeg(initialWayPoint=tailWayPoint,
                          finalWayPoint=headWayPoint,
                          initialHeadingDegrees=initialHeadingDegrees,
                          aircraft=self.aircraft,
                          reverse=False)

        distanceToLastFixMeters = self.computeDistanceToLastFixMeters(currentPosition=tailWayPoint,
                                                                      fixListIndex=headWayPointIndex)
        print(self.className + ': distance to last fix= {0} nautics'.format(
            distanceToLastFixMeters * Meter2NauticalMiles))
        distanceStillToFlyMeters = self.flightLengthMeters - self.finalRoute.getLengthMeters()
        print(self.className + ': still to fly= {0} nautics'.format(distanceStillToFlyMeters * Meter2NauticalMiles))

        endOfSimulation = turnLeg.buildTurnLeg(deltaTimeSeconds=self.deltaTimeSeconds,
                                               elapsedTimeSeconds=tailWayPoint.getElapsedTimeSeconds(),
                                               distanceStillToFlyMeters=distanceStillToFlyMeters,
                                               distanceToLastFixMeters=distanceToLastFixMeters)
        self.finalRoute.addGraph(turnLeg)

        if (endOfSimulation == False):
            print(' ==================== end of turn leg  ==================== ')

            endOfTurnLegWayPoint = self.finalRoute.getLastVertex().getWeight()
            lastLeg = self.finalRoute.getLastEdge()
            print(self.className + ': end of turn orientation= {0:.2f} degrees'.format(
                lastLeg.getBearingTailHeadDegrees()))

            '''==================== check if anticipated turn or fly by is applicable '''
            anticipatedTurnWayPoint = None
            if (self.flightListIndex + 2) < len(self.fixList):
                ''' still another fix in the list '''
                firstAngleDegrees = endOfTurnLegWayPoint.getBearingDegreesTo(headWayPoint)
                secondAngleDegrees = headWayPoint.getBearingDegreesTo(
                    self.wayPointsDict[self.fixList[self.flightListIndex + 2]])
                firstAngleRadians = math.radians(firstAngleDegrees)
                secondAngleRadians = math.radians(secondAngleDegrees)

                angleDifferenceDegrees = math.degrees(math.atan2(math.sin(secondAngleRadians - firstAngleRadians),
                                                                 math.cos(secondAngleRadians - firstAngleRadians)))
                print(self.className + ': difference= {0:.2f} degrees'.format(angleDifferenceDegrees))

                tasMetersPerSecond = self.aircraft.getCurrentTrueAirSpeedMetersSecond()
                radiusOfTurnMeters = (tasMetersPerSecond * tasMetersPerSecond) / (9.81 * math.tan(math.radians(15.0)))

                anticipatedTurnStartMeters = radiusOfTurnMeters * math.tan(
                    math.radians((180.0 - abs(angleDifferenceDegrees)) / 2.0))
                print(self.className + ': anticipated turn start from end point= {0:.2f} meters'.format(
                    anticipatedTurnStartMeters))

                if ((endOfTurnLegWayPoint.getDistanceMetersTo(headWayPoint) > (1.1 * anticipatedTurnStartMeters)
                     and abs(angleDifferenceDegrees) > 30.)):
                    print(self.className + ': Envisage anticipated Fly By turn !!!!!!!!!!!!!!!!!!!!!!!!!')
                    bearingDegrees = math.fmod(firstAngleDegrees + 180.0, 360.0)
                    anticipatedTurnWayPoint = headWayPoint.getWayPointAtDistanceBearing(
                        Name='Anticipated-Turn-' + headWayPoint.getName(),
                        DistanceMeters=anticipatedTurnStartMeters,
                        BearingDegrees=bearingDegrees)
                    headWayPoint = anticipatedTurnWayPoint

            print(' ==================== great circle ======================== ')
            greatCircle = GreatCircleRoute(initialWayPoint=endOfTurnLegWayPoint,
                                           finalWayPoint=headWayPoint,
                                           aircraft=self.aircraft)

            distanceToLastFixMeters = self.computeDistanceToLastFixMeters(currentPosition=endOfTurnLegWayPoint,
                                                                          fixListIndex=headWayPointIndex)
            print(self.className + ': distance to last fix= {0} nautics'.format(
                distanceToLastFixMeters * Meter2NauticalMiles))

            distanceStillToFlyMeters = self.flightLengthMeters - self.finalRoute.getLengthMeters()
            print(self.className + ': still to fly= {0} nautics'.format(distanceStillToFlyMeters * Meter2NauticalMiles))

            endOfSimulation = greatCircle.computeGreatCircle(deltaTimeSeconds=self.deltaTimeSeconds,
                                                             elapsedTimeSeconds=endOfTurnLegWayPoint.getElapsedTimeSeconds(),
                                                             distanceStillToFlyMeters=distanceStillToFlyMeters,
                                                             distanceToLastFixMeters=distanceToLastFixMeters)
            ''' update final route '''
            self.finalRoute.addGraph(greatCircle)

            print(' ================== end of great circle ================== ')

            finalWayPoint = self.finalRoute.getLastVertex().getWeight()
            # print self.className + ': current end way point= ' + str(finalWayPoint)

            lastLeg = self.finalRoute.getLastEdge()
            finalHeadingDegrees = lastLeg.getBearingTailHeadDegrees()
            # print self.className + ': last leg orientation= {0:.2f} degrees'.format(finalHeadingDegrees)

            distanceStillToFlyMeters = self.flightLengthMeters - self.finalRoute.getLengthMeters()
            print(self.className + ': still to fly= {0:.2f} meters - still to fly= {1:.2f} nautics'.format(
                distanceStillToFlyMeters, distanceStillToFlyMeters * Meter2NauticalMiles))
            ''' print the way point that has been passed right now '''
            self.printPassedWayPoint(finalWayPoint)
        ''' return to caller '''
        return endOfSimulation, finalHeadingDegrees, finalWayPoint.getElapsedTimeSeconds(), anticipatedTurnWayPoint

    def loopThroughFixList(self,
                           initialHeadingDegrees,
                           elapsedTimeSeconds):

        anticipatedTurnWayPoint = None
        ''' start loop over the fix list '''
        ''' fix list does not contain departure and arrival airports '''
        self.flightListIndex = 0
        ''' loop over the fix list '''
        endOfSimulation = False
        while (endOfSimulation == False) and (self.flightListIndex < len(self.fixList)):
            # print  self.className + ': initial heading degrees= ' + str(initialHeadingDegrees) + ' degrees'
            print(self.flightLengthMeters)
            ''' get the next fix '''
            if (anticipatedTurnWayPoint is None):
                fix = self.fixList[self.flightListIndex]
                ''' tail way point to reach '''
                tailWayPoint = self.wayPointsDict[fix]
            else:
                ''' we do not use the next fix but the anticipated turn way point '''
                tailWayPoint = anticipatedTurnWayPoint
            print("fix, tailWayPoint: ", self.flightListIndex, len(self.fixList), fix, tailWayPoint)
            tailWayPoint.setElapsedTimeSeconds(elapsedTimeSeconds)

            if (self.flightListIndex + 1) < len(self.fixList):
                ''' next way point is still in the fix list => not yet the arrival airport '''
                headWayPoint = self.wayPointsDict[self.fixList[self.flightListIndex + 1]]
                print(headWayPoint)
                ''' turn and fly '''
                endOfSimulation, initialHeadingDegrees, elapsedTimeSeconds, anticipatedTurnWayPoint = self.turnAndFly(
                    tailWayPoint=tailWayPoint,
                    headWayPoint=headWayPoint,
                    initialHeadingDegrees=initialHeadingDegrees,
                    headWayPointIndex=self.flightListIndex)
            ''' prepare for next loop '''
            self.flightListIndex += 1

        ''' return final heading of the last great circle '''
        return endOfSimulation, initialHeadingDegrees

    def buildDeparturePhase(self):
        '''
        this function manages the departure phases with a ground run and a climb ramp
        '''

        print(self.className + ' ============== build the departure ground run =========== ')
        self.finalRoute = GroundRunLeg(runway=self.departureRunway,
                                       aircraft=self.aircraft,
                                       airport=self.departureAirport)

        distanceToLastFixMeters = self.computeDistanceToLastFixMeters(currentPosition=self.departureAirport,
                                                                      fixListIndex=0)
        distanceStillToFlyMeters = self.flightLengthMeters - self.finalRoute.getLengthMeters()
        elapsedTimeSeconds = 0.0
        self.finalRoute.buildDepartureGroundRun(deltaTimeSeconds=self.deltaTimeSeconds,
                                                elapsedTimeSeconds=elapsedTimeSeconds,
                                                distanceStillToFlyMeters=distanceStillToFlyMeters,
                                                distanceToLastFixMeters=distanceToLastFixMeters)
        distanceStillToFlyMeters = self.flightLengthMeters - self.finalRoute.getLengthMeters()

        # print '==================== end of ground run ==================== '
        initialWayPoint = self.finalRoute.getLastVertex().getWeight()
        distanceToFirstFixNautics = initialWayPoint.getDistanceMetersTo(self.getFirstWayPoint()) * Meter2NauticalMiles
        # print '==================== Initial Climb Ramp ==================== '

        climbRamp = ClimbRamp(initialWayPoint=initialWayPoint,
                              runway=self.departureRunway,
                              aircraft=self.aircraft,
                              departureAirport=self.departureAirport)
        ''' climb ramp of 5.0 nautics is not possible if first fix placed in between '''
        climbRampLengthNautics = min(distanceToFirstFixNautics / 2.0, 5.0)
        climbRamp.buildClimbRamp(deltaTimeSeconds=self.deltaTimeSeconds,
                                 elapsedTimeSeconds=initialWayPoint.getElapsedTimeSeconds(),
                                 distanceStillToFlyMeters=distanceStillToFlyMeters,
                                 distanceToLastFixMeters=distanceToLastFixMeters,
                                 climbRampLengthNautics=climbRampLengthNautics)
        self.finalRoute.addGraph(climbRamp)

        # print '============= initial condition for the route ================='

        initialWayPoint = self.finalRoute.getLastVertex().getWeight()
        lastLeg = self.finalRoute.getLastEdge()
        initialHeadingDegrees = lastLeg.getBearingTailHeadDegrees()
        print(self.className + ': last leg orientation= {0:.2f} degrees'.format(initialHeadingDegrees))

        # '''============= add way point in the fix list =============== '''
        self.insert(position='begin', wayPoint=initialWayPoint)
        # print self.className + ': fix list= {0}'.format(self.fixList)

        return initialHeadingDegrees, initialWayPoint

    def buildSimulatedArrivalPhase(self):

        print(self.className + '=========== add final turn, descent and ground run ===================')
        arrivalGroundRun = GroundRunLeg(runway=self.arrivalRunway,
                                        aircraft=self.aircraft,
                                        airport=self.arrivalAirport)
        self.touchDownWayPoint = arrivalGroundRun.computeTouchDownWayPoint()
        # add touch down to constraint list
        self.constraintsList.append(ArrivalRunWayTouchDownConstraint(self.touchDownWayPoint))

        print(self.touchDownWayPoint)
        ''' distance from last fix to touch down '''
        distanceToLastFixNautics = self.touchDownWayPoint.getDistanceMetersTo(
            self.getLastWayPoint()) * Meter2NauticalMiles

        print(self.className + '===================== final 3 degrees descending glide slope ================')
        descentGlideSlope = DescentGlideSlope(runway=self.arrivalRunway,
                                              aircraft=self.aircraft,
                                              arrivalAirport=self.arrivalAirport,
                                              descentGlideSlopeDegrees=3.0)
        ''' if there is a fix nearer to 5 nautics of the touch-down then limit size of simulated glide slope '''
        descentGlideSlopeSizeNautics = min(distanceToLastFixNautics / 2.0, 5.0)
        ''' build simulated glide slope '''
        descentGlideSlope.buildSimulatedGlideSlope(descentGlideSlopeSizeNautics)
        self.firstGlideSlopeWayPoint = descentGlideSlope.getVertex(v=0).getWeight()
        print(self.className + ': top of arrival glide slope= {0}'.format(self.firstGlideSlopeWayPoint))

        print(
            self.className + ' ================= need a turn leg to find the junction point the last way-point in the fix list to the top of the final glide slope')
        '''
        initial heading is the orientation of the run-way
        '''
        lastFixListWayPoint = self.wayPointsDict[self.fixList[-1]]
        initialHeadingDegrees = self.arrivalRunway.getTrueHeadingDegrees()

        lastTurnLeg = TurnLeg(initialWayPoint=self.firstGlideSlopeWayPoint,
                              finalWayPoint=lastFixListWayPoint,
                              initialHeadingDegrees=initialHeadingDegrees,
                              aircraft=self.aircraft,
                              reverse=True)
        lastTurnLeg.buildNewSimulatedArrivalTurnLeg(deltaTimeSeconds=self.deltaTimeSeconds,
                                                    elapsedTimeSeconds=0.0,
                                                    distanceStillToFlyMeters=0.0,
                                                    simulatedAltitudeSeaLevelMeters=self.firstGlideSlopeWayPoint.getAltitudeMeanSeaLevelMeters(),
                                                    flightPathAngleDegrees=3.0,
                                                    bankAngleDegrees=5.0)
        descentGlideSlope.addGraph(lastTurnLeg)
        # print self.className + ': compute arrival phase length= {0:.2f} meters'.format(descentGlideSlope.getLengthMeters())
        # descentGlideSlope.createXlsxOutputFile()
        # descentGlideSlope.createKmlOutputFile()
        ''' prepare next step '''
        beginOfLastTurnLeg = lastTurnLeg.getVertex(v=0).getWeight()
        print(self.className + ': begin of last turn= {0}'.format(beginOfLastTurnLeg))
        ''' add to constraint list '''
        self.constraintsList.append(TargetApproachConstraint(beginOfLastTurnLeg))

        ''' add the three last way-points in the fix list '''
        self.insert(position='end', wayPoint=beginOfLastTurnLeg)
        ''' update the length of the flight path '''
        self.distanceFromApproachToTouchDownMeters = descentGlideSlope.getLengthMeters()
        self.flightLengthMeters = self.computeLengthMeters() + descentGlideSlope.getLengthMeters()

        print(self.className + ': updated flight path length= {0:.2f} nautics'.format(
            self.flightLengthMeters * Meter2NauticalMiles))

        ''' target approach fix is equal to the begin of the SIMULATED last turn leg '''
        self.aircraft.setTargetApproachWayPoint(beginOfLastTurnLeg)
        self.aircraft.setArrivalRunwayTouchDownWayPoint(self.touchDownWayPoint)
        print(self.className + ': fix list= {0}'.format(self.fixList))

    def buildArrivalPhase(self, initialHeadingDegrees):

        print(self.className + '==================== add last turn ==================== ')
        if self.isDomestic() or self.isInBound():
            endOfLastGreatCircleWayPoint = self.finalRoute.getLastVertex().getWeight()

            finalHeadingDegrees = self.arrivalRunway.getTrueHeadingDegrees()
            finalHeadingDegrees = math.fmod(finalHeadingDegrees + 180.0, 360.0)
            print(self.className + ': runway final heading= {0:.2f} degrees'.format(finalHeadingDegrees))

            turnLeg = TurnLeg(initialWayPoint=endOfLastGreatCircleWayPoint,
                              # finalWayPoint    = self.firstGlideSlopeWayPoint,
                              finalWayPoint=self.touchDownWayPoint,
                              initialHeadingDegrees=initialHeadingDegrees,
                              aircraft=self.aircraft,
                              reverse=False)

            distanceStillToFlyMeters = self.flightLengthMeters - self.finalRoute.getLengthMeters()
            print("distanceStillToFlyMeters: ", distanceStillToFlyMeters)
            distanceToLastFixMeters = self.computeDistanceToLastFixMeters(currentPosition=endOfLastGreatCircleWayPoint,
                                                                          fixListIndex=self.flightListIndex)
            distanceToLastFixMeters = distanceStillToFlyMeters
            ''' for the last turn => final heading towards the runway orientation '''
            deltaTimeSeconds = 0.1
            turnLeg.buildTurnLeg(deltaTimeSeconds=deltaTimeSeconds,
                                 elapsedTimeSeconds=endOfLastGreatCircleWayPoint.getElapsedTimeSeconds(),
                                 distanceStillToFlyMeters=distanceStillToFlyMeters,
                                 distanceToLastFixMeters=distanceToLastFixMeters,
                                 finalHeadingDegrees=finalHeadingDegrees,
                                 lastTurn=True,
                                 bankAngleDegrees=5.0)
            self.finalRoute.addGraph(turnLeg)

            endOfTurnLegWayPoint = self.finalRoute.getLastVertex().getWeight()
            ''' ============= use touch-down way-point to compute distance to fly ============='''
            distanceStillToFlyMeters = endOfTurnLegWayPoint.getDistanceMetersTo(self.touchDownWayPoint)
            print(self.className + ': distance still to fly= {0:.2f} nautics'.format(
                distanceStillToFlyMeters * Meter2NauticalMiles))

            # print '==================== add descent slope ================= '
            descentGlideSlope = DescentGlideSlope(runway=self.arrivalRunway,
                                                  aircraft=self.aircraft,
                                                  arrivalAirport=self.arrivalAirport,
                                                  descentGlideSlopeDegrees=3.0)

            flownDistanceMeters = self.finalRoute.getLengthMeters()
            print("flowDistanceMeters", flownDistanceMeters)
            distanceStillToFlyMeters = self.flightLengthMeters - self.finalRoute.getLengthMeters()
            distanceToLastFixMeters = self.computeDistanceToLastFixMeters(currentPosition=endOfTurnLegWayPoint,
                                                                          fixListIndex=self.flightListIndex)
            distanceToLastFixMeters = distanceStillToFlyMeters

            descentGlideSlope.buildGlideSlope(deltaTimeSeconds=self.deltaTimeSeconds,
                                              elapsedTimeSeconds=endOfTurnLegWayPoint.getElapsedTimeSeconds(),
                                              initialWayPoint=endOfTurnLegWayPoint,
                                              flownDistanceMeters=flownDistanceMeters,
                                              distanceStillToFlyMeters=distanceStillToFlyMeters,
                                              distanceToLastFixMeters=distanceToLastFixMeters)
            self.finalRoute.addGraph(descentGlideSlope)
            endOfDescentGlideSlope = self.finalRoute.getLastVertex().getWeight()

            # print '================= add arrival ground run ================'
            arrivalGroundRun = GroundRunLeg(runway=self.arrivalRunway,
                                            aircraft=self.aircraft,
                                            airport=self.arrivalAirport)
            arrivalGroundRun.buildArrivalGroundRun(deltaTimeSeconds=self.deltaTimeSeconds,
                                                   elapsedTimeSeconds=endOfDescentGlideSlope.getElapsedTimeSeconds(),
                                                   initialWayPoint=endOfDescentGlideSlope)
            self.finalRoute.addGraph(arrivalGroundRun)

    def computeFlight(self, deltaTimeSeconds):

        # '''
        # main entry to compute a whole flight
        # '''
        self.deltaTimeSeconds = deltaTimeSeconds

        assert not (self.aircraft is None)
        assert not (self.departureRunway is None)
        assert not (self.departureAirport is None)

        if self.isDomestic() or self.isOutBound():
            initialHeadingDegrees , initialWayPoint = self.buildDeparturePhase()
            print(initialHeadingDegrees, initialWayPoint)
        if self.isDomestic() or self.isInBound():
            assert not(self.arrivalAirport is None)
            self.buildSimulatedArrivalPhase()

        #print '==================== Loop over the fix list ==================== '

        endOfSimulation, initialHeadingDegrees = self.loopThroughFixList(initialHeadingDegrees = initialHeadingDegrees,
                                                        elapsedTimeSeconds = initialWayPoint.getElapsedTimeSeconds())

        if (endOfSimulation == False):
            #print '=========== build arrival phase =============='
            self.buildArrivalPhase(initialHeadingDegrees)

        print ( self.className + ' ========== delta mass status ==============' )
        print ( self.className + ': initial mass= {0:.2f} kilograms = {1:.2f} pounds'.format(self.aircraft.getAircraftInitialMassKilograms(),
                                                                                           self.aircraft.getAircraftInitialMassKilograms()*Kilogram2Pounds) )
        print ( self.className + ': final mass= {0:.2f} kilograms = {1:.2f} pounds'.format(self.aircraft.getAircraftCurrentMassKilograms(),
                                                                                         self.aircraft.getAircraftCurrentMassKilograms()*Kilogram2Pounds) )
        print ( self.className + ': diff mass= {0:.2f} kilograms = {1:.2f} pounds'.format(self.aircraft.getAircraftInitialMassKilograms()-self.aircraft.getAircraftCurrentMassKilograms(),
                                                                                        (self.aircraft.getAircraftInitialMassKilograms()-self.aircraft.getAircraftCurrentMassKilograms())*Kilogram2Pounds) )
        print ( self.className + ' ========== delta mass status ==============' )

    def createFlightOutputFiles(self):
        ''' build outputs '''
        self.finalRoute.createXlsxOutputFile()
        self.finalRoute.createKmlOutputFile()
        ''' add a prefix to the file path to identify the departure and arrival airport '''
        # filePrefix = ''
        # if not(self.departureAirport is None):
        #     filePrefix = self.departureAirport.getICAOcode()
        #     if not(self.arrivalAirport is None):
        #         filePrefix += '-' + self.arrivalAirport.getICAOcode()
        # self.aircraft.createStateVectorOutputFile(filePrefix)
        print(self.className + ': final route length= {0} nautics'.format(
            self.finalRoute.getLengthMeters() * Meter2NauticalMiles))
