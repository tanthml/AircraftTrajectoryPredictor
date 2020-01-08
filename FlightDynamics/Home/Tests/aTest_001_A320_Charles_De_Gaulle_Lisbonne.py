'''
Created on 6 janvier 2015

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
'''

import time
import unittest

from Home.Guidance.FlightPathFile import FlightPath


Meter2Feet = 3.2808 # one meter equals 3.28 feet
Meter2NauticalMiles = 0.000539956803 # One Meter = 0.0005 nautical miles
NauticalMiles2Meter = 1852
Knots2MetersPerSecond = 0.514444444

class Test_Route(unittest.TestCase):


    def test_route(self):

        print ( "=========== Flight Plan start  =========== "  )

        strRoute = 'ADEP/LFPG/26R-LAIGLE-ROLEN-PEPON-KURIS-TERPO-ERIGA-INBAB-ATLEN-DEVAR-ASTURIAS-KUVAN-BISMU-BARKO-FATIMA-ADES/LPPT/03'
        flightPath = FlightPath(route = strRoute,
                                aircraftICAOcode = 'A320',
                                RequestedFlightLevel = 330,
                                cruiseMach = 0.82,
                                takeOffMassKilograms = 68000.0,
                                windSpeedMetersPerSecond=25* Knots2MetersPerSecond,
                                windDirectionDegrees=25
        )

        # # If we want to generate the whole flight using the old code, use the .computeFlight function
        flightPath.computeFlight(deltaTimeSeconds = 1.0)
        flightPath.createFlightOutputFiles()


        # If we want to fly through the flight and generate the configuration for later fly, use .fly()
        flyAndGenerateConfig = FlightPath(route = strRoute,
                                          aircraftICAOcode = 'A320',
                                          RequestedFlightLevel = 330,
                                          cruiseMach = 0.82,
                                          takeOffMassKilograms = 68000.0,
                                          windSpeedMetersPerSecond=25* Knots2MetersPerSecond,
                                          windDirectionDegrees=25)
        flyAndGenerateConfig.fly()


        # If we want to simulate the fly based on previous configuration, use .simulateFly
        strRoute = 'ADEP/LFPG/26R-LAIGLE-ROLEN-PEPON-KURIS-TERPO-ERIGA-INBAB-ATLEN-DEVAR-ASTURIAS-KUVAN-BISMU-BARKO-ADES/LPPT/03'
        updateFlightPath = FlightPath(route = strRoute,
                                     aircraftICAOcode = 'A320',
                                     RequestedFlightLevel = 330,
                                     cruiseMach = 0.82,
                                     takeOffMassKilograms = 68000.0,
                                     windSpeedMetersPerSecond=25* Knots2MetersPerSecond,
                                     windDirectionDegrees=25
        )

        # fly from index 4 for the new route
        updateFlightPath.simulateFly(4, strRoute)
        # fly from index 4 for the new route, with speed 260 and altitude 10000
        updateFlightPath.simulateFly(4, strRoute, 260, 10000.0)

        # courseDegrees = 20.71
        # windSpeedKnots = 25.0

        # print(time.clock() - t0)


if __name__ == '__main__':
    unittest.main()
