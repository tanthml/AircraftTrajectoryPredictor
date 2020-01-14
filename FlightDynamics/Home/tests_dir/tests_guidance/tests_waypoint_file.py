import time
import unittest

from Home.Guidance.WayPointFile import WayPoint, Airport
from Home.Environment.RunWaysDatabaseFile import RunWayDataBase


class TestWayPointFile(unittest.TestCase):

    def test_waypoint(self):

        print("=========== WayPoint start  =========== " + time.strftime("%c"))
        London = WayPoint('London-Heathrow', 51.5, 0.0)
        Orly = WayPoint('Orly', 48.726254, 2.365247)
        print("distance from London to Orly= ",
              London.getDistanceMetersTo(Orly), " meters")
        print("bearing from London to Orly= ", London.getBearingDegreesTo(Orly),
              " degrees")

        # Zurich = WayPoint('Zurich-Kloten', 47.458215, 8.555424)
        Marseille = WayPoint('Marseille-Marignane', 43.438431, 5.214382)
        Zurich = WayPoint('Zurich-Kloten', 47.458215, 8.555424)

        print("=========== WayPoint resume  =========== " + time.strftime("%c"))

        print("distance from Marseille to Zurich= ",
              Marseille.getDistanceMetersTo(Zurich), " meters")
        print("bearing from Zurich to Marseille = ",
              Zurich.getBearingDegreesTo(Marseille), " degrees")

        distanceMeters = 321584.699454
        bearingDegrees = Zurich.getBearingDegreesTo(Marseille)
        # bearingDegrees = Marseille.getBearingDegreesTo(Zurich)
        Zurich.dump()
        Marseille.dump()
        TopOfDescent = Zurich.getWayPointAtDistanceBearing('TopOfDescent',
                                                           distanceMeters,
                                                           bearingDegrees)
        TopOfDescent.dump()

        print("=========== WayPoint resume  =========== " + time.strftime("%c"))
        London.dump()
        Orly.dump()
        bearingDegrees = Orly.getBearingDegreesTo(London)
        print("bearing from London to Orly= ", London.getBearingDegreesTo(Orly),
              " degrees")

        TopOfDescent = Orly.getWayPointAtDistanceBearing('TopOfDescent',
                                                         distanceMeters,
                                                         bearingDegrees)
        TopOfDescent.dump()

    def test_airport(self):

        print("=========== Airport  =========== " + time.strftime("%c"))

        airportICAOcode = 'LFPG'
        CharlesDeGaulle = Airport(Name='CharlesDeGaulle',
                                  ICAOcode=airportICAOcode,
                                  Country='France')
        self.assertTrue(not (CharlesDeGaulle is None))

        runWaysDatabase = RunWayDataBase()
        self.assertTrue(runWaysDatabase.read(), 'run ways DB read correctly')

        self.assertTrue(CharlesDeGaulle.hasRunWays(runWaysDatabase))
        print('airport= {0} has run-ways= {1}'.format(CharlesDeGaulle,
                                                      CharlesDeGaulle.hasRunWays(
                                                          runWaysDatabase)))

        print("=========== Airport run ways ONE =========== " + time.strftime(
            "%c"))

        for runway in CharlesDeGaulle.getRunWaysAsDict(runWaysDatabase):
            print(runway)

        print("=========== Airport run ways TWO =========== " + time.strftime(
            "%c"))

        for runway in CharlesDeGaulle.getRunWays(runWaysDatabase):
            print(runway)


if __name__ == '__main__':
    unittest.main()
