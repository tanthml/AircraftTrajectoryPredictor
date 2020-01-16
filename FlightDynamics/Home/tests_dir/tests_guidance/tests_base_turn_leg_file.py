import time
import unittest

from Home.Guidance.BaseTurnLegFile import BaseTurnLeg


class TestBaseTurnLeg(unittest.TestCase):

    def test_positive_increase(self):

        print(
            "=== Base Turn Leg testing Positive Increase ===" + time.strftime(
                "%c"))

        baseTurnLeg = BaseTurnLeg(150.0, 190.0, 1.0)
        ret = baseTurnLeg.build()
        # print(baseTurnLeg)
        expectation = [150.0, 151.0, 152.0, 153.0, 154.0, 155.0, 156.0, 157.0,
                       158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0,
                       166.0, 167.0, 168.0, 169.0, 170.0, 171.0, 172.0, 173.0,
                       174.0, 175.0, 176.0, 177.0, 178.0, 179.0, 180.0, 181.0,
                       182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0,
                       190.0]

        self.assertListEqual(ret, expectation)

    def test_positive_decrease(self):
        print(
            "=== Base Turn Leg testing Positive Decrease  ===" + time.strftime(
                "%c"))

        baseTurnLeg = BaseTurnLeg(350.0, 10.0, 1.0)
        ret = baseTurnLeg.build()
        # print(baseTurnLeg)
        expectation = [350.0, 351.0, 352.0, 353.0, 354.0, 355.0, 356.0, 357.0,
                       358.0, 359.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                       8.0, 9.0, 10.0]
        self.assertListEqual(ret, expectation)

    def test_negative_increase(self):
        print(
            "=== Base Turn Leg testing Negative Increase ===" + time.strftime(
                "%c"))
        baseTurnLeg = BaseTurnLeg(10.0, 350.0, -1.0)
        ret = baseTurnLeg.build()
        # print(baseTurnLeg)
        expectation = [10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 360.0,
                       359.0, 358.0, 357.0, 356.0, 355.0, 354.0, 353.0, 352.0,
                       351.0, 350.0]
        self.assertListEqual(ret, expectation)

    def test_negative_decrease(self):
        print(
            "=========== Base Turn Leg testing   =========== " + time.strftime(
                "%c"))
        baseTurnLeg = BaseTurnLeg(270.0, 80.0, -1.0)
        ret = baseTurnLeg.build()
        # print(baseTurnLeg)
        expectation = [270.0, 269.0, 268.0, 267.0, 266.0, 265.0, 264.0, 263.0,
                       262.0, 261.0, 260.0, 259.0, 258.0, 257.0, 256.0, 255.0,
                       254.0, 253.0, 252.0, 251.0, 250.0, 249.0, 248.0, 247.0,
                       246.0, 245.0, 244.0, 243.0, 242.0, 241.0, 240.0, 239.0,
                       238.0, 237.0, 236.0, 235.0, 234.0, 233.0, 232.0, 231.0,
                       230.0, 229.0, 228.0, 227.0, 226.0, 225.0, 224.0, 223.0,
                       222.0, 221.0, 220.0, 219.0, 218.0, 217.0, 216.0, 215.0,
                       214.0, 213.0, 212.0, 211.0, 210.0, 209.0, 208.0, 207.0,
                       206.0, 205.0, 204.0, 203.0, 202.0, 201.0, 200.0, 199.0,
                       198.0, 197.0, 196.0, 195.0, 194.0, 193.0, 192.0, 191.0,
                       190.0, 189.0, 188.0, 187.0, 186.0, 185.0, 184.0, 183.0,
                       182.0, 181.0, 180.0, 179.0, 178.0, 177.0, 176.0, 175.0,
                       174.0, 173.0, 172.0, 171.0, 170.0, 169.0, 168.0, 167.0,
                       166.0, 165.0, 164.0, 163.0, 162.0, 161.0, 160.0, 159.0,
                       158.0, 157.0, 156.0, 155.0, 154.0, 153.0, 152.0, 151.0,
                       150.0, 149.0, 148.0, 147.0, 146.0, 145.0, 144.0, 143.0,
                       142.0, 141.0, 140.0, 139.0, 138.0, 137.0, 136.0, 135.0,
                       134.0, 133.0, 132.0, 131.0, 130.0, 129.0, 128.0, 127.0,
                       126.0, 125.0, 124.0, 123.0, 122.0, 121.0, 120.0, 119.0,
                       118.0, 117.0, 116.0, 115.0, 114.0, 113.0, 112.0, 111.0,
                       110.0, 109.0, 108.0, 107.0, 106.0, 105.0, 104.0, 103.0,
                       102.0, 101.0, 100.0, 99.0, 98.0, 97.0, 96.0, 95.0, 94.0,
                       93.0, 92.0, 91.0, 90.0, 89.0, 88.0, 87.0, 86.0, 85.0,
                       84.0, 83.0, 82.0, 81.0, 80.0]

        self.assertListEqual(ret, expectation)

    def test_false_input_assert(self):
        print(
            "=========== Base Turn Leg testing   =========== " + time.strftime(
                "%c"))
        try:
            BaseTurnLeg(361.0, 0.0, 0.0)
            print(" not false")
            self.assertFalse(True)
        except:
            print(" except")
            self.assertTrue(True)

    def test_turn_leg(self):
        print(
            "=========== Base Turn Leg testing   =========== " + time.strftime(
                "%c"))
        baseTurnLeg = BaseTurnLeg(22.0, 87.0, -3.0)
        ret = baseTurnLeg.build()
        print(baseTurnLeg)


if __name__ == '__main__':
    unittest.main()
