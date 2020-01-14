import time
import unittest

from Home.Environment.AirportDatabaseFile import AirportsDatabase
from Home.Guidance.GraphFile import Graph, Vertex


class TestGraphFile(unittest.TestCase):

    def test_graph_one_vertex(self):
        g1 = Graph()
        print('empty graph= ', g1)
        self.assertEqual(str(g1), 'Graph: number of vertices= 0')
        v1 = Vertex('Robert')
        g1.addVertex(v1)
        print(g1)
        self.assertEqual(str(g1), 'Graph: number of vertices= 1')
        print('number of vertices: {0}'.format(g1.getNumberOfVertices()))
        self.assertEqual(g1.getNumberOfVertices(), 1)
        print(g1.getLastVertex().getWeight())
        self.assertEqual(str(g1.getLastVertex().getWeight()), 'Vertex: vertex= Robert')
        print(g1.getVertex(0).getWeight())
        self.assertEqual(str(g1.getVertex(0).getWeight()), 'Vertex: vertex= Robert')

    def test_graph_two_vertexs(self):
        g1 = Graph()
        v1 = Vertex('Robert')
        v2 = Vertex('Francois')
        g1.addVertex(v1)
        g1.addVertex(v2)
        print('number of vertices: {0}'.format(g1.getNumberOfVertices()))
        self.assertEqual(g1.getNumberOfVertices(), 2)
        print('number of edges: {0}'.format(g1.getNumberOfEdges()))
        self.assertEqual(g1.getNumberOfEdges(), 1)
        print(g1.getLastVertex().getWeight())

    def test_graph_three_vertexs(self):

        g2 = Graph()
        v3 = Vertex('Marie')
        g2.addVertex(v3)

        g1 = Graph()
        v1 = Vertex('Robert')
        v2 = Vertex('Francois')
        g1.addVertex(v1)
        g1.addVertex(v2)

        g1.addGraph(g2)
        print(g1)
        self.assertEqual(g1.getNumberOfVertices(), 3)

        for vertex in g1.getVertices():
            print(vertex)
        print("=================")
        for edge in g1.getEdges():
            print(edge.getTail(), edge.getHead())

    def test_graph_with_airport(self):

        print(" ========== AirportsDatabase testing ======= time start= ")
        airportsDb = AirportsDatabase()
        assert (airportsDb.read())

        airportsDb.dumpCountry(Country="France")
        print("number of airports= ", airportsDb.getNumberOfAirports())

        for ap in ['Orly', 'paris', 'toulouse', 'marseille', 'roissy',
                   'blagnac', 'provence', 'de gaulle']:
            print("ICAO Code of= ", ap, " ICAO code= ",
                  airportsDb.getICAOCode(ap))

        t1 = time.clock()
        print(" ========== AirportsDatabase testing ======= time start= ", t1)
        CharlesDeGaulleRoissy = airportsDb.getAirportFromICAOCode('LFPG')
        print(CharlesDeGaulleRoissy)
        MarseilleMarignane = airportsDb.getAirportFromICAOCode('LFML')
        print(MarseilleMarignane)

        g0 = Graph()
        for icao in ['LFPO', 'LFMY', 'LFAT', 'LFGJ']:
            airport = airportsDb.getAirportFromICAOCode(icao)
            g0.addVertex(airport)
        print('================ g0 =================')
        for node in g0.getVertices():
            print(node)
        self.assertEqual(g0.getNumberOfVertices(), 4)

        g1 = Graph()
        for icao in ['LFKC', 'LFBO', 'LFKB']:
            airport = airportsDb.getAirportFromICAOCode(icao)
            g1.addVertex(airport)
        self.assertEqual(g1.getNumberOfVertices(), 3)

        print('================ g1 =================')
        for node in g1.getVertices():
            print(node)

        print(' ============== g0.add_graph(g1) ===============')
        g0.addGraph(g1)
        for node in g0.getVertices():
            print(node)

        self.assertEqual(g0.getNumberOfVertices(), 7)

        print('============== g0.create XLS file ===============')

        g0.createXlsxOutputFile()
        g0.createKmlOutputFile()

    def test_performance_graph_with_airport(self):

        airportsDb = AirportsDatabase()
        assert (airportsDb.read())

        print(' ============== g3 performance ===============')
        t0 = time.process_time()
        g3 = Graph()
        count = 0
        for airport in airportsDb.getAirports():
            print(airport)
            g3.addVertex(airport)
            count += 1
        t1 = time.process_time()
        print('number of airports= {0} - duration= {1} seconds'.format(
            count, t1 - t0)
        )
        self.assertEqual(g3.getNumberOfVertices(), count)

        g3.createXlsxOutputFile()
        g3.createKmlOutputFile()

        print(' ============== g4 performance ===============')
        airport = airportsDb.getAirportFromICAOCode('LFPG')
        t2 = time.process_time()
        g4 = Graph()
        for i in range(0, 10000):
            g4.addVertex(airport)
        t3 = time.process_time()
        print('number of addVertex = {0} - duration= {1:.8f} seconds'.format(
            i, t3 - t2)
        )


if __name__ == '__main__':
    unittest.main()
