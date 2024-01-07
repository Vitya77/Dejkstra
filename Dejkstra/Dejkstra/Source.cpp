#include <iostream>
#include <vector>
#include <limits>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace std;

const int INF = numeric_limits<int>::max(); // Нескінченність

class Graph {
private:
    vector<vector<int>> adjacencyMatrix;
    int numVertices;

public:
    Graph(int n) : numVertices(n) {
        adjacencyMatrix.resize(n, vector<int>(n, 0));
    }

    void addEdge(int source, int destination, int weight) {
        adjacencyMatrix[source][destination] = weight;
    }

    vector<int> dejkstra(int source) {
        vector<int> distance(numVertices, INF);
        vector<bool> visited(numVertices, false);
        distance[source] = 0;

        for (int i = 0; i < numVertices - 1; i++) {
            int minDistance = INF;
            int minIndex = -1;

            // Знайти найближчий непройдений вузол
            for (int j = 0; j < numVertices; j++) {
                if (!visited[j] && distance[j] < minDistance) {
                    minDistance = distance[j];
                    minIndex = j;
                }
            }

            visited[minIndex] = true;

            // Оновити відстані до сусідніх вузлів
            for (int k = 0; k < numVertices; k++) {
                if (!visited[k] && adjacencyMatrix[minIndex][k] != 0 &&
                    distance[minIndex] != INF && distance[minIndex] + adjacencyMatrix[minIndex][k] < distance[k]) {
                    distance[k] = distance[minIndex] + adjacencyMatrix[minIndex][k];
                }
            }
        }

        return distance;
    }
};


TEST_CASE("Dejkstra's algorithm") {
    Graph graph(6);

    graph.addEdge(0, 1, 2);
    graph.addEdge(0, 2, 4);
    graph.addEdge(1, 2, 1);
    graph.addEdge(1, 3, 7);
    graph.addEdge(2, 4, 3);
    graph.addEdge(3, 4, 1);
    graph.addEdge(3, 5, 5);
    graph.addEdge(4, 5, 2);

    SUBCASE("Shortest distances from vertex 0") {
        int source = 0;
        std::vector<int> distances = graph.dejkstra(source);

        CHECK(distances[0] == 0);
        CHECK(distances[1] == 2);
        CHECK(distances[2] == 3);
        CHECK(distances[3] == 9);
        CHECK(distances[4] == 6);
        CHECK(distances[5] == 8);
    }

    SUBCASE("Shortest distances from vertex 1") {
        int source = 1;
        std::vector<int> distances = graph.dejkstra(source);

        CHECK(distances[0] == INF);
        CHECK(distances[1] == 0);
        CHECK(distances[2] == 1);
        CHECK(distances[3] == 7);
        CHECK(distances[4] == 4);
        CHECK(distances[5] == 6);
    }
}

