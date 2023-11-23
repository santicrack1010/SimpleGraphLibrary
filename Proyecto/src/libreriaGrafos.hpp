#ifndef __LIBRERIAGRAFOS_H__
#define __LIBRERIAGRAFOS_H__

#include <climits>
#include <iostream>
#include <set>
#include <vector>

class Graph {
 public:
  Graph(int numVertices);
  void addEdge(int source, int destination, int weight);
  void removeEdge(int source, int destination);
  void addVertex();
  void removeVertex(int vertex);
  void dijkstra(int source);
  void primKruskal();
  void floydWarshall();
  void printGraph();

 private:
  std::vector<std::vector<std::pair<int, int>>> adjacencyList;
  std::vector<std::vector<int>> adjacencyMatrix;
};

Graph::Graph(int numVertices) {
  adjacencyList.resize(numVertices);
  adjacencyMatrix.resize(numVertices, std::vector<int>(numVertices, INT_MAX));
}

void Graph::addEdge(int source, int destination, int weight) {
  adjacencyList[source].emplace_back(destination, weight);
  adjacencyList[destination].emplace_back(source, weight);
  adjacencyMatrix[source][destination] = weight;
  adjacencyMatrix[destination][source] = weight;
}

void Graph::removeEdge(int source, int destination) {
  adjacencyList[source].erase(
      std::remove_if(adjacencyList[source].begin(), adjacencyList[source].end(),
                     [destination](const std::pair<int, int>& edge) {
                       return edge.first == destination;
                     }),
      adjacencyList[source].end());

  adjacencyList[destination].erase(
      std::remove_if(adjacencyList[destination].begin(),
                     adjacencyList[destination].end(),
                     [source](const std::pair<int, int>& edge) {
                       return edge.first == source;
                     }),
      adjacencyList[destination].end());

  adjacencyMatrix[source][destination] = INT_MAX;
  adjacencyMatrix[destination][source] = INT_MAX;
}

void Graph::addVertex() {
  int numVertices = adjacencyList.size();
  adjacencyList.emplace_back();
  for (int i = 0; i < numVertices; ++i) {
    adjacencyMatrix[i].push_back(INT_MAX);
  }
  adjacencyMatrix.emplace_back(numVertices, INT_MAX);
}

void Graph::removeVertex(int vertex) {
  int numVertices = adjacencyList.size();
  adjacencyList.erase(adjacencyList.begin() + vertex);
  for (int i = 0; i < numVertices; ++i) {
    adjacencyMatrix[i].erase(adjacencyMatrix[i].begin() + vertex);
  }
  adjacencyMatrix.erase(adjacencyMatrix.begin() + vertex);
}

void Graph::dijkstra(int source) {
  std::vector<int> distance(adjacencyList.size(), INT_MAX);
  std::vector<bool> visited(adjacencyList.size(), false);

  distance[source] = 0;

  for (int i = 0; i < adjacencyList.size(); i++) {
    int minDistance = INT_MAX;
    int minIndex = -1;

    for (int j = 0; j < adjacencyList.size(); j++) {
      if (!visited[j] && distance[j] < minDistance) {
        minDistance = distance[j];
        minIndex = j;
      }
    }

    if (minIndex == -1) break;
    visited[minIndex] = true;

    for (const auto& edge : adjacencyList[minIndex]) {
      int neighbor = edge.first;
      int weight = edge.second;
      if (!visited[neighbor] && distance[minIndex] != INT_MAX &&
          distance[minIndex] + weight < distance[neighbor]) {
        distance[neighbor] = distance[minIndex] + weight;
      }
    }
  }

  std::cout << "Distancias mas cortas desde el vertice " << source << ":\n";
  for (int i = 0; i < adjacencyList.size(); i++) {
    std::cout << "Hasta vertice " << i << ": " << distance[i] << std::endl;
  }
}

void Graph::primKruskal() {
  // Implementación del algoritmo Prim-Kruskal
  int numVertices = adjacencyList.size();
  std::vector<int> parent(numVertices, -1);
  std::vector<int> key(numVertices, INT_MAX);
  std::vector<bool> inMST(numVertices, false);

  std::set<std::pair<int, int>> pq;

  key[0] = 0;
  pq.insert({0, 0});

  while (!pq.empty()) {
    int u = pq.begin()->second;
    pq.erase(pq.begin());
    inMST[u] = true;

    for (const auto& edge : adjacencyList[u]) {
      int v = edge.first;
      int weight = edge.second;
      if (!inMST[v] && weight < key[v]) {
        pq.erase({key[v], v});
        key[v] = weight;
        parent[v] = u;
        pq.insert({key[v], v});
      }
    }
  }

  std::cout << "Aristas del Arbol de Expansion Minima (MST):\n";
  for (int i = 1; i < numVertices; i++) {
    std::cout << "Arista: " << parent[i] << " - " << i << " Peso: " << key[i]
              << std::endl;
  }
}

void Graph::floydWarshall() {
  int numVertices = adjacencyList.size();
  std::vector<std::vector<int>> distance(adjacencyMatrix);

  for (int k = 0; k < numVertices; k++) {
    for (int i = 0; i < numVertices; i++) {
      for (int j = 0; j < numVertices; j++) {
        if (distance[i][k] != INT_MAX && distance[k][j] != INT_MAX &&
            distance[i][k] + distance[k][j] < distance[i][j]) {
          distance[i][j] = distance[i][k] + distance[k][j];
        }
      }
    }
  }

  std::cout << "Matriz de distancias de Floyd-Warshall:\n";
  for (int i = 0; i < numVertices; i++) {
    for (int j = 0; j < numVertices; j++) {
      if (distance[i][j] == INT_MAX) {
        std::cout << "INF\t";
      } else {
        std::cout << distance[i][j] << "\t";
      }
    }
    std::cout << std::endl;
  }
}

void Graph::printGraph() {
  std::cout << "Lista de adyacencia del grafo:\n";
  for (int i = 0; i < adjacencyList.size(); i++) {
    std::cout << "Vértice " << i << ": ";
    for (const auto& edge : adjacencyList[i]) {
      std::cout << "(" << edge.first << ", " << edge.second << ") ";
    }
    std::cout << std::endl;
  }

  std::cout << "Matriz de adyacencia del grafo:\n";
  for (int i = 0; i < adjacencyMatrix.size(); i++) {
    for (int j = 0; j < adjacencyMatrix[i].size(); j++) {
      if (adjacencyMatrix[i][j] == INT_MAX) {
        std::cout << "INF\t";
      } else {
        std::cout << adjacencyMatrix[i][j] << "\t";
      }
    }
    std::cout << std::endl;
  }
}

#endif  // __LIBRERIAGRAFOS_H__