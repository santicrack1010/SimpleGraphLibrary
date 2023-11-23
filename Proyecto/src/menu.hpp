#ifndef __MENU_H__
#define __MENU_H__

#include "libreriaGrafos.hpp"

void menuPrim() {}

void menuKruskal() {}

void menuFloyd() {}

void mostrarMenu() {
  int numVertices;
  std::cout << "\t\nIngrese el numero de vertices: ";
  std::cin >> numVertices;

  Graph graph(numVertices);

  int choice;
  do {
    std::cout << "\n[1] Agregar vertice\n"
              << "[2] Eliminar vertice\n"
              << "[3] Agregararista"
              << "[4] Eliminar arista\n"
              << "[5] Dijkstra\n"
              << "[6] Prim-Kruskal\n"
              << "[7] Floyd-Warshall\n"
              << "[8] Imprimir grafo\n"
              << "[9] Salir\n";
    std::cout << "Elija una opcion: ";
    std::cin >> choice;

    switch (choice) {
      case 1:
        graph.addVertex();
        break;
      case 2:
        int vertex;
        std::cout << "Ingrese el vertice a eliminar: ";
        std::cin >> vertex;
        graph.removeVertex(vertex);
        break;
      case 3:
        int source, destination, weight;
        std::cout << "Ingrese origen, destino y peso de la arista: ";
        std::cin >> source >> destination >> weight;
        graph.addEdge(source, destination, weight);
        break;
      case 4:
        int sourceE, destinationE;
        std::cout << "Ingrese origen y destino de la arista a eliminar: ";
        std::cin >> source >> destination;
        graph.removeEdge(source, destination);
        break;
      case 5:
        int sourceD;
        std::cout << "Ingrese el vertice de origen para Dijkstra: ";
        std::cin >> source;
        graph.dijkstra(source);
        break;
      case 6:
        graph.primKruskal();
        break;
      case 7:
        graph.floydWarshall();
        break;
      case 8:
        graph.printGraph();
        break;
      case 9:
        std::cout << "\nGracias por usar mi libreria :)";
        break;
      default:
        std::cout << "Opcion no valida.\n";
        break;
    }
  } while (choice != 9);
}

#endif  // __MENU_H__