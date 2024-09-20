#include "bib_grafo.h"
#include <cassert>
#include <iostream>
#include <random>
#include <chrono>
#include <vector>
#include <fstream>


// main para testes gerais
int main() {
    cin.tie(NULL);
    ios_base::sync_with_stdio(false);
    
    clock_t start = clock();

    std::string FileName = "grafo_teste.txt";   // Nome do arquivo txt
    int mode = 0;                               // 0 para lista de adjacência e 1 para matriz de adjacência

    Grafo g(FileName, mode);                    // Criação do objeto grafo
    // if(mode == 0) g.printListAdj(); 
    // else g.printMatrizAdj();

    cout << "Vertices: " << g.getV() << endl;                   // Print do número de vértices
    cout << "Aresta: " << g.getA() << endl;                     // Print do número de arestas
    cout << "Grau maximo: " << g.getGrauMax() << endl;          // Print do grau máximo
    cout << "Grau minimo: " << g.getGrauMin() << endl;          // Print do grau mínimo
    cout << "Grau mediano: " << g.getGrauMediano() << endl;     // Print do grau mediano
    cout << "Grau medio: " << g.getGrauMedio() << endl;         // Print do grau médio

    g.printCompCon();                                        // Print do número de componentes conexas e o tamanho de cada uma
    
    g.bfs(10);                                              // BFS a partir do vértice 10
    g.dfs(10);                                              // DFS a partir do vértice 10
    cout << "Distancia: " << g.distancia(10, 15) << endl;   // Distância entre os vértices 10 e 15

    cout << "Diametro: " << g.diametro_aprox() << endl;     // Diâmetro do grafo de forma aproximada

    clock_t end = clock();

    cout << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl; // Tempo de execução

    return 0;   

}


//  g++ main.cpp -o main -O3
// ./main
