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

    string FileName = "grafo_teste_W.txt";      // nome do arquivo txt com o grafo
    int mode = 0;                               // 0 para lista de adjacência e 1 para matriz de adjacência

    GrafoComPeso g(FileName, mode);                    // Criação do objeto grafo com peso
    // if(mode == 0) g.printListAdj(); 
    // else g.printMatrizAdj();            

    cout << "Vertices: " << g.getV() << endl;                   // Print do número de vértices
    cout << "Aresta: " << g.getA() << endl;                     // Print do número de arestas
    cout << "Grau maximo: " << g.getGrauMax() << endl;          // Print do grau máximo
    cout << "Grau minimo: " << g.getGrauMin() << endl;          // Print do grau mínimo
    cout << "Grau mediano: " << g.getGrauMediano() << endl;     // Print do grau mediano
    cout << "Grau medio: " << g.getGrauMedio() << endl;         // Print do grau médio

    cout << g.distancia(10, 30, 0, 1) << endl;                  // Distância mínima entre os vértices 10 e 30

    clock_t end = clock();

    cout << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl; // Tempo de execução

    return 0;   

}
