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
    
    // clock_t start = clock();

    for (int i = 1; i <= 5; ++i) {

        clock_t start = clock();

        string FileName = "grafo_W_" + to_string(i) + ".txt";
        int mode = 0; // 0 para lista de adjacência e 1 para matriz de adjacência

        GrafoComPeso g(FileName, mode); // Criação do objeto grafo

        for (int j = 20; j <= 60; j += 10) {
            cout << "Distancia entre 10 e " << j << ": " << g.distancia(10, j, 1, 1) << endl;
        }

        clock_t end = clock();

        // cout << "Tempo de execucao para " << FileName << ": " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl; // Tempo de execução
        cout << endl;
    }

    // int mode = 0;                               // 0 para lista de adjacência e 1 para matriz de adjacência

    // GrafoComPeso g(FileName, mode);                    // Criação do objeto grafo
    // // if(mode == 0) g.printListAdj(); 
    // // else g.printMatrizAdj();            

    // // cout << "Vertices: " << g.getV() << endl;                   // Print do número de vértices
    // // cout << "Aresta: " << g.getA() << endl;                     // Print do número de arestas
    // // cout << "Grau maximo: " << g.getGrauMax() << endl;          // Print do grau máximo
    // // cout << "Grau minimo: " << g.getGrauMin() << endl;          // Print do grau mínimo
    // // cout << "Grau mediano: " << g.getGrauMediano() << endl;     // Print do grau mediano
    // // cout << "Grau medio: " << g.getGrauMedio() << endl;         // Print do grau médio

  

    // g.distancia(10, 20, 1, 0);                                    // Print da distância entre os vértices 1 e 5                   

    // clock_t end = clock();

    // cout << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl; // Tempo de execução

    return 0;   

}

