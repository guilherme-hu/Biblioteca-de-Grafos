#include "bib_grafo.h"
#include <bits/stdc++.h>
using namespace std;

// Função para executar o Ford-Fulkerson 10 vezes e medir o tempo de execução
void executarFordFulkerson(GrafoComPeso& g, int s, int t) {
    for (int i = 0; i < 10; ++i) {
        cout << "Execucao " << i + 1 << ": ";
        // clock_t start = clock();
        int fluxo_max = g.ford_fulkerson(s, t, 0).first;
        // clock_t end = clock();
        cout << "Fluxo maximo = " << fluxo_max << endl; 
            //  << ", Tempo de execução = " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;
    }
}

// main para testes gerais
int main() {
    cin.tie(NULL);
    ios_base::sync_with_stdio(false);

    for (int num = 1; num <= 6; ++num) {
        string FileName = "grafo_rf_" + to_string(num) + ".txt"; // nome do arquivo txt com o grafo
        int mode = 0; // 0 para lista de adjacência e 1 para matriz de adjacência

        GrafoComPeso g(FileName, mode, 1); // Criação do objeto grafo com peso

        // Executar o Ford-Fulkerson 10 vezes
        executarFordFulkerson(g, 1, 2); // Ajuste os vértices de origem e destino conforme necessário
    }

    return 0;
}

// g++ benchmarking.cpp -o benchmarking -O3
// ./benchmarking
