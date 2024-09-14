#include <bits/stdc++.h>
using namespace std;
#include <iostream> //Biblioteca para o uso do cout
#include <stdio.h> //Biblioteca para usar o get char
#include <ctime> //Biblioteca para medir o tempo de execução
#include <fstream> //Biblioteca para escrita e leitura de arquivos txt
#include <stack> //Biblioteca com implementação de pilha
#include <queue> //Biblioteca com implementação de fila
#include <vector> //Biblioteca com implementação de array dinâmico
#include <algorithm> //Biblioteca com implementação do sort
#include <string.h> //Biblioteca com implementação do memset
#include <utility>  //Biblioteca com implementação do pair
#include <set> //Biblioteca com implementação do set
#include <deque> //Biblioteca com implementação de deque

#define INF 0x3f3f3f3f //Define um valor infinito


// Variáveis globais
int V;                          // Número de vértices
int A = 0;                      // Número de arestas
vector<vector<int>> adj(INF);   // Lista de adjacências
vector<vector<int>> mat(V);     // Matriz de adjacências


void addEdge(int mode, int v1, int v2) { // mode = 0 para representação em lista e mode = 1 para representação em matriz
    if (mode == 0){
        adj[v1].push_back(v2);
        adj[v2].push_back(v1); 
    }
    else{
        mat[v1][v2] = 1;
        mat[v2][v1] = 1;
    }
}

void printListAdj() {
    for (int i = 0; i < V; ++i) {
        cout << "Vertice " << i << ":";
        for (int vizinho : adj[i]) {
            cout << " " << vizinho;
        }
        cout << endl;
    }
}

void printMatrizAdj() {
    cout << "Matriz de Adjacencias:" << endl;
    for (int i = 0; i < V; ++i) {
        for (int j = 0; j < V; ++j) {
            cout << mat[i][j] << " ";
        }
        cout << endl;
    }
}

int main() {
    cin.tie(NULL);
    ios_base::sync_with_stdio(false);

    // string FileName; cin >> FileName;
    string FileName = "grafo_1.txt";
    std::ifstream arquivo(FileName);
    string linha;

    cout << "inicio" << endl;
    if(!arquivo.is_open()){
        std::cerr << "Não foi possível abrir o arquivo" << "\n";
    }
    else{
        int v1, v2;
        arquivo >> V;

        while(getline(arquivo,linha)){
            std::istringstream iss(linha);
            if(iss >> v1 >> v2){
                addEdge(0, v1, v2);
                A++;
            }
        }
        arquivo.close();
    }

    cout << "Grafo transferido" << endl;
    printListAdj();
    cout << "Numero de vertices no grafo: " << V << endl;
    cout << "Numero de arestas no grafo: " << A << endl;
    return 0;

}

// g++ function.cpp -o function -O2
// ./function.exe
