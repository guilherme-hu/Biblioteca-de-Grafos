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
int V;                               // Número de vértices
int A = 0;                           // Número de arestas
vector<vector<int>> adj(V+1);        // Lista de adjacências
vector<vector<int>> mat(V+1);        // Matriz de adjacências
bool mode;                           // 0 para lista e 1 para matriz


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
    for (int i = 0; i < V+1; ++i) {
        cout << "Vertice " << i << ":";
        for (int vizinho : adj[i]) {
            cout << " " << vizinho;
        }
        cout << endl;
    }
}

void printMatrizAdj() {
    cout << "Matriz de Adjacencias:" << endl;
    for (int i = 0; i < V+1; ++i) {
        for (int j = 0; j < V+1; ++j) {
            cout << mat[i][j] << " ";
        }
        cout << endl;
    }
}

int main() {
    cin.tie(NULL);
    ios_base::sync_with_stdio(false);

    // string FileName; cin >> FileName;
    string FileName = "grafo_6.txt";
    std::ifstream arquivo(FileName);

    // cin >> mode;
    mode = 0; // 0 para lista e 1 para matriz

    clock_t start = clock();

    cout << "Pegando dados do txt..." << endl;
    if(!arquivo.is_open()){
        std::cerr << "Não foi possível abrir o arquivo" << "\n";
    }
    else{
        int v1, v2;
        arquivo >> V;
        if (mode == 0) adj.resize(V+1);
        else mat.resize(V+1, vector<int>(V+1, 0));

        while(arquivo >> v1 >> v2){
            addEdge(0, v1, v2);
            A++;
        }
        arquivo.close();
    }

    clock_t end = clock();

    cout << "Grafo transferido!" << endl;
    // printListAdj();
    cout << "Numero de vertices no grafo: " << V << endl;
    cout << "Numero de arestas no grafo: " << A << endl;
    cout << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;

    // Criar arquivo de saída com as informações
    std::ofstream outputFile("grafo_info.txt");
    if (outputFile.is_open()) {
        outputFile << "Numero de vertices no grafo: " << V << endl;
        outputFile << "Numero de arestas no grafo: " << A << endl;
        outputFile << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;
        outputFile.close();
        cout << "Informacoes salvas em grafo_info.txt" << endl;
    } 
    else {
        std::cerr << "Nao foi possível criar o arquivo de saida" << "\n";
    }

    return 0;

}

// g++ function.cpp -o function -O2
// ./function.exe
