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
int mode;


class Grafo {
private:
    int V;                                  // Número de vértices
    int A = 0;                              // Número de arestas
    int grauMax;
    int grauMin;
    double grauMediano;
    double grauMedio;
    vector<vector<int>> adj;                // Lista de adjacências
    vector<vector<int>> mat;                // Matriz de adjacências
    vector<int> grau;                       // Vetor com os graus de cada vértice
public:
    Grafo(string FileName, int mode);
    void addEdge(int v1, int v2, int mode);
    int getGrauMax();
    void printListAdj();
    void printMatrizAdj();

    // Métodos getters
    int getV() const { return V; }
    int getA() const { return A; }
    int getGrauMax() const { return grauMax; }
    int getGrauMin() const { return grauMin; }
    double getGrauMediano() const { return grauMediano; }
    double getGrauMedio() const { return grauMedio; }
};

Grafo::Grafo(string FileName, int mode) {
    std::ifstream arquivo(FileName);
    cout << "Pegando dados do txt..." << endl;
    if(!arquivo.is_open()){
        std::cerr << "Não foi possível abrir o arquivo" << "\n";
    }
    else{
        int v1, v2;
        arquivo >> V;
        if (mode == 0) adj.resize(V);
        else mat.resize(V, vector<int>(V, 0));
        grau.resize(V, 0);

        while(arquivo >> v1 >> v2){
            addEdge(v1, v2, mode);
            A++;
        }
        arquivo.close();
    }
    // this->V = V;

    grauMax = getGrauMax();
    grauMedio = 2*A/V;  
}

void Grafo::addEdge(int v1, int v2, int mode) { // mode = 0 para representação em lista e mode = 1 para representação em matriz
    v1--; v2--;
    if (mode == 0){
        adj[v1].push_back(v2);
        adj[v2].push_back(v1); 
    }
    else{
        mat[v1][v2] = 1;
        mat[v2][v1] = 1;
    }
}

int Grafo::getGrauMax() {
    int grauMax = 0;
    for (int i = 0; i < V; ++i) {
        grauMax = max(grauMax, (int)adj[i].size());
    }
    return grauMax;
}

void Grafo::printListAdj() {
    for (int i = 0; i < V; ++i) {
        cout << "Vertice " << i+1 << ":";
        for (int vizinho : adj[i]) {
            cout << " " << vizinho+1;
        }
        cout << endl;
    }
}

void Grafo::printMatrizAdj() {
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
    
    // cin >> mode;     // 0 para lista e 1 para matriz
    mode = 0;

    clock_t start = clock();

    Grafo g(FileName, mode);
    if(mode == 0) g.printListAdj(); 
    else g.printMatrizAdj();

    clock_t end = clock();

    cout << g.getV() << " " << g.getA() << endl;
    cout << "Grau maximo: " << g.getGrauMax() << endl;   
    cout << "Grau medio: " << g.getGrauMedio() << endl;
    cout << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;

    // Criar arquivo de saída com as informações
    // std::ofstream outputFile("grafo_info.txt");
    // if (outputFile.is_open()) {
    //     outputFile << "Numero de vertices no grafo: " << V << endl;
    //     outputFile << "Numero de arestas no grafo: " << A << endl;
    //     outputFile << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;
    //     outputFile.close();
    //     cout << "Informacoes salvas em grafo_info.txt" << endl;
    // } 
    // else {
    //     std::cerr << "Nao foi possível criar o arquivo de saida" << "\n";
    // }
    
    return 0;

}

// g++ bib_grafo.cpp -o bib_grafo -O2
// ./bib_grafo
