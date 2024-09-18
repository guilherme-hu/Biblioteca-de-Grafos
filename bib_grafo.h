#ifndef BIB_GRAFO_H
#define BIB_GRAFO_H


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
#include <cstdlib> //Biblioteca para usar o system

#define INF 0x3f3f3f3f //Define um valor infinito


class Grafo {
private:  
    int V;                                  // Número de vértices
    int A = 0;                              // Número de arestas
    int mode;                               // 0 para lista e 1 para matriz
    int grauMax;
    int grauMin;
    double grauMediano;
    double grauMedio;
    int first_diameter = 0;                 // Diâmetro inicial
    pair<int,int> maior_dist = {-1,0};      // Vértice com maior distância e sua distância, para calculo do diâmetro pelas 2 bfs
    vector<vector<int>> adj;                // Lista de adjacências
    vector<vector<bool>> mat;               // Matriz de adjacências
    vector<int> grau;                       // Vetor com os graus de cada vértice
    vector<bool> vis;                       // Vetor de visitados
    vector<int> pai;                        // Vetor com o vértice pai de cada vértice
    vector<int> nivel;                      // Vetor com o nível de cada vértice
    vector<int> dist;                       // Vetor com o nível de cada vértice (compcon)
    vector<pair<int,vector<int>>> compCon;  // Componentes conexas, com o tamanho e os vértices de cada uma

public:
    Grafo(string FileName, int mode);
    // ~Grafo();
    void addEdge(int v1, int v2, int mode);
    vector<int> bfs_CompCon(int s);
    void bfs(int s, int print = 0);
    void dfs(int s, int print = 0);
    int distancia(int v, int u);
    int diametro();
    void geradortxt();
    void printListAdj();
    void printMatrizAdj();


    // Métodos getters
    int getV() const { return V; }
    int getA() const { return A; }
    int getGrauMax() const { return grauMax; }
    int getGrauMin() const { return grauMin; }
    double getGrauMediano() const { return grauMediano; }
    double getGrauMedio() const { return grauMedio; }
    vector<pair<int,vector<int>>> getCompCon() const { return compCon; }
};

#endif // GRAFO_H
