#ifndef BIB_GRAFO_H
#define BIB_GRAFO_H


#include <bits/stdc++.h>
using namespace std;
#include <iostream>     // Biblioteca para o uso do cout
#include <stdio.h>      // Biblioteca para usar o get char
#include <ctime>        // Biblioteca para medir o tempo de execução
#include <fstream>      // Biblioteca para escrita e leitura de arquivos txt
#include <stack>        // Biblioteca com implementação de pilha
#include <queue>        // Biblioteca com implementação de fila
#include <vector>       // Biblioteca com implementação de array dinâmico
#include <algorithm>    // Biblioteca com implementação do sort
#include <string.h>     // Biblioteca com implementação do memset
#include <utility>      // Biblioteca com implementação do pair
#include <set>          // Biblioteca com implementação do set
#include <deque>        // Biblioteca com implementação de deque
#include <random>       // Biblioteca com implementação de números aleatórios

#define INF 0x3f3f3f3f  // Define um valor infinito


class Grafo {
private:  
    int V;                                      // Número de vértices
    int A = 0;                                  // Número de arestas
    int mode;                                   // 0 para lista e 1 para matriz
    int grauMax;                                // Grau máximo
    int grauMin;                                // Grau mínimo
    double grauMediano;                         // Grau mediano
    double grauMedio;                           // Grau médio
    int first_diameter = 0;                     // Diâmetro inicial, calculado pela BFS dupla
    pair<int,int> maior_dist = {-1,0};          // Vértice com maior distância a raiz da árvore e sua respectiva distância, para cálculo do diâmetro pela BFS dupla
    vector<vector<int>> adj;                    // Lista de adjacências
    vector<vector<bool>> mat;                   // Matriz de adjacências
    vector<int> grau;                           // Vetor com os graus de cada vértice
    vector<bool> vis;                           // Vetor de visitados
    vector<int> pai;                            // Vetor com o vértice pai de cada vértice
    vector<int> nivel;                          // Vetor com o nível de cada vértice
    vector<int> dist;                           // Vetor com o nível de cada vértice, para uso CompCon
    vector<pair<int,vector<int>>> compCon;      // Componentes conexas, com o tamanho e os vértices de cada uma
    void addEdge(int v1, int v2, int mode);     // Adiciona uma aresta entre os vértices v1 e v2 a estrutura de dados do grafo
    vector<int> bfs_CompCon(int s);             // BFS para achar as componentes conexas

public:
    Grafo(string FileName, int mode);           // Construtor da classe
    void bfs(int s, int print = 0);             // Método que realiza a BFS para um vértice s, pode ou não gerar um txt com as informações da BFS
    void dfs(int s, int print = 0);             // Método que realiza a DFS para um vértice s, pode ou não gerar um txt com as informações da DFS
    int distancia(int v, int u);                // Método que calcula a distância entre os vértices v e u
    int diametro();                             // Método que calcula o diâmetro do grafo. Para grafos grandes, pode ser muito custoso, por isso, recoendamos usar o método diametro_aprox
    int diametro_aprox();                       // Método que calcula o diâmetro do grafo de forma aproximada
    void geradortxt();                          // Método que gera um arquivo de saída com as informações gerais do grafo
    void printListAdj();                        // Método que printa a lista de adjacências
    void printMatrizAdj();                      // Método que printa a matriz de adjacências
    void printCompCon(int print);               // Método que printa o numero de componentes conexas e o tamanho de cada uma, pode ou não printar todos os vértices de cada componente


    // Métodos getters  
    int getV() const { return V; }                             // Retorna o número de vértices
    int getA() const { return A; }                             // Retorna o número de arestas
    int getGrauMax() const { return grauMax; }                 // Retorna o grau máximo
    int getGrauMin() const { return grauMin; }                 // Retorna o grau mínimo
    double getGrauMediano() const { return grauMediano; }      // Retorna o grau mediano
    double getGrauMedio() const { return grauMedio; }          // Retorna o grau médio
    size_t getAdjMemoryUsage() const;                          // Método que calcula a memória usada pela representação em lista
    size_t getMatMemoryUsage() const;                          // Método que calcula a memória usada pela representação em matriz
};


Grafo::Grafo(string FileName, int mode = 0) {
    this->mode = mode;
    std::ifstream arquivo(FileName);
    cout << "-> Pegando dados do txt: '" << FileName << "'" << endl;
    if(!arquivo.is_open()){
        std::cerr << "-> Não foi possível abrir o arquivo '" << FileName << "'\n";
    }
    else{
        int v1, v2;
        arquivo >> V;
        if (mode == 0) adj.resize(V);
        else mat.resize(V, vector<bool>(V, false));
        grau.resize(V,0);
        vis.resize(V,false);
        pai.resize(V,-2);
        nivel.resize(V,0);
        dist.resize(V,0);

        while(arquivo >> v1 >> v2){
            addEdge(v1, v2, mode);
            A++;
        }
        arquivo.close();
        cout << "-> Dados do txt '" << FileName << "' pegos com sucesso!" << endl;
    }

    // Cálculo dos graus
    vector<int> graus = grau;
    std::sort(graus.begin(), graus.end());
    grauMax = graus[V-1];
    grauMin = graus[0];
    if (V % 2 == 0) grauMediano = (graus[V/2] + graus[V/2 - 1])/2;
    else grauMediano = graus[V/2];
    grauMedio = 2*A/V;  

    // Cálculo das componentes conexas
    for (int i = 0; i < V; i++){
        if(vis[i] == false){
            vector<int> aux = bfs_CompCon(i);
            compCon.push_back({aux.size(), aux});

            // Cálculo do diâmetro
            // -> bfs_compcon acha o vertice com maior nivel para um vertice aleatorio. fazer bfs a partir desse vertice para achar o diametro  
            bfs(maior_dist.first+1,1);              
            // cout << maior_dist.first+1 << " " << maior_dist.second << endl;                     
            first_diameter = max(first_diameter, maior_dist.second);    
        }
    }
    std::sort(compCon.begin(),compCon.end(),std::greater< std::pair<int,std::vector<int>> >());
    
    // Gera arquivo de saída
    geradortxt();
}

void Grafo::addEdge(int v1, int v2, int mode) { // mode = 0 para representação em lista e mode = 1 para representação em matriz
    v1--; v2--;
    grau[v1]++; grau[v2]++;
    if (mode == 0){
        adj[v1].push_back(v2);
        adj[v2].push_back(v1); 
    }
    else{
        mat[v1][v2] = true;
        mat[v2][v1] = true;
    }
}

vector<int> Grafo::bfs_CompCon(int s) { // Retorna um vetor com os vértices pertencentes a uma componente conexa
    vector<int> componente;
    maior_dist = {-1,0};
    dist[s] = 0; 
    if (mode == 0){         // mode = 0 para representação em lista
        queue<int> q;
        vis[s] = true; componente.push_back(s);
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (int v : adj[u]) {
                if (!vis[v]) {
                    vis[v] = true; componente.push_back(v);
                    dist[v] = dist[u] + 1;  
                    if(dist[v] >= maior_dist.second) maior_dist = {v,dist[v]};
                    q.push(v);
                }
            }
        }
    }
    else{                   // mode = 1 para representação em matriz
        queue<int> q;
        vis[s] = true; componente.push_back(s);
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (int v = 0; v < V; ++v) {
                if (mat[u][v] && !vis[v]) {
                    vis[v] = true; componente.push_back(v);
                    dist[v] = dist[u] + 1;  
                    if(dist[v] >= maior_dist.second) maior_dist = {v,dist[v]};
                    q.push(v);
                }
            }
        }
    }
    return componente;
}

void Grafo::bfs(int s, int print){          // print = 0 para printar e 1 para não printar informações da arvore geradora da BFS
    s--;                                        // os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s
    pai.clear(); pai.resize(V,-2);
    nivel.clear(); nivel.resize(V,-1);
    nivel[s] = 0; pai[s] = -1;
    vector<bool> visitados(V, false);
    if (mode == 0){         // mode = 0 para representação em lista
        queue<int> q;
        visitados[s] = true; 
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (int v : adj[u]) {
                if (!visitados[v]) {
                    visitados[v] = true; 
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    if(nivel[v] >= maior_dist.second) maior_dist = {v,nivel[v]};
                    q.push(v);
                }
            }
        }
    }
    else{                   // mode = 1 para representação em matriz
        queue<int> q;
        visitados[s] = true;
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (int v = 0; v < V; ++v) {
                if (mat[u][v] && !visitados[v]) {
                    visitados[v] = true; 
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    if(dist[v] >= maior_dist.second) maior_dist = {v,dist[v]};
                    q.push(v);
                }
            }
        }
    }
    // cout no arquivo
    if (print == 0){
        std::ofstream outputFile("bfs_info.txt");
        if (outputFile.is_open()) {
            for (int i = 0; i < V; i++){
                if(visitados[i] == false) continue;
                else outputFile << "Vertice " << i+1 << ": pai = " << pai[i]+1 << ", nivel = " << nivel[i] << endl;
            }
            cout << "-> Informacoes da bfs salvas em 'bfs_info.txt'" << endl;
        } 
        else {
            std::cerr << "-> Nao foi possível criar o arquivo de saida" << "\n";
        }
    }
}

void Grafo::dfs(int s, int print){          // print = 0 para printar e 1 para não printar informações da arvore geradora da DFS
    s--;                                    // os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s
    vector<bool> visitados(V, false);
    pai.clear(); pai.resize(V,-2);
    nivel.clear(); nivel.resize(V,-1);

    stack<int> pilha;
    pilha.push(s);
    nivel[s] = 0; pai[s] = -1;

    if (mode == 0) { // representação em lista
        while (!pilha.empty()){
            int v = pilha.top();
            pilha.pop();
            if (visitados[v]) continue;
            visitados[v] = true;
            for (int u : adj[v]){
                if (visitados[u]) continue;     // pular se u já foi visitado
                pilha.push(u);
                pai[u] = v; nivel[u] = nivel[v] + 1;
            }
        }
    }

    else {      // representação em matriz
        while (!pilha.empty()){
            int v = pilha.top();
            pilha.pop();
            if (visitados[v]) continue;
            visitados[v] = true;
            for (int u = 0 ; u < V ; u++){
                if (visitados[u] || mat[v][u] == false) continue;     // pular se u já foi visitado ou não é vizinho
                pilha.push(u);
                pai[u] = v; nivel[u] = nivel[v] + 1;
            }
        }
    }

    // cout no arquivo
    if (print == 0){
        std::ofstream outputFile("dfs_info.txt");
        if (outputFile.is_open()) {
            for (int i = 0; i < V; i++){
                if(visitados[i] == false) continue;
                else outputFile << "Vertice " << i+1 << ": pai = " << pai[i]+1 << ", nivel = " << nivel[i] << endl;
            }
            cout << "-> Informacoes da dfs salvas em 'dfs_info.txt'" << endl;
        } 
        else {
            std::cerr << "-> Nao foi possível criar o arquivo de saida" << "\n";
        }
    }
}

int Grafo::distancia(int v, int u){         // v é o vértice inicial, e u é o vértice ao qual se quer chegar 
    bfs(v,1);                               // obs: os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s,
    return nivel[u-1];                      //      o de u a gente faz aqui
}


int Grafo::diametro(){
    // Calcula o diâmetro do grafo. Para grafos grandes, pode ser muito custoso, por isso, recoendamos usar o método diametro_aprox
    int max_diameter = first_diameter; // Diâmetro inicial, calculado pela BFS dupla
    for (int i = 0; i < V; ++i) {
        bfs(i+1,1);
        max_diameter = max(max_diameter, maior_dist.second);
    }
    return max_diameter;
}

int Grafo::diametro_aprox() {
    // Calcula o diâmetro do grafo de forma aproximada, usando a BFS dupla e em seguida fazendo uma amostragem de vértices aleatórios para melhorar a estimativa
    int max_diameter = first_diameter; // Diâmetro inicial, calculado pela BFS dupla
    set<int> st;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, V); // Gera um número aleatório entre 1 e V
    
    int samples = 0;
    int max_samples = 100;

    while (samples < max_samples) {
        int s;
        if (st.size() == V) break;
        do {
            s = dis(gen);                   // Gera um número aleatório entre 1 e V
        } while (st.find(s) != st.end());   // Enquanto o número gerado já estiver no set, gera outro
        st.insert(s);                       // Insere o número gerado no set
        bfs(s, 1);
        if (maior_dist.second > max_diameter) {
            max_diameter = maior_dist.second;
            // cout << "--> Novo diametro estimado: " << max_diameter << endl;
        }
        samples++;
    }

    // cout << "-> Diametro inicial: " << first_diameter << endl;
    // cout << "-> Diametro final estimado: " << max_diameter << endl;
    return max_diameter;
}

void Grafo::geradortxt(){  // Criar arquivo de saída com as informações
    std::ofstream outputFile("grafo_info.txt");
    if (outputFile.is_open()) {
        outputFile << "Vértices: " << V << endl; 
        outputFile << "Aresta: " << A << endl;
        outputFile << "Grau máximo: " << grauMax << endl; 
        outputFile << "Grau mínimo: " << grauMin << endl;    
        outputFile << "Grau mediano: " << grauMediano << endl;
        outputFile << "Grau médio: " << grauMedio << endl;
        outputFile << endl;
        for (int i = 0; i < compCon.size(); i++){
            outputFile << "Componente conexa " << i+1 << ": " << compCon[i].first << " vértices" << endl;
            for (int j = 0; j < compCon[i].first; j++){
                outputFile << compCon[i].second[j]+1 << " ";
            }
            outputFile << endl;   
        }
        cout << "-> Informacoes salvas em 'grafo_info.txt'" << endl;
    } 
    else {
        std::cerr << "-> Nao foi possível criar o arquivo de saida" << "\n";
    }
}

void Grafo::printListAdj() {  // Printa a lista de adjacências
    for (int i = 0; i < V; ++i) {
        cout << "Vertice " << i+1 << ":";
        for (int vizinho : adj[i]) {
            cout << " " << vizinho+1;
        }
        cout << endl;
    }
}

void Grafo::printMatrizAdj() {  // Printa a matriz de adjacências
    cout << "Matriz de Adjacencias:" << endl;
    for (int i = 0; i < V; ++i) {
        for (int j = 0; j < V; ++j) {
            cout << mat[i][j] << " ";
        }
        cout << endl;
    }
}

void Grafo::printCompCon(int print = 1){  // print = 0 para printar todos os vertices pertencentes a cada componente e 1 para não printar
    for (int i = 0; i < compCon.size(); i++){
        cout << "Componente conexa " << i+1 << ": " << compCon[i].first << " vertices" << endl;
        if(print == 0){
            for (int j = 0; j < compCon[i].first; j++){
                cout << compCon[i].second[j]+1 << " ";
            }
            cout << endl;
        }
    }
}

size_t Grafo::getAdjMemoryUsage() const {
    size_t totalSize = sizeof(adj);
    for (const auto& vec : adj) {
        totalSize += sizeof(vec) + (vec.capacity() * sizeof(int));
    }
    return totalSize;
}

size_t Grafo::getMatMemoryUsage() const {
    size_t totalSize = sizeof(mat);
    for (const auto& vec : mat) {
        totalSize += sizeof(vec) + (vec.capacity() * sizeof(bool));
    }
    return totalSize;
}

#endif 
