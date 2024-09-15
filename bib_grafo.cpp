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


class Grafo {
private:  
    int V;                                  // Número de vértices
    int A = 0;                              // Número de arestas
    int mode;                               // 0 para lista e 1 para matriz
    int grauMax;
    int grauMin;
    double grauMediano;
    double grauMedio;
    vector<vector<int>> adj;                // Lista de adjacências
    vector<vector<int>> mat;                // Matriz de adjacências
    vector<int> grau;                       // Vetor com os graus de cada vértice
    vector<bool> vis;                       // Vetor de visitados
    vector<int> pai;                        // Vetor com o vértice pai de cada vértice
    vector<int> nivel;                     // Vetor com o nível de cada vértice
    vector<pair<int,vector<int>>> compCon;  // Componentes conexas, com o tamanho e os vértices de cada uma

public:
    Grafo(string FileName, int mode);
    void addEdge(int v1, int v2, int mode);
    void printListAdj();
    void printMatrizAdj();
    vector<int> bfs_CompCon(int s);
    void bfs(int s, int print);
    void dfs(int s, int print);
    int distancia(int v, int u);
    int diametro();


    // Métodos getters
    int getV() const { return V; }
    int getA() const { return A; }
    int getGrauMax() const { return grauMax; }
    int getGrauMin() const { return grauMin; }
    double getGrauMediano() const { return grauMediano; }
    double getGrauMedio() const { return grauMedio; }
    vector<pair<int,vector<int>>> getCompCon() const { return compCon; }
};

Grafo::Grafo(string FileName, int mode = 0) {
    this->mode = mode;
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
        grau.resize(V,0);
        vis.resize(V,false);
        pai.resize(V,-2);
        nivel.resize(V,0);

        while(arquivo >> v1 >> v2){
            addEdge(v1, v2, mode);
            A++;
        }
        arquivo.close();
        cout << "Dados do txt pegos com sucesso!" << endl;
    }

    // Cálculo dos graus
    vector<int> graus = grau;
    sort(graus.begin(), graus.end());
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
        }
    }
    sort(compCon.begin(),compCon.end(),greater< pair<int,vector<int>> >());

}

void Grafo::addEdge(int v1, int v2, int mode) { // mode = 0 para representação em lista e mode = 1 para representação em matriz
    v1--; v2--;
    grau[v1]++; grau[v2]++;
    if (mode == 0){
        adj[v1].push_back(v2);
        adj[v2].push_back(v1); 
    }
    else{
        mat[v1][v2] = 1;
        mat[v2][v1] = 1;
    }
}

vector<int> Grafo::bfs_CompCon(int s) {
    vector<int> componente;
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
                    q.push(v);
                }
            }
        }
    }
    return componente;
}

void Grafo::bfs(int s, int print = 0){      // print = 0 para printar e 1 para não printar
    s--;                                    // os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s
    vector<int> visitados(V, false);
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
            cout << "Informacoes salvas em bfs_info.txt" << endl;
        } 
        else {
            std::cerr << "Nao foi possível criar o arquivo de saida" << "\n";
        }
    }
}

void Grafo::dfs(int s, int print = 0){
    s--;                                    // os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s
    vector<int> visitados(V, false);

    // pai e nivel
    stack<int> pilha;
    
    // cout no arquivo
    if (print == 0){
        std::ofstream outputFile("bfs_info.txt");
        if (outputFile.is_open()) {
            for (int i = 0; i < V; i++){
                if(visitados[i] == false) continue;
                else outputFile << "Vertice " << i+1 << ": pai = " << pai[i]+1 << ", nivel = " << nivel[i] << endl;
            }
            cout << "Informacoes salvas em bfs_info.txt" << endl;
        } 
        else {
            std::cerr << "Nao foi possível criar o arquivo de saida" << "\n";
        }
    }
}

int Grafo::distancia(int v, int u){         // v é o vértice inicial, e u é o vértice ao qual se quer chegar 
    pai.clear(); pai.resize(V,-2);
    nivel.clear(); nivel.resize(V,0);
    bfs(v,1);                               // obs: os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s,
    return nivel[u-1];                      //      o de u a gente faz aqui
}

int Grafo::diametro(){
    return 0;
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
    string FileName = "grafo_teste.txt";
    
    // cin >> mode;     // 0 para lista e 1 para matriz
    int mode = 0;

    clock_t start = clock();

    Grafo g(FileName);
    // if(mode == 0) g.printListAdj(); 
    // else g.printMatrizAdj();
    
    cout << "Vertices: " << g.getV() << endl; 
    cout << "Aresta: " << g.getA() << endl;
    cout << "Grau maximo: " << g.getGrauMax() << endl; 
    cout << "Grau minimo: " << g.getGrauMin() << endl;    
    cout << "Grau mediano: " << g.getGrauMediano() << endl;
    cout << "Grau medio: " << g.getGrauMedio() << endl;

    for (int i = 0; i < g.getCompCon().size(); i++){
        cout << "Componente conexa " << i+1 << ": " << g.getCompCon()[i].first << " vertices" << endl;
        // for (int j = 0; j < g.getCompCon()[i].first; j++){
        //     cout << g.getCompCon()[i].second[j]+1 << " ";
        // }
    }

    clock_t end = clock();
    cout << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;

    g.bfs(17);
    cout << "Distancia: " << g.distancia(7, 14) << endl;

    // Criar arquivo de saída com as informações
    std::ofstream outputFile("grafo_info.txt");
    if (outputFile.is_open()) {
        outputFile << "Vertices: " << g.getV() << endl; 
        outputFile << "Aresta: " << g.getA() << endl;
        outputFile << "Grau maximo: " << g.getGrauMax() << endl; 
        outputFile << "Grau minimo: " << g.getGrauMin() << endl;    
        outputFile << "Grau mediano: " << g.getGrauMediano() << endl;
        outputFile << "Grau medio: " << g.getGrauMedio() << endl;
        outputFile << endl;
        for (int i = 0; i < g.getCompCon().size(); i++){
            outputFile << "Componente conexa " << i+1 << ": " << g.getCompCon()[i].first << " vertices" << endl;
            for (int j = 0; j < g.getCompCon()[i].first; j++){
                outputFile << g.getCompCon()[i].second[j]+1 << " ";
            }
            outputFile << endl;   
        }
        cout << "Informacoes salvas em grafo_info.txt" << endl;
    } 
    else {
        std::cerr << "Nao foi possível criar o arquivo de saida" << "\n";
    }
    
    return 0;

}

// g++ bib_grafo.cpp -o bib_grafo -O2
// ./bib_grafo
