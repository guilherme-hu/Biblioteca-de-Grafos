#include "grafo.h"


class GrafoComPeso : public Grafo {
protected:
    vector<vector<pair<double, int>>> adjPeso; // Lista de adjacências com pesos (peso, vértice)
    vector<vector<double>> matPeso;            // Matriz de adjacências com pesos
    bool negativo = false;                     // Variável para verificar se existe aresta com peso negativo
    vector<int> bfs_CompCon(int s);            // Método que realiza a busca em largura para componentes conexas

public:
    GrafoComPeso(string FileName, int mode);                // Construtor da classe
    void addEdge(int v1, int v2, double peso, int mode);    // Método que adiciona aresta com peso
    void Dijsktra(int s);                                   // Método que realiza o algoritmo de Dijsktra
    void Prim(int s);                                       // Método que realiza o algoritmo de Prim, para montar uma MST do grafo
    void printListAdj() const;                              // Método que imprime a lista de adjacências
    void printMatrizAdj() const;                            // Método que imprime a matriz de adjacências
    size_t getAdjMemoryUsage() const;                       // Método que obtém memória (calculada) usada pela representação em lista
    size_t getMatMemoryUsage() const;                       // Método que obtém memória (calculada) usada pela representação em matriz
}; 

// Constructor
GrafoComPeso::GrafoComPeso(string FileName, int mode) {
    // Inicializar membros da classe pai manualmente
    this->mode = mode;
    this->V = 0;
    this->A = 0;
    this->grau.clear();
    this->vis.clear();
    this->vis.resize(V, false);
    this->pai.clear();
    this->nivel.clear();
    this->dist.clear();

    std::ifstream arquivo(FileName);
    cout << "-> Pegando dados do txt de grafo com peso: '" << FileName << "'" << endl;
    if(!arquivo.is_open()){
        std::cerr << "-> Não foi possível abrir o arquivo '" << FileName << "'\n";
    }
    else{
        int v1, v2;
        double peso;
        arquivo >> V;
        if (mode == 0) adjPeso.resize(V);
        else matPeso.resize(V, vector<double>(V, INF));
        grau.resize(V,0);
        vis.resize(V,false);
        pai.resize(V,-2);
        nivel.resize(V,0);
        dist.resize(V,0);

        while(arquivo >> v1 >> v2 >> peso){
            if (peso < 0) negativo = true;
            addEdge(v1, v2, peso, mode);
            A++;
        }
        arquivo.close();
        cout << "-> Dados do txt de grafo com pesos '" << FileName << "' pegos com sucesso!" << endl;
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
        if(vis[i] == 0){
            vector<int> aux = bfs_CompCon(i);
            compCon.push_back({aux.size(), aux});

            // // Cálculo do diâmetro
            // // -> bfs_compcon acha o vertice com maior nivel para um vertice aleatorio. fazer bfs a partir desse vertice para achar o diametro aproximado
            // bfs(maior_dist.first+1,1);              
            // // cout << maior_dist.first+1 << " " << maior_dist.second << endl;                     
            // first_diameter = max(first_diameter, maior_dist.second);    
        }
    }
    std::sort(compCon.begin(),compCon.end(),std::greater< std::pair<int,std::vector<int>> >());
}

// Método que adiciona aresta com peso
void GrafoComPeso::addEdge(int v1, int v2, double peso, int mode) {
    v1--; v2--;
    grau[v1]++; grau[v2]++;
    if (mode == 0){
        adjPeso[v1].push_back({peso, v2});
        adjPeso[v2].push_back({peso, v1}); 
    }
    else{
        matPeso[v1][v2] = peso;
        matPeso[v2][v1] = peso;
    }
}


// Método que realiza a busca em largura para componentes conexas
vector<int> GrafoComPeso::bfs_CompCon(int s) { // Retorna um vetor com os vértices pertencentes a uma componente conexa
    vector<int> componente;
    maior_dist = make_pair(-1, 0);
    this->dist[s] = 0;
    if (mode == 0) { // mode = 0 para representação em lista
        queue<int> q;
        this->vis[s] = true;
        componente.push_back(s);
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (size_t i = 0; i < adjPeso[u].size(); ++i) {
                int v = adjPeso[u][i].second;
                if (!vis[v]) {
                    this->vis[v] = true;
                    componente.push_back(v);
                    this->dist[v] = this->dist[u] + 1;
                    if (this->dist[v] >= maior_dist.second) maior_dist = std::make_pair(v, this->dist[v]);
                    q.push(v);
                }
            }
        }
    } else { // mode = 1 para representação em matriz
        queue<int> q;
        vis[s] = true;
        componente.push_back(s);
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (int v = 0; v < V; ++v) {
                if (matPeso[u][v] != INF && !vis[v]) {
                    vis[v] = true;
                    componente.push_back(v);
                    dist[v] = dist[u] + 1;
                    if (dist[v] >= maior_dist.second) maior_dist = {v, dist[v]};
                    q.push(v);
                }
            }
        }
    }
    return componente;
}

// Método que realiza o algoritmo de Dijsktra
void GrafoComPeso::Dijsktra(int s){
    if (negativo) {
        cout << "O grafo possui arestas com peso negativo, a biblioteca ainda nao implementa caminhos minimos com pesos negativos!" << endl;
        return;
    }
    return;
}

// Método que realiza o algoritmo de Prim
void GrafoComPeso::Prim(int s){
    return;
}

// Método que imprime a lista de adjacências
void GrafoComPeso::printListAdj() const {
    for (int i = 0; i < adjPeso.size(); ++i) {
        cout << "Vertice " << i + 1 << ": [";
        for (size_t j = 0; j < adjPeso[i].size(); ++j) {
            cout << "[" << adjPeso[i][j].second + 1 << ", " << adjPeso[i][j].first << "]";
            if (j < adjPeso[i].size() - 1) {
                cout << ", ";
            }
        }
        cout << "]" << endl;
    }
}

// Método que imprime a matriz de adjacências
void GrafoComPeso::printMatrizAdj() const {
    for (const auto& row : matPeso) {
        for (const auto& weight : row) {
            if (weight == INF) {
                cout << "INF ";
            } else {
                cout << weight << " ";
            }
        }
        cout << endl;
    }
}

// Método que obtém memória (calculada) usada pela representação em lista
size_t GrafoComPeso::getAdjMemoryUsage() const {
    size_t memoryUsage = 0;
    for (const auto& vec : adjPeso) {
        memoryUsage += sizeof(vec) + (vec.capacity() * sizeof(pair<double, int>));
    }
    return memoryUsage;
}

// Método que obtém memória (calculada) usada pela representação em matriz
size_t GrafoComPeso::getMatMemoryUsage() const {
    size_t memoryUsage = 0;
    for (const auto& vec : matPeso) {
        memoryUsage += sizeof(vec) + (vec.capacity() * sizeof(double));
    }
    return memoryUsage;
}
