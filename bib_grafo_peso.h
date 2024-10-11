#include "bib_grafo.h"


class GrafoComPeso : public Grafo {
protected:
    vector<vector<pair<double, int>>> adjPeso; // Lista de adjacências com pesos (peso, vértice)
    vector<vector<double>> matPeso;            // Matriz de adjacências com pesos
    bool negativo = false;                     // Variável para verificar se existe aresta com peso negativo

public:
    GrafoComPeso(string FileName, int mode);                // Construtor da classe
    void addEdge(int v1, int v2, double peso, int mode);    // Método que adiciona aresta com peso
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
