#include "bib_grafo.h"


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
    
    geradortxt();
}

// Grafo::~Grafo() {
//     for (int i = 0; i < V; ++i) {
//         free(mat[i]);
//     }
//     free(mat);
// }


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

vector<int> Grafo::bfs_CompCon(int s) {
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

void Grafo::bfs(int s, int print){      // print = 0 para printar e 1 para não printar
    s--;                                    // os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s
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

void Grafo::dfs(int s, int print){      // print = 0 para printar e 1 para não printar
    s--;                                    // os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s
    vector<bool> visitados(V, false);
    pai.clear(); pai.resize(V,-2);
    nivel.clear(); nivel.resize(V,-1);

    stack<int> pilha;
    pilha.push(s);
    nivel[s] = 0; pai[s] = -1;

    if (mode == 0) {                     // representação em lista
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

    else {                     // representação em matriz
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

#include <random>

int Grafo::diametro(){
    int max_diameter = first_diameter; // Diâmetro inicial
    // int num_samples = 100; // Número de amostras para estimar o diâmetro

    // // Inicializa o gerador de números aleatórios
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<> dis(1, V); // Gera um número aleatório entre 1 e V
    if (V <=  50000){
      for (int i = 0; i < V; ++i) {
          // int s = dis(gen); // Gera um número aleatório entre 0 e V-1
          bfs(i+1,1);
          max_diameter = max(max_diameter, maior_dist.second);
      }
      cout << "-> Diâmetro inicial: " << first_diameter << endl;
      cout << "-> Diâmetro estimado: " << max_diameter << endl;
      return max_diameter;
    }
    else return first_diameter;
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



// g++ bib_grafo.cpp -o bib_grafo -O2
// ./bib_grafo
