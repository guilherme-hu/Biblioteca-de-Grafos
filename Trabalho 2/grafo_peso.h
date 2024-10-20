#include "grafo.h"


class GrafoComPeso : public Grafo {
protected:
    vector<vector<pair<float, int>>> adjPeso;               // Lista de adjacências com pesos (peso, vértice)
    vector<vector<float>> matPeso;                          // Matriz de adjacências com pesos
    bool negativo = false;                                  // Variável para verificar se existe aresta com peso negativo
    void addEdge(int v1, int v2, float peso, int mode);     // Método que adiciona aresta com peso
    vector<int> bfs_CompCon(int s);                         // Método que realiza a busca em largura para componentes conexas
    vector<float> distPeso;                                 // Variável para registar as distâncias mínimas em um grafo com peso 

public:
    GrafoComPeso(string FileName, int mode);                              // Construtor da classe
    void bfs(int s, int print = 0);                                       // Método que realiza a BFS para um vértice s
    void dfs(int s, int print = 0);                                       // Método que realiza a DFS para um vértice s
    int diametro();                                                       // Método que calcula o diâmetro do grafo 
    int diametro_aprox();                                                 // Método que calcula o diâmetro do grafo de forma aproximada 
    void Dijkstra(int s, int heap = 1);                                   // Método que realiza o algoritmo de Dijsktra. Parâmetro heap = 0 para sem heap e 1 para com heap
    float distancia(int v, int u, int print_caminho = 0, int heap = 1);   // Método que calcula a distância entre os vértices v e u
    void printListAdj() const;                                            // Método que imprime a lista de adjacências
    void printMatrizAdj() const;                                          // Método que imprime a matriz de adjacências
    size_t getAdjMemoryUsage() const;                                     // Método que obtém memória (calculada) usada pela representação em lista
    size_t getMatMemoryUsage() const;                                     // Método que obtém memória (calculada) usada pela representação em matriz
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
        float peso;
        arquivo >> V;
        if (mode == 0) adjPeso.resize(V);
        else matPeso.resize(V, vector<float>(V, INF));
        grau.resize(V,0);
        vis.resize(V,false);
        pai.resize(V,-2);
        nivel.resize(V,0);
        dist.resize(V,0);
        distPeso.resize(V,INF); 

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
        }
    }
    std::sort(compCon.begin(),compCon.end(),std::greater< std::pair<int,std::vector<int>> >());

}

// Método que adiciona aresta com peso
void GrafoComPeso::addEdge(int v1, int v2, float peso, int mode) {
    v1--; v2--;
    grau[v1]++; grau[v2]++;
    if (mode == 0){ // lista adj
        adjPeso[v1].push_back({peso, v2});
        adjPeso[v2].push_back({peso, v1}); 
    }
    else{ // matriz
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
    } 
    else { // mode = 1 para representação em matriz
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

// Método que realiza a busca em largura para um vértice s
void GrafoComPeso::bfs(int s, int print){
    s--; // os vértices são indexados de 1 a V, na bfs ja subtrai -1 de s
    pai.clear(); pai.resize(V, -2);
    nivel.clear(); nivel.resize(V, -1);
    nivel[s] = 0; pai[s] = -1;
    vector<bool> visitados(V, false);
    if (mode == 0){ // mode = 0 para representação em lista
        queue<int> q;
        visitados[s] = true;
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (const auto& p : adjPeso[u]) {
                int v = p.second;
                if (!visitados[v]) {
                    visitados[v] = true;
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    if(nivel[v] >= maior_dist.second) maior_dist = {v, nivel[v]};
                    q.push(v);
                }
            }
        }
    }
    else { // mode = 1 para representação em matriz
        queue<int> q;
        visitados[s] = true;
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (int v = 0; v < V; ++v) {
                if (matPeso[u][v] != INF && !visitados[v]) {
                    visitados[v] = true;
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    if(nivel[v] >= maior_dist.second) maior_dist = {v, nivel[v]};
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

// Método que realiza a busca em profundidade para um vértice s
void GrafoComPeso::dfs(int s, int print){
    s--; // os vértices são indexados de 1 a V, na dfs ja subtrai -1 de s
    vector<bool> visitados(V, false);
    pai.clear(); pai.resize(V, -2);
    nivel.clear(); nivel.resize(V, -1);

    stack<int> pilha;
    pilha.push(s);
    nivel[s] = 0; pai[s] = -1;

    if (mode == 0) { // representação em lista
        while (!pilha.empty()){
            int v = pilha.top();
            pilha.pop();
            if (visitados[v]) continue;
            visitados[v] = true;
            for (const auto& p : adjPeso[v]){
                int u = p.second;
                if (visitados[u]) continue; // pular se u já foi visitado
                pilha.push(u);
                pai[u] = v; nivel[u] = nivel[v] + 1;
            }
        }
    }
    else { // representação em matriz
        while (!pilha.empty()){
            int v = pilha.top();
            pilha.pop();
            if (visitados[v]) continue;
            visitados[v] = true;
            for (int u = 0; u < V; ++u){
                if (visitados[u] || matPeso[v][u] == INF) continue; // pular se u já foi visitado ou não é vizinho
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
int GrafoComPeso::diametro() {  // Método que calcula o diâmetro do grafo 
    cout << "Nao implementado" << endl;
    return 0;
}       

int GrafoComPeso::diametro_aprox(){ // Método que calcula o diâmetro do grafo de forma aproximada
    cout << "Nao implementado" << endl;
    return 0;
}

// Método que realiza o algoritmo de Dijsktra
void GrafoComPeso::Dijkstra(int s, int heap){

    if (negativo) {
        cout << "O grafo possui arestas com peso negativo, a biblioteca ainda nao implementa caminhos minimos com pesos negativos!" << endl;
        return;
    }

    // Reiniciando os vetores da BFS_CompCon 
    this->vis.assign(V, false); 
    this->dist.assign(V, INF); 

    // Chamada do método bfs_CompCon
    vector<int> componente = bfs_CompCon(s); 
    int componente_size = componente.size(); // encontrar o tamanho da componente conexa
    // cout << "O tamanho da componente: " << componente_size << endl;

    if (heap == 0) { // versão sem heap
        vector<bool> visited(V, false);
        int visitados = 0; // conta quantos já foram visitados
        distPeso[s] = 0; pai[s]=-3; //visited[s] = true; visitados++; // s foi visitado

        if (mode==0) { // list adj
            while (visitados != componente_size){ // fazemos isso para todos os vértices da componente
                int u = -3;
                for (int i = 0 ; i < V ; i++){ // pegar o vertice u não marcado e com dist[u] mínima
                    if (visited[i]) continue;
                    if (u == -3) u = i;
                    if (distPeso[u]>distPeso[i]) u = i;
                    
                }

                visited[u] = true; // visitar u
                visitados++;

                for (pair<float,int> p : adjPeso[u]){ // para cada vizinho v de u
                    int v = p.second;
                    float peso_uv = p.first;

                    if (distPeso[v] > distPeso[u] + peso_uv){ // se a gnt achou um dist[v] menor, muda o dist[v]
                        
                        distPeso[v] = distPeso[u] + peso_uv;
                        pai[v] = u;
                    }
                }
            }
        }

        else { // matriz
            while (visitados != componente_size){ // fazemos isso para todos os vértices da componente
                int u = -3;
                for (int i = 0 ; i < V ; i++){ // pegar o vertice u não marcado e com dist[u] mínima
                    if (visited[i]) continue;
                    if (u == -3) u = i;
                    if (distPeso[u]>distPeso[i]) u = i;
                    
                }

                visited[u] = true; // visitar u
                visitados++;

                for (int v = 0 ; v < V ; v++){ // para cada vizinho v de u
                    if (matPeso[u][v] != INF){
                        float peso_uv = matPeso[u][v];

                        if (distPeso[v] > distPeso[u] + peso_uv){ // se a gnt achou um dist[v] menor, muda o dist[v]
                            distPeso[v] = distPeso[u] + peso_uv;
                            pai[v] = u;
                        }
                    }
                }
            }
        }

    }

    else { // versão com heap
        // Implementar com heap -> usamos priority queue, equivalente a heap
        vector<bool> visited(V, false);
        priority_queue<pair<float, pair<int,int>>, vector<pair<float, pair<int,int>>>, greater<pair<float, pair<int,int>>>> h;
            // esse heap guarda um float com a distância achada até o vértice, e um pair com o vértice e seu pai 
        distPeso[s] = 0;
        h.push({distPeso[s],{s,-3}});

        if (mode==0) { // list adj
            while (!h.empty()){ // fazemos isso para todos os vértices da componente
                

                int u,pai_u;
                float dist_u;

                pair<float,pair<int,int>> p = h.top();
                h.pop();
                u = p.second.first;
                if (visited[u]) continue;

                dist_u = p.first;
                pai_u = p.second.second;

                pai[u] = pai_u;
                distPeso[u] = dist_u;
                visited[u] = true; // visitar u

                for (pair<float,int> p : adjPeso[u]){ // para cada vizinho v de u
                    int v = p.second;
                    float peso_uv = p.first;
                    h.push({distPeso[u]+peso_uv,{v,u}});
                }
            }
        }

        else { // matriz
            while (!h.empty()){ // fazemos isso para todos os vértices da componente
                
                int u,pai_u;
                float dist_u;
                
                pair<float,pair<int,int>> p = h.top();
                h.pop();
                u = p.second.first;
                if (visited[u]) continue;

                dist_u = p.first;
                pai_u = p.second.second;

                pai[u] = pai_u;
                distPeso[u] = dist_u;
                visited[u] = true; // visitar u


                for (int v = 0 ; v < V ; v++){ // para cada vizinho v de u
                    if (matPeso[u][v] != INF){
                        float peso_uv = matPeso[u][v];
                        h.push({distPeso[u]+peso_uv,{v,u}});
                    }
                }
            }
        }

    }
    
    return;
}


float GrafoComPeso::distancia(int v, int u, int print_caminho, int heap){
    v--; u--;

    if (pai[v] != -3) Dijkstra(v,heap);

    // cout << "Pais: ";
    // for (int i : pai){
    //     cout << i+1 << ", ";
    // }
    // cout << endl;
    
    vector<int> caminho;
    if (print_caminho == 1){
        if (distPeso[u] != INF){
            int i = u;
            while (pai[i] != -3){
                caminho.insert(caminho.begin(),i);
                i = pai[i]; 
            }
            caminho.insert(caminho.begin(),i);

            cout << "Caminho de " << v+1 << " para " << u+1 << ": ";
            for (int i : caminho){
                cout << i+1 << " ";
            }
            cout << "\n";
        }
        else cout << "Caminho inexistente entre " << v << " e " << u << endl;

    }

    if (distPeso[u] ==  INF) return -3;
    return distPeso[u];
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
        memoryUsage += sizeof(vec) + (vec.capacity() * sizeof(pair<float, int>));
    }
    return memoryUsage;
}

// Método que obtém memória (calculada) usada pela representação em matriz
size_t GrafoComPeso::getMatMemoryUsage() const {
    size_t memoryUsage = 0;
    for (const auto& vec : matPeso) {
        memoryUsage += sizeof(vec) + (vec.capacity() * sizeof(float));
    }
    return memoryUsage;
}
