#include "grafo.h"


class GrafoComPeso : public Grafo {
protected:
    vector<vector<pair<float, int>>> adjPeso;                        // Lista de adjacências com pesos -> cada vizinho de um vértice é salvo no formato (peso, vértice)
    vector<vector<float>> matPeso;                                   // Matriz de adjacências com pesos
    bool negativo = false;                                           // Variável que indica se existe aresta com peso negativo no grafo recebido
    void addEdge(int v1, int v2, float peso, int mode);              // Método que adiciona aresta com peso à estrutra de representação do grafo com peso escolhida -> mode = 0 para lista e 1 para matriz
    vector<int> bfs_CompCon(int s);                                  // Método que realiza a busca em largura para contabilizar as componentes conexas existentes no grafo
    vector<float> distPeso;                                          // Vetor que contém as distâncias mínimas de um vértice escolhido com raiz (no Dijkstra) para todos os outros vértices do grafo

    //                        cap  fluxo  destino
    vector<vector< pair< pair<int, int> , int> >> original;         // Lista de adjacências com capacidades e fluxos para grafos direcionados
                                                                    // cap = capacidade da aresta; fluxo = fluxo da aresta
    //                        o/r  f/r    destino
    vector<vector< pair< pair<int, int> , int> >> residual;         // Lista de adjacências para o grafo residual, usado no Ford-Fulkerson
                                                                    // o/r = tipo da aresta: origem/reversa -> 1 se a aresta é de origem e 0 se é reversa
                                                                    // f/r = valor do fluxo/resíduo -> se a aresta é de origem o valor é do fluxo na aresta, se for reversa é o valor do resíduo
    vector<pair<int, int>> pai_duplo;                               // Vetor que guarda o pai de cada vértice no caminho aumentante encontrado pelo Ford-Fulkerson
                                                                    // Primeiro int é o pai; segundo int do pai é o indice na lista de adj; e o terceiro int é o tipo da aresta (origem/reversa)
    bool bfs_FF(int s, int t);                                      // Método que realiza a BFS para o algoritmo de Ford-Fulkerson
    bool dfs_FF(int s, int t);                                      // Método que realiza a DFS para o algoritmo de Ford-Fulkerson
    int gargalo(int s, int t);                                      // Método que calcula o gargalo do caminho encontrado pelo Ford-Fulkerson
    void atualizar_grafo_residual(int u, int v, int gargalo);       // Método que atualiza o grafo residual após encontrar um caminho aumentante no Ford-Fulkerson
    void atualizar_grafo_original();                                // Método que atualiza o grafo original ao fim do Ford-Fulkerson

public:
    GrafoComPeso(string FileName, int mode, int direcionado);               // Construtor da subclasse

    void bfs(int s, int print = 0);                                         // Método que realiza a BFS para um vértice s -> para o grafo com peso, ignora os pesos e realiza a BFS normalmente
    void dfs(int s, int print = 0);                                         // Método que realiza a DFS para um vértice s -> para o grafo com peso, ignora os pesos e realiza a DFS normalmente
    int diametro();                                                         // Método que calcula o diâmetro do grafo -> não implementado para grafos com pesos por estar fora do escopo do trabalho
    int diametro_aprox();                                                   // Método que calcula o diâmetro do grafo de forma aproximada -> não implementado para grafos com pesos por estar fora do escopo do trabalho
    void Dijkstra(int s, int heap = 1);                                     // Método que realiza o algoritmo de Dijsktra. O parâmetro heap define se o algoritmo será executado com uso de heap (1) ou sem uso de heap (0)
    float distancia(int v, int u, int print_caminho = 0, int heap = 1);     // Método que calcula a distância mínima entre os vértices v e u -> válido apenas para grafos sem pesos negativos. 
                                                                            // -/-> O parâmetro heap define se o Dijkstra será executado com heap (1) ou sem heap (0), e o parâmetro print_caminho define se o caminho mínimo será impresso junto da distância mínima
    void printListAdj() const;                                              // Método que imprime a lista de adjacências com pesos
    void printMatrizAdj() const;                                            // Método que imprime a matriz de adjacências com pesos
    size_t getAdjMemoryUsage() const;                                       // Método que obtém memória (calculada) usada pela representação em lista
    size_t getMatMemoryUsage() const;                                       // Método que obtém memória (calculada) usada pela representação em matriz

    
    pair <int,vector<vector<pair<pair<int, int>,int>>>> ford_fulkerson(int s, int t, int print = 0);  // Método que realiza o algoritmo de Ford-Fulkerson 
                                                                                                            // Retorna pair de fluxo máximo e lista de adjacência do grafo original
                                                                                                            // Quando print = 1 informações são printadas no txt


}; 

// Constructor da subclasse GrafoComPeso
// mode = 0 para lista de adjacência e 1 para matriz de adjacência
// direcionado = 0 para grafo não direcionado e 1 para grafo direcionado
GrafoComPeso::GrafoComPeso(string FileName, int mode, int direcionado = 0) {
    
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

    // Leitura do arquivo de entrada
    std::ifstream arquivo(FileName);
    cout << "-> Pegando dados do txt de grafo com peso: '" << FileName << "'" << endl;
    if(!arquivo.is_open()){
        std::cerr << "-> Não foi possível abrir o arquivo '" << FileName << "'\n";
    }
    else{
        int v1, v2;
        float peso;
        int cap;
        arquivo >> V;

        // Inicializar os vetores de acordo com o modo de representação escolhido
        // Se o grafo for direcionado, a lista de adjacência será a usada, já que a implementação pela matriz é opcional
        if (direcionado == 0){
            if (mode == 0) adjPeso.resize(V);
            else matPeso.resize(V, vector<float>(V, INF));
        }
        else {
            adjPeso.resize(V);
            original.resize(V);
            residual.resize(V);
            pai_duplo.resize(V, {-2,-2});
        }

        // Inicializar os vetores de grau, visitados, pai, nível, distância e distância mínima
        grau.resize(V,0);
        vis.resize(V,false);
        pai.resize(V,-2);
        nivel.resize(V,0);
        dist.resize(V,0);
        distPeso.resize(V,INF); 

        if (direcionado == 0){
            while(arquivo >> v1 >> v2 >> peso){
                if (peso < 0) negativo = true;
                addEdge(v1, v2, peso, mode);
                A++;
            }
        }
        else{ // grafo direcionado
            if (mode == 1) cout << "Modo de representação escolhido não é válido para grafos direcionados. Usando lista de adjacência." << endl;
            while(arquivo >> v1 >> v2 >> cap){
                v1--; v2--;
                grau[v1]++;
                if (cap < 0) negativo = true; // capacidade negativa
                adjPeso[v1].push_back({cap, v2});
                original[v1].push_back({{cap, 0}, v2});
                residual[v1].push_back({{1, cap}, v2});
                residual[v2].push_back({{0, 0}, v1});
                A++;
            }
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


// Método que adiciona aresta com peso ao modo de representação escolhido do grafo com peso
// mode = 0 para lista de adjacência e 1 para matriz de adjacência
void GrafoComPeso::addEdge(int v1, int v2, float peso, int mode) {
    v1--; v2--;
    grau[v1]++; grau[v2]++;
    if (mode == 0){ // lista de adjacência
        adjPeso[v1].push_back({peso, v2});
        adjPeso[v2].push_back({peso, v1}); 
    }
    else{ // matriz
        matPeso[v1][v2] = peso;
        matPeso[v2][v1] = peso;
    }
}


// Método que realiza a busca em largura para contabilizar as componentes conexas existentes no grafo
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

// Método que realiza a busca em largura para um vértice s -> para o grafo com peso, ignora os pesos e realiza a BFS normalmente
// print = 0 para printar e 1 para não printar informações da arvore geradora da BFS
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

// Método que realiza a busca em profundidade para um vértice s -> para o grafo com peso, ignora os pesos e realiza a DFS normalmente
// print = 0 para printar e 1 para não printar informações da arvore geradora da DFS
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

// Método que calcula o diâmetro do grafo -> não implementado para grafos com pesos por estar fora do escopo do trabalho
int GrafoComPeso::diametro() {  
    cout << "Nao implementado" << endl;
    return 0;
}       

// Método que calcula o diâmetro do grafo de forma aproximada -> não implementado para grafos com pesos por estar fora do escopo do trabalho
int GrafoComPeso::diametro_aprox(){ 
    cout << "Nao implementado" << endl;
    return 0;
}

// Método que realiza o algoritmo de Dijsktra
// s é o vértice de origem, e heap é um parâmetro que define se o algoritmo será executado com uso de heap (1) ou sem uso de heap (0)
void GrafoComPeso::Dijkstra(int s, int heap){

    if (negativo) { // Dijkstra não funciona com pesos negativos
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

                    if (distPeso[v] > distPeso[u] + peso_uv){ // se foi encontrado um dist[v] menor, muda o dist[v]
                        
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

                        if (distPeso[v] > distPeso[u] + peso_uv){ // se foi encontrado um dist[v] menor, muda o dist[v]
                            distPeso[v] = distPeso[u] + peso_uv;
                            pai[v] = u;
                        }
                    }
                }
            }
        }

    }

    else { // versão com heap
        // Implementar com heap -> usamos priority queue, equivalente a min-heap ao definir o comparator como greater
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

// Método que calcula a distância mínima entre os vértices v e u -> válido apenas para grafos sem pesos negativos. 
// O parâmetro heap define se o Dijkstra será executado com heap (1) ou sem heap (0), e o parâmetro print_caminho define se o caminho mínimo será impresso junto da distância mínima
float GrafoComPeso::distancia(int v, int u, int print_caminho, int heap){
    v--; u--; 

    if (pai[v] != -3) Dijkstra(v,heap); // se o último Dijkstra feito foi executado com raiz no primeiro dos vértices passados
                                        // para a função atual, não é necessário rodar o Dijkstra novamente

    // cout << "Pais: ";
    // for (int i : pai){
    //     cout << i+1 << ", ";
    // }
    // cout << endl;
    
    vector<int> caminho;
    if (print_caminho == 1){ // printar o caminho mínimo
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

    if (distPeso[u] ==  INF) return -3; // não existe caminho entre os vértices, definimos -3 como valor de retorno para indicar isso
    return distPeso[u]; // retorna a distância mínima entre os vértices
}

// Método que imprime a lista de adjacências com pesos
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

// Método que imprime a matriz de adjacências com pesos
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



// Adicionar a função dfs para encontrar um caminho aumentante
bool GrafoComPeso::bfs_FF(int s, int t) {
    vector<bool> vis(V, false);
    vis[s] = true;
    queue<int> fila;
    fila.push(s);

    while (!fila.empty()) { // enquanto a fila não estiver vazia
        int u = fila.front();
        fila.pop();

        for (int i = 0; i < residual[u].size(); ++i) {
            int v = residual[u][i].second; // vértice adjacente
            int capacity = residual[u][i].first.second; // capacidade da aresta

            if (!vis[v] && capacity > 0) { // se o vértice não foi visitado e a capacidade da aresta é maior que 0
                fila.push(v);
                if (residual[u][i].first.first == 1) pai_duplo[v] = {u, i}; // se a aresta é de origem
                else pai_duplo[v] = {u, i}; // se a aresta é reversa
                vis[v] = true;

                if (v == t) return true;
            }
        }
    }

    return false;
}

// Método que calcula o gargalo (capacidade mínima) do caminho encontrado pelo Ford-Fulkerson
int GrafoComPeso::gargalo(int s, int t) {
    int gargalo = INF;
    int v = t;
    while (v != s) { // percorre o caminho de t até s, pelo pai de cada vértice
        int u = pai_duplo[v].first; // vértice pai
        int i = pai_duplo[v].second; // índice da aresta na lista de adjacências
        gargalo = min(gargalo, residual[u][i].first.second); // atualiza o gargalo
        v = u;
    }
    
    return gargalo;
}

// Método que atualiza o grafo residual após encontrar um caminho aumentante no Ford-Fulkerson
void GrafoComPeso::atualizar_grafo_residual(int u, int v, int gargalo) {
    while (v != u) {
        int p = pai_duplo[v].first; // vértice pai
        int i = pai_duplo[v].second; // índice da aresta na lista de adjacências
        
        residual[p][i].first.second -= gargalo; // diminui a capacidade da aresta no grafo residual
                                                // Peso da aresta p -> v -= gargalo
        
        // Atualiza, no grafo residual, a aresta contrária à atualizada acima
        for (int j = 0; j < residual[v].size(); ++j) {
            if (residual[v][j].second == p && residual[v][j].first.first == !(residual[p][i].first.first)) {
            // se a aresta é v -> p & se seu tipo é contrário ao de antes (original e reversa)
                residual[v][j].first.second += gargalo; // Peso da aresta v -> p += gargalo
                break;
            }
        }
        v = p;
    }
}

void GrafoComPeso::atualizar_grafo_original() { // Método que atualiza o grafo original ao fim do Ford-Fulkerson

    for (int v = 0; v < V; ++v) { // Percorrendo todo o grafo residual
        for (int j = 0; j < residual[v].size(); ++j) {
            if (residual[v][j].first.first == 0){ // Se a aresta v -> u no grafo residual for reversa
                int u = residual[v][j].second;
                int p = residual[v][j].first.second; // Sendo p o peso da aresta v -> u no grafo residual

                for (int i = 0 ; i < original[u].size() ; ++i){ // Encontrando aresta u -> v no grafo original
                    int w = original[u][i].second;
                    if (w == v){
                        original[u][i].first.second = p; // Fluxo da aresta u -> v original é igual a p
                    }
                }
            }
        }        
    }
}

// Método que realiza o algoritmo de Ford-Fulkerson para fluxo máximo
pair <int,vector<vector<pair< pair<int,int>,int>>>> GrafoComPeso::ford_fulkerson(int s, int t, int print) {
    // cout << "checkpoint 1 " << endl ;
    clock_t start = clock();
    s--; t--;
    int fluxo_max = 0;

    if (negativo) { // Ford-Fulkerson não funciona com pesos negativos (capacidade negativa)
        cout << "O grafo possui arestas com peso negativo, a biblioteca nao implementa fluxo máximo com pesos negativos!" << endl;
        return {0,{}};
    }

    // Enquanto houver um caminho aumentante de chegada 's' ao vértice de destino 't'
    while (bfs_FF(s, t)) {
        int garg = gargalo(s, t); // calcula o gargalo do caminho        
        atualizar_grafo_residual(s, t, garg); // Atualiza pesos das arestas no grafo residual
        fluxo_max += garg; // Adiciona o fluxo do caminho ao fluxo máximo
    }

    atualizar_grafo_original(); // Atualizando o fluxo do grafo original


    clock_t end = clock();
    cout << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl; // Tempo de execução

    if (print == 1) { // printar informações do Ford-Fulkerson no txt
        std::ofstream outputFile("ford_fulkerson_info.txt");
        if (outputFile.is_open()) {
            for (int i = 0; i < V; ++i) {
                for (int j = 0; j < original[i].size(); ++j) {
                    if (original[i][j].first.second > 0)
                    outputFile << "Aresta (" << i + 1 << " -> " << original[i][j].second + 1 << "): fluxo = " << original[i][j].first.second << endl;
                    // cout << "Aresta (" << i + 1 << " -> " << original[i][j].second + 1 << "): fluxo = " << original[i][j].first.second << endl;
                }
            }
            outputFile << "Fluxo máximo: " << fluxo_max << endl << endl;
            cout << "-> Informacoes de Ford-Fulkerson salvas em 'ford_fulkerson_info.txt'" << endl;
        } else {
            std::cerr << "-> Nao foi possível criar o arquivo de saida" << "\n";
        }
        outputFile << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;
    }
    // else{ // Printar alocação de fluxo em cada aresta
    //     for (int i = 0; i < V; ++i) {
    //         for (int j = 0; j < original[i].size(); ++j) {
    //             cout << "Aresta (" << i + 1 << " -> " << original[i][j].second + 1 << "): fluxo = " << original[i][j].first.second << endl;
    //         }
    //     }
    // }
    
    return {fluxo_max, original};
}
