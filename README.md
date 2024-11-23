# Uso da Biblioteca

Esse projeto é faz parte do trabalho da disciplina de Teoria dos Grafos - COS242, cuja proposta é implementar uma biblioteca capaz de armazenar, manipular e 
obter diferentes informações sobre grafos repassados por meio de um arquivo representando sua estrutura, assim como implementar um conjunto de algoritmos em grafos. 
Aqui apresentamos um guia com instruções, de forma mais direta, sobre como utilizar a biblioteca. 

### Criação de um objeto da classe:

```cpp
Grafo G (arquivo txt, mode) // Para grafos não direcionados e sem pesos nas arestas
GrafoComPeso G (arquivo txt, mode, direcionado) // Para grafos com pesos nas arestas, podendo ser definido se o grafo é ou não direcionado
```

### Chamada dos Métodos Principais (funcionam para todos objetos):

```cpp
G.bfs(int vértice inicial) // retorna um arquivo txt com a árvore geradora criada pela bfs
G.dfs(int vértice inicial) // retorna um arquivo txt com a árvore geradora criada pela dfs
G.distancia(int vértice 1, int vértice 2) // retorna a distância entre os vértices 1 e 2, ou, caso não seja possível um caminho entre eles, -1 é retornado
G.diametro() // retorna o diâmetro exato do grafo. Para grafos muito grandes, o tempo de cálculo é enorme,
		// então se recomenda usar o método diametro_aprox, que calcula o diâmetro aproximado de modo bem veloz
G.diametro_aprox() // retorna o diâmetro aproximado, bem mais rápido que o método diâmetro normal
```

- G.bfs(int vértice inicial) → retorna um arquivo txt com a árvore geradora criada pela bfs de raiz escolhida
- G.dfs(int vértice inicial) → retorna um arquivo txt com a árvore geradora criada pela dfs de raiz escolhida
- G.distancia(int vértice 1, int vértice 2) → retorna a distância entre os vértices 1 e 2, ou, caso não seja possível um caminho entre eles, -1 é retornado
- G.diametro() → retorna o diâmetro exato do grafo. Para grafos muito grandes, o tempo de cálculo é enorme, então se recomenda usar o método diametro_aprox, que calcula o diâmetro aproximado de modo bem veloz
- G.diametro_aprox() → retorna o diâmetro aproximado, bem mais rápido que o método diâmetro normal

### Chamada dos Métodos Principais para GrafoComPeso (exclusivo para grafos com peso):
```cpp
G.Dijkstra(int vértice inicial, int heap) // executa o algoritmo de Dijkstra, para uso prático use o método Distância
G.distancia(int vértice 1, int vértice 2, int print_caminho, int heap) // calcula a distância entre os vértices 1 e 2, a partir do algoritmo de Dijkstra

// Se o grafo for direcionado, além dos métodos acima e os da classe pai:
G.ford_fulkerson(int vértice s, int vértice t, int print) // calcula o fluxo máximo no grafo dado, de vértices inicial s e final t 
```
- G.Dijkstra(int vértice inicial, int heap) → executa o algoritmo de Dijkstra. Esse método não retorna nada e só altera variáveis protegidas da subclasse, o usuário pode chamá-lo, mas não terá nenhum efeito visível a ele. Sua importância se deve ao seu uso no método "distância".
- G.distancia(int vértice 1, int vértice 2, int print_caminho, int heap) → override do método genérico distância. Retorna a distância mínima entre os vértices 1 e 2, ou, caso não haja um caminho entre eles, -1 é retornado. Por meio dos parâmetros passados ao método, é possível escolher se será ou não impresso o caminho mínimo, assim como escolher se, no Dijkstra dentro do método, será ou não executado usando o heap

-- Caso o grafo seja direcionado, é possível também chamar o seguinte método:
- G.ford_fulkerson(int vértice s, int vértice t, int print) -> calcula e retorna o valor do fluxo máximo no grafo direcionado G e a distribuição de fluxos para cada aresta, de vértices inicial s e final t, com a opção de salvar esses dados em um arquivo txt


### Chamada dos Métodos Extras (funcionam para todos objetos):

```cpp
G.geradortxt() // cria um arquivo txt com os dados do grafo. O construtor já chama esse método e cria esse arquivo, mas você pode recriar o txt usando esse método
G.printListAdj() // printa a lista de adjacência do grafo
G.printListAdj() // printa a matriz de adjacência do grafo
G.printCompCon(int print); // Método que printa o número de componentes conexas e o tamanho de cada, e printa (caso print = 0) ou não (caso print = 1) todos os vértices de cada componente
Métodos getters // tem de Vértice, aresta, grau máximo, grau mínimo, média do grau, mediana do grau e compcon (ajustar isso, o método retorna compcon mas isso não é printavel)
```

- G.geradortxt() → cria um arquivo txt com os dados do grafo. O construtor já chama esse método e cria esse arquivo, mas você pode recriar o txt usando esse método
- G.printListAdj() → printa a lista de adjacência do grafo
- G.printListAdj() → printa a matriz de adjacência do grafo
- G.printCompCon(int print); → Método que printa o número de componentes conexas e o tamanho de cada, e printa (caso print = 0) ou não (caso print = 1) todos os vértices de cada componente
- G.getV() → Método que retorna o número de vértices
- G.getA() → Método que retorna o número de arestas
- G.getGrauMax() → Método que retorna o grau máximo do grafo
- G.getGrauMin() → Método que retorna o  grau mínimo do grafo
- G.getGrauMediano() → Método que retorna o grau mediano do grafo
- G.getGrauMedio() → Método que retorna a média do grau do grafo
- G.getAdjMemoryUsage() → Método que calcula a memória usada pela representação em lista
- G.getMatMemoryUsage() → Método que calcula a memória usada pela representação em lista
