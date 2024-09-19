import matplotlib.pyplot as plt
"""
This script generates a bar chart using matplotlib to display the average times of BFS (Breadth-First Search) 
for different graphs represented by adjacency lists.
Functions:
    None
Variables:
    medias (list of float): A list containing the average times for BFS.
    categorias (list of str): A list containing the names of the graphs.
    plt (module): The matplotlib.pyplot module used for plotting.
Usage:
    Run the script to display a bar chart with the average BFS times for each graph.
Color Options:
    RGB tuples: (R, G, B) where R, G, B are floats between 0 and 1.
"""

# Valores das médias
medias = [0.0714254, 0.139992, 1.64059 , 1.27249, 7.20167, 4.58899]

# Valores do eixo x
categorias = ['grafo_1', 'grafo_2', 'grafo_3', 'grafo_4', 'grafo_5', 'grafo_6']

# Criar o gráfico de barras
plt.figure(figsize=(10, 5))
plt.bar(categorias, medias, color=(98/255,137/255,238/255))  # Alterar a cor para verde

# Adicionar título e rótulos aos eixos
plt.title('Gráfico de Médias de Tempo da DFS por Lista de Adjacência')
plt.xlabel('Grafos')
plt.ylabel('Médias (segundos)')

# Ajustar a escala do eixo y
plt.ylim(0, 8)  # Ajuste os valores conforme necessário

# Exibir o gráfico
plt.grid(True)
plt.show()