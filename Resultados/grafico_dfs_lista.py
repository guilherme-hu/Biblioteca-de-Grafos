# import matplotlib.pyplot as plt
# """
# This script generates a bar chart using matplotlib to display the average times of BFS (Breadth-First Search) 
# for different graphs represented by adjacency lists.
# Functions:
#     None
# Variables:
#     medias (list of float): A list containing the average times for BFS.
#     categorias (list of str): A list containing the names of the graphs.
#     plt (module): The matplotlib.pyplot module used for plotting.
# Usage:
#     Run the script to display a bar chart with the average BFS times for each graph.
# Color Options:
#     RGB tuples: (R, G, B) where R, G, B are floats between 0 and 1.
# """

# # Valores das médias
# medias = [0.00096044 , 0.00460351, 0.0156679 , 0.0736509, 0.403126, 0.648353]

# # Valores do eixo x
# categorias = ['grafo_1', 'grafo_2', 'grafo_3', 'grafo_4', 'grafo_5', 'grafo_6']

# # Criar o gráfico de barras
# plt.figure(figsize=(10, 5))
# plt.bar(categorias, medias, color=(98/255,137/255,238/255))  # Alterar a cor para verde

# # Adicionar título e rótulos aos eixos
# plt.title('Gráfico de Médias de Tempo da DFS por Lista de Adjacência')
# plt.xlabel('Grafos')
# plt.ylabel('Médias (segundos)')

# # Ajustar a escala do eixo y
# plt.ylim(0, 0.7)  # Ajuste os valores conforme necessário

# # Exibir o gráfico
# plt.grid(True)
# plt.show()



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
medias = [0.00096044 , 0.00460351, 0.0156679 , 0.0736509, 0.403126, 0.648353]

# Valores do eixo x
categorias = ['grafo_1', 'grafo_2', 'grafo_3', 'grafo_4', 'grafo_5', 'grafo_6']

# Criar o gráfico de barras
plt.figure(figsize=(10, 5))
plt.bar(categorias, medias, color=(139/255, 169/255, 1))  # Alterar a cor para verde

# Adicionar título e rótulos aos eixos
plt.title('Gráfico de Médias de Tempo da DFS por Lista de Adjacência')
plt.xlabel('Grafos')
plt.ylabel('Médias (segundos)')

# Ajustar a escala do eixo y para logarítmica
plt.yscale('log')

# Exibir o gráfico
plt.grid(True, which="both", ls="--")
plt.show()
