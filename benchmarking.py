import subprocess
import random
import time
import numpy as np
import matplotlib.pyplot as plt

# Lista de arquivos de grafos
grafos = ["grafo_1.txt", "grafo_2.txt", "grafo_3.txt", "grafo_4.txt", "grafo_5.txt", "grafo_6.txt"]

# Função para executar o arquivo C++ e medir o tempo de execução
def executar_benchmark(grafo, mode, k):
    start_time = time.time()
    result = subprocess.run(["./bib_grafo.exe", grafo, str(mode), str(k)], capture_output=True, text=True)
    end_time = time.time()
    return end_time - start_time

# Função para ler o primeiro valor do arquivo
def ler_primeiro_valor(arquivo):
    with open(arquivo, 'r') as f:
        return int(f.readline().strip())

# Arrays para armazenar os tempos de execução
tempos_mode_0 = {grafo: [] for grafo in grafos}
tempos_mode_1 = {grafo: [] for grafo in grafos}

# Loop de benchmarking
for grafo in grafos:
    primeiro_valor = ler_primeiro_valor(grafo)
    for _ in range(100):
        k = random.randint(0, primeiro_valor)
        tempos_mode_0[grafo].append(executar_benchmark(grafo, 0, k))
        tempos_mode_1[grafo].append(executar_benchmark(grafo, 1, k))

# Função para calcular a média dos tempos
def calcular_media(tempos):
    return {grafo: np.mean(tempos[grafo]) for grafo in tempos}

# Calcular médias
medias_mode_0 = calcular_media(tempos_mode_0)
medias_mode_1 = calcular_media(tempos_mode_1)

# Função para calcular o desvio padrão dos tempos
def calcular_desvio_padrao(tempos):
    return {grafo: np.std(tempos[grafo]) for grafo in tempos}

# Calcular desvios padrão
desvios_mode_0 = calcular_desvio_padrao(tempos_mode_0)
desvios_mode_1 = calcular_desvio_padrao(tempos_mode_1)

# Plotar os resultados
labels = grafos
mode_0_means = [medias_mode_0[grafo] for grafo in grafos]
mode_1_means = [medias_mode_1[grafo] for grafo in grafos]

x = np.arange(len(labels))  # Posições das labels
width = 0.35  # Largura das barras

fig, ax = plt.subplots()
rects1 = ax.bar(x - width/2, mode_0_means, width, label='Mode 0')
rects2 = ax.bar(x + width/2, mode_1_means, width, label='Mode 1')

# Adicionar texto e labels
ax.set_xlabel('Grafos')
ax.set_ylabel('Tempo Médio (s)')
ax.set_title('Tempo Médio de Execução por Grafo e Modo')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()

fig.tight_layout()
plt.show()

# Preparar tabelas com os resultados
import pandas as pd

df_mode_0 = pd.DataFrame.from_dict(medias_mode_0, orient='index', columns=['Tempo Médio Mode 0'])
df_mode_1 = pd.DataFrame.from_dict(medias_mode_1, orient='index', columns=['Tempo Médio Mode 1'])

# Exibir tabelas
print("Resultados Mode 0:")
print(df_mode_0)
print("\nResultados Mode 1:")
print(df_mode_1)
