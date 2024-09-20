#include "bib_grafo.h"
#include <cassert>
#include <iostream>
#include <random>
#include <chrono>
#include <vector>
#include <fstream>

// void testDFS(const std::string& filename, std::vector<double>& times) {
//     std::ifstream file(filename);
//     int first_vertex;
//     file >> first_vertex;
//     file.close();

//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_int_distribution<> dis(1, first_vertex);

//     Grafo g(filename, 0);

//     for (int i = 0; i < 100; ++i) {
//         int random_vertex = dis(gen);

//         auto start = std::chrono::high_resolution_clock::now();
//         g.dfs(random_vertex);
//         auto end = std::chrono::high_resolution_clock::now();

//         std::chrono::duration<double> duration = end - start;
//         times.push_back(duration.count());
//     }
//     std::cout << "testDFS for " << filename << " passed 100 times!" << std::endl;
//     cout << endl;
// }

// int main() {

//     cin.tie(NULL);
//     ios_base::sync_with_stdio(false);

//     std::vector<double> times_grafo_1, times_grafo_2, times_grafo_3, times_grafo_4, times_grafo_5, times_grafo_6;

//     testDFS("grafo_1.txt", times_grafo_1); 
//     testDFS("grafo_2.txt", times_grafo_2); 
//     testDFS("grafo_3.txt", times_grafo_3); 
//     testDFS("grafo_4.txt", times_grafo_4); 
//     testDFS("grafo_5.txt", times_grafo_5); 
//     testDFS("grafo_6.txt", times_grafo_6); 

//     std::cout << "All DFS tests passed 100 times!" << std::endl;
//     cout << endl;

//     // Function to calculate mean
//     auto calculateMean = [](const std::vector<double>& times) {
//         double sum = 0.0;
//         for (const auto& time : times) {
//             sum += time;
//         }
//         return sum / times.size();
//     };

//     // Function to calculate standard deviation
//     auto calculateStdDev = [](const std::vector<double>& times, double mean) {
//         double sum = 0.0;
//         for (const auto& time : times) {
//             sum += (time - mean) * (time - mean);
//         }
//         return std::sqrt(sum / times.size());
//     };

//     // Save times and statistics to a file
//     std::ofstream outFile("times_and_stats_DFS.txt");

//     auto saveTimesAndStats = [&](const std::string& name, const std::vector<double>& times) {
//         double mean = calculateMean(times);
//         double stdDev = calculateStdDev(times, mean);
        
//         outFile << "Times for " << name << ":\n";
//         for (const auto& time : times) {
//             outFile << time << " seconds\n";
//         }
//         outFile << "Mean: " << mean << " seconds\n";
//         outFile << "Standard Deviation: " << stdDev << " seconds\n\n";
//     };

//     saveTimesAndStats("grafo_1", times_grafo_1);
//     saveTimesAndStats("grafo_2", times_grafo_2);
//     saveTimesAndStats("grafo_3", times_grafo_3);
//     saveTimesAndStats("grafo_4", times_grafo_4);
//     saveTimesAndStats("grafo_5", times_grafo_5);
//     saveTimesAndStats("grafo_6", times_grafo_6);

//     outFile.close();

//     return 0;
// }


// // main para testes gerais
int main() {
    cin.tie(NULL);
    ios_base::sync_with_stdio(false);
    
    string FileName = "grafo_teste.txt";
    int mode = 1; 

    clock_t start = clock();

    Grafo g(FileName, mode);
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
    }
    
    g.bfs(17);
    g.dfs(17);
    cout << "Distancia: " << g.distancia(7, 14) << endl;

    cout << "Diametro: " << g.diametro() << endl;

    clock_t end = clock();

    cout << "Tempo de execucao: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;

    return 0;   

}



//  g++ bib_grafo.cpp main.cpp -o main -O2
// ./main
