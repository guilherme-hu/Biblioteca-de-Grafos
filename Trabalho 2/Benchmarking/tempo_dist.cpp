#include "bib_grafo.h"
#include <cassert>
#include <iostream>
#include <random>
#include <chrono>
#include <vector>
#include <fstream>

void testDijkstra(const std::string& filename, std::vector<double>& times, bool useHeap) {
    std::ifstream file(filename);
    int vertices;
    file >> vertices;
    file.close();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, vertices);

    GrafoComPeso g(filename, 0);

    for (int i = 0; i < 100; ++i) {
        int random_vertex = dis(gen);
        auto start = std::chrono::high_resolution_clock::now();
        for(int j = 0; j < vertices; ++j) {
            g.distancia(random_vertex, j, 0, useHeap);
        }
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> duration = end - start;
        times.push_back(duration.count());

    }
    std::cout << "testDijkstra for " << filename << " passed 100 times!" << std::endl;
}

int main() {
    std::cin.tie(NULL);
    std::ios_base::sync_with_stdio(false);

    // Function to calculate mean
    auto calculateMean = [](const std::vector<double>& times) {
        double sum = 0.0;
        for (const auto& time : times) {
            sum += time;
        }
        return sum / times.size();
    };

    // Function to calculate standard deviation
    auto calculateStdDev = [](const std::vector<double>& times, double mean) {
        double sum = 0.0;
        for (const auto& time : times) {
            sum += (time - mean) * (time - mean);
        }
        return std::sqrt(sum / times.size());
    };

    // Save times and statistics to a file
    auto saveTimesAndStats = [&](const std::string& name, const std::vector<double>& times) {
        std::ofstream outFile(name);
        double mean = calculateMean(times);
        double stdDev = calculateStdDev(times, mean);
        
        outFile << "Times for " << name << ":\n";
        outFile << "[ ";
        for (size_t i = 0; i < times.size(); ++i) {
            outFile << times[i];
            if (i != times.size() - 1) {
                outFile << ", ";
            }
        }
        outFile << " ]\n";

        outFile << "Mean: " << mean << " seconds\n";
        outFile << "Standard Deviation: " << stdDev << " seconds\n\n";
        outFile.close();
    };


    std::vector<double> times_grafo_W_1, times_grafo_W_2, times_grafo_W_3, times_grafo_W_4, times_grafo_W_5;

    testDijkstra("grafo_W_1.txt", times_grafo_W_1, true); 
        saveTimesAndStats("dij_1.txt", times_grafo_W_1);

    testDijkstra("grafo_W_2.txt", times_grafo_W_2, true); 
        saveTimesAndStats("dij_2.txt", times_grafo_W_2);

    testDijkstra("grafo_W_3.txt", times_grafo_W_3, true); 
        saveTimesAndStats("dij_3.txt", times_grafo_W_3);

    testDijkstra("grafo_W_4.txt", times_grafo_W_4, true); 
        saveTimesAndStats("dij_4.txt", times_grafo_W_4);

    testDijkstra("grafo_W_5.txt", times_grafo_W_5, true); 
        saveTimesAndStats("dij_5.txt", times_grafo_W_5);
        
    std::cout << "All Dijkstra tests passed 100 times!" << std::endl;

    return 0;
}
