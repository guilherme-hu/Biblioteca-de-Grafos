#include "bib_grafo.h"
#include <cassert>
#include <iostream>
#include <random>
#include <chrono>
#include <vector>
#include <fstream>

void testBFS(const std::string& filename, std::vector<double>& times) {
    std::ifstream file(filename);
    int first_vertex;
    file >> first_vertex;
    file.close();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, first_vertex);

    Grafo g(filename, 0);

    for (int i = 0; i < 100; ++i) {
        int random_vertex = dis(gen);

        auto start = std::chrono::high_resolution_clock::now();
        g.bfs(random_vertex,1);
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> duration = end - start;
        times.push_back(duration.count());
    }
    std::cout << "testBFS for " << filename << " passed 100 times!" << std::endl;
    cout << endl;
}

int main() {

    cin.tie(NULL);
    ios_base::sync_with_stdio(false);

    std::vector<double> times_grafo_1, times_grafo_2, times_grafo_3, times_grafo_4, times_grafo_5, times_grafo_6;

    testBFS("grafo_1.txt", times_grafo_1); 
    testBFS("grafo_2.txt", times_grafo_2); 
    testBFS("grafo_3.txt", times_grafo_3); 
    testBFS("grafo_4.txt", times_grafo_4); 
    testBFS("grafo_5.txt", times_grafo_5); 
    testBFS("grafo_6.txt", times_grafo_6); 

    std::cout << "All BFS tests passed 100 times!" << std::endl;
    cout << endl;

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
    std::ofstream outFile("times_and_stats_BFS.txt");

    auto saveTimesAndStats = [&](const std::string& name, const std::vector<double>& times) {
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
    };

    saveTimesAndStats("grafo_1", times_grafo_1);
    saveTimesAndStats("grafo_2", times_grafo_2);
    saveTimesAndStats("grafo_3", times_grafo_3);
    saveTimesAndStats("grafo_4", times_grafo_4);
    saveTimesAndStats("grafo_5", times_grafo_5);
    saveTimesAndStats("grafo_6", times_grafo_6);

    outFile.close();

    return 0;
}
