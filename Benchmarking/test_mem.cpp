#include "bib_grafo.h"
#include <iostream>
#include <cassert>
#include <vector>
#include <string>

void testMemoryUsage(const std::vector<std::string>& graphFiles) {
    for (const auto& file : graphFiles) {
        // Test adjacency list representation (mode 0)
        Grafo gList(file, 0);
        size_t actualAdjMemoryUsage = gList.getAdjMemoryUsage();
        std::cout << "Memória usada por adj (lista) para " << file << ": " << actualAdjMemoryUsage << " bytes" << std::endl;

        // Test adjacency matrix representation (mode 1)
        Grafo gMat(file, 1);
        size_t actualMatMemoryUsage = gMat.getMatMemoryUsage();
        std::cout << "Memória usada por mat (matriz) para " << file << ": " << actualMatMemoryUsage << " bytes" << std::endl;
    }
}

int main() {
    std::vector<std::string> graphFiles = {
        "grafo_1.txt",
        "grafo_2.txt",
        "grafo_3.txt",
        "grafo_4.txt",
        "grafo_5.txt",
        "grafo_6.txt"
    };

    testMemoryUsage(graphFiles);
    std::cout << "All tests completed!" << std::endl;
    return 0;
}
