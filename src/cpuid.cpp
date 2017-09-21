#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>

int main(int argc, char** argv) {
    std::ifstream info(CPUINFO_PATH);
    if (!info) return 1;

    std::string line;
    std::vector<std::string> isa_list{
        "neon", "sse4_2", "avx", "avx2"
    };
    std::map<std::string, bool> detected;
    while (std::getline(info, line)) {
        for (auto isa : isa_list) {
            detected[isa] |= line.find(isa) != std::string::npos;
        }
    }
    bool found = false;
    for (auto pair : detected) {
        found |= pair.second;
        if (pair.second) std::cout << pair.first << ' ';
    }
    return found ? 0 : 1;
}
