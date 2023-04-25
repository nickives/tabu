#include "ProblemReader.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <vector>

using namespace std;

vector<string> tokenize(const string& s) {
    vector<string> result;
    result.reserve(7);
    stringstream ss(s);
    string word;
    while (ss >> word) {
        if (!word.empty()) {
            result.push_back(word);
        }
    }
    return result;
}

DARProblem ProblemReader::read(const std::string& filename)
{
    const std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file" << std::endl;
        throw exception("Unable to open file");
    }



    return DARProblem();
}
