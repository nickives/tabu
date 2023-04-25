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



    // extract header
    char input[50]{};
    file.getline(input, 50);
    auto first_line = tokenize(input);
    numberOfVehicles_ = stoi(first_line[0]);
    numberOfRequests_ = stoi(first_line[1]);
    planningHorizon_ = stoi(first_line[2]);
    vehicleCapacity_ = stoi(first_line[3]);
    maximumRideTime_ = stoi(first_line[4]);

    numberOfNodes_ = numberOfRequests_ * 2;

    // extract requests
    while (!file.eof()) {
        file.getline(input, 50);
        auto current_line = tokenize(input);
        if (!current_line.empty()) {
            // [node num] [X] [Y] [q_i - load] [d_i - service duration] [start time] [end time]
            Request r{
                    stod(current_line[1]), stod(current_line[2]), stoi(current_line[3]),
                    stoi(current_line[4]),stoi(current_line[3]), stoi(current_line[3])
            };
            requests_.push_back(r);
        }
    }
    file.close();

    // Read distances between nodes
    distances_ = std::vector<std::vector<double>>(numberOfNodes_, std::vector<double>(numberOfNodes_));
    for (int i = 0; i < numberOfNodes_; ++i) {
        for (int j = 0; j < numberOfNodes_; ++j) {
            distances_[i][j] = arc_cost(requests_[i], requests_[j]);
        }
    }

    // Read time windows for each request
    timeWindows_ = std::vector<std::pair<double, double>>(numberOfNodes_);
    for (int i = 0; i < numberOfNodes_; ++i) {
        file >> timeWindows_[i].first >> timeWindows_[i].second;
    }

    // Read pickup and delivery nodes for each request
    pickupDeliveryPairs_ = std::vector<std::pair<int, int>>(numberOfNodes_ / 2);
    for (int i = 0; i < numberOfNodes_ / 2; ++i) {
        file >> pickupDeliveryPairs_[i].first >> pickupDeliveryPairs_[i].second;
    }

    return DARProblem();
}
