#include "ProblemReader.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <vector>

#include "tabutypes.h"

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

DARProblem ProblemReader::readTabu(const filesystem::path& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file" << std::endl;
        throw filesystem::filesystem_error("Unable to open file", filename, std::make_error_code(std::errc::io_error));
    }

    // extract header
    char input[50]{};
    file.getline(input, 50);
    auto first_line = tokenize(input);
    const uint16_t number_of_vehicles = stoi(first_line[0]);
    const uint16_t number_of_nodes = stoi(first_line[1]);
    const double planningHorizon = stoi(first_line[2]);
    const uint16_t vehicle_capacity = stoi(first_line[3]);
    const uint16_t maximum_ride_time = stoi(first_line[4]);

    const auto number_of_requests = number_of_nodes / 2;
    vector<Node> nodes;
    nodes.reserve(number_of_nodes);

    Node depot;

    // extract requests
    uint64_t request_id = 0;
    bool is_depot_line = true;
    while (!file.eof()) {
        file.getline(input, 50);
        auto current_line = tokenize(input);
        if (!current_line.empty()) {
            // [i - node num] [X] [Y] [d_i - service duration] [q_i - load] [e_i - start time] [l_i - end time]
            NodeId id = stoi(current_line[0]);
            double x = stod(current_line[1]);
            double y = stod(current_line[2]);
            double service_duration = stod(current_line[3]);
            int16_t load = stoi(current_line[4]);
            double time_window_start = stod(current_line[5]);
            double time_window_end = stod(current_line[6]);
            Node n{
                id, request_id, load, service_duration, time_window_start, time_window_end, x, y
            };

            if (is_depot_line)
            {
                depot = n;
                is_depot_line = false;
            }

            nodes.push_back(n);
            request_id += 1;

            if (id == number_of_requests) {
                request_id = 1;
            }
        }
    }
    file.close();
    depot.id = nodes.size();
    nodes.push_back(depot);

    return DARProblem(
        nodes, number_of_vehicles, number_of_requests, vehicle_capacity,
        maximum_ride_time, planningHorizon, filename
    );
}


DARProblem ProblemReader::readDarp(const filesystem::path& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file" << std::endl;
        throw filesystem::filesystem_error("Unable to open file", filename, std::make_error_code(std::errc::io_error));
    }

    // extract header
    char input[50]{};
    file.getline(input, 50);
    auto first_line = tokenize(input);
    const uint16_t number_of_vehicles = stoi(first_line[0]);
    const uint16_t number_of_requests = stoi(first_line[1]);
    const double planningHorizon = stoi(first_line[2]);
    const uint16_t vehicle_capacity = stoi(first_line[3]);
    const uint16_t maximum_ride_time = stoi(first_line[4]);

    const auto number_of_nodes = (number_of_requests * 2) + 2;

    vector<Node> nodes;
    nodes.reserve(number_of_nodes);

    Node depot;

    // extract requests
    uint64_t request_id = 0;
    bool is_depot_line = true;
    while (!file.eof()) {
        file.getline(input, 50);
        auto current_line = tokenize(input);
        if (!current_line.empty()) {
            // [i - node num] [X] [Y] [d_i - service duration] [q_i - load] [e_i - start time] [l_i - end time]
            NodeId id = stoi(current_line[0]);
            double x = stod(current_line[1]);
            double y = stod(current_line[2]);
            double service_duration = stod(current_line[3]);
            int16_t load = stoi(current_line[4]);
            double time_window_start = stod(current_line[5]);
            double time_window_end = stod(current_line[6]);
            Node n{
                id, request_id, load, service_duration, time_window_start, time_window_end, x, y
            };

            if (is_depot_line)
            {
                depot = n;
                is_depot_line = false;
            }

            nodes.push_back(n);
            request_id += 1;

            if (id == number_of_requests) {
                request_id = 1;
            }
        }
    }
    file.close();
    nodes[nodes.size() - 1].request_id = 0;

    return DARProblem(
        nodes, number_of_vehicles, number_of_requests, vehicle_capacity,
        maximum_ride_time, planningHorizon, filename
    );
}
