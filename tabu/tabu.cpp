// tabu.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <chrono>
#include <filesystem>
#include <string>

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

#include <iostream>
#include <vector>
#include "SolutionPrinter.h"
#include "tabutypes.h"
#include "TabuSearch.h"
#include "ProblemReader.h"
#include "SolutionFileWriter.h"

#ifdef _WIN32
#define D_SEP "\\"
#else
#define D_SEP "/"
#endif

int main() {

    std::vector<Node> nodes = {
        {0, 0, 0, 0, 0, 1400, 0, 0},
        {1, 1, 1, 10, 0, 1400, 9, 8},
        {2, 2, 1, 5, 0, 1400, 6, 7},
        {3, 3, 1, 10, 0, 1400, 7, 9},
        {4, 4, 1, 5, 0, 1400, 3, 5},
        {5, 5, 1, 5, 0, 1400, -2, 5},
        {6, 6, 1, 5, 0, 1400, 5, 4.8},
        {7, 7, 1, 5, 0, 1400, -2, 5.9},
        {8, 8, 1, 5, 0, 1400, 8.4, 9.6},
        {9, 9, 1, 5, 0, 1400, 5, 2},
        {10, 10, 1, 5, 0, 1400, -7.5, -9.1},
        {11, 1, -1, 5, 600, 700, -5, 4.8},
        {12, 2, -1, 5, 500, 650, 6, 5.9},
        {13, 3, -1, 5, 800, 900, -8.4, 5.6},
        {14, 4, -1, 5, 500, 950, 5, -0.02},
        {15, 5, -1, 5, 500, 600, 7.5, 5.1},
        {16, 6, -1, 5, 500, 550, -9.6, 4.8},
        {17, 7, -1, 5, 800, 800, -1.789, 1.9},
        {18, 8, -1, 5, 400, 500, 0.4, 9.6},
        {19, 9, -1, 5, 500, 600, -5, 5},
        {20, 10, -1, 5, 826, 936, 7.5, 9.1},
        {21, 0, 0, 0, 0, 1400, 0, 0}
    };

    const int max_iterations = 100000;
    const int max_vehicle_load = 6;
    const int planning_horizon = 1000;
    const int max_ride_time = 90;
    const double penalty_lambda = 0.015;
    const int num_vehicles = 4;
    const double tabu_duration_theta = 7.5 * std::log10(nodes.size());

    DARProblem problem{
        nodes, num_vehicles, (uint16_t)((nodes.size() / 2) - 1), max_vehicle_load, max_ride_time,
        planning_horizon, ""
    };

    const std::string filename{ "C:\\Users\\nicki\\source\\repos\\nickives\\tabu\\tabu\\data\\darp\\a2-16" };
    const std::string darpPath{ "C:\\Users\\nicki\\source\\repos\\nickives\\tabu\\tabu\\data\\darp"};
    const std::string tabuPath{ "C:\\Users\\nicki\\source\\repos\\nickives\\tabu\\tabu\\data\\tabu" };

    vector<DARProblem> problems;
    problems.reserve(500);
    for (const auto& entry : std::filesystem::directory_iterator(tabuPath)) {
        problems.emplace_back(ProblemReader::readTabu(entry.path().string()));
    }
    for (const auto& entry : std::filesystem::directory_iterator(darpPath)) {
        problems.emplace_back(ProblemReader::readDarp(entry.path().string()));
    }

    DARProblem from_file = ProblemReader::readTabu(filename);

    //TabuSearch ts(max_vehicle_load, planning_horizon, tabu_duration_theta, penalty_lambda, problem);

    const double from_file_theta = 7.5 * std::log10(from_file.number_of_requests * from_file.number_of_vehicles);

    //TabuSearch ts(
    //    from_file.vehicle_capacity,
    //    from_file.planning_horizon,
    //    from_file_theta,
    //    penalty_lambda,
    //    from_file
    //);
    
    try {
        //auto start_time = std::chrono::steady_clock::now();
        //auto best_solution = ts.search(max_iterations);
        //auto end_time = std::chrono::steady_clock::now();
        //auto time_diff = end_time - start_time;
        //std::cout << "Best solution cost: " << best_solution.cost << std::endl;
        //SolutionPrinter::print_solution(best_solution.solution);
        //auto time_diff_minutes = std::chrono::duration_cast<std::chrono::minutes>(time_diff);
        //auto time_diff_seconds = std::chrono::duration_cast<std::chrono::seconds>(time_diff);
        //auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_diff);
        //std::cout << "Running time: " << time_diff_minutes << ":" << time_diff_seconds << ":" << time_diff_ms << std::endl;

        const filesystem::path base_path(filesystem::current_path().string() + D_SEP +  "data" + D_SEP + "results");

        for (const auto& p : problems) {
            const auto& result_filename(base_path.string() + D_SEP + p.path.filename().string());
            const double theta = 7.5 * std::log10(p.number_of_requests * p.number_of_vehicles);
            TabuSearch ts(
                p.vehicle_capacity,
                p.planning_horizon,
                theta,
                penalty_lambda,
                p
            );
            SolutionFileWriter writer(result_filename);
            auto start_time = std::chrono::steady_clock::now();
            auto best_solution = ts.search(max_iterations);
            auto end_time = std::chrono::steady_clock::now();
            auto time_diff = end_time - start_time;
            auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_diff);
            writer.addSolution(best_solution);
            writer.addRunningTime(time_diff_ms);
            writer.close();
        }
    }
    catch (const std::bad_alloc& e) {
        std::cout << e.what() << std::endl;
    }
    
    return 0;
}