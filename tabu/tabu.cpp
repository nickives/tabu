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

    const int max_iterations = 10000;
    const double penalty_lambda = 0.015;

    const filesystem::path darpPath(filesystem::current_path().string() + D_SEP + "data" + D_SEP + "darp");
    const filesystem::path tabuPath(filesystem::current_path().string() + D_SEP + "data" + D_SEP + "tabu");
    //const std::string darpPath{ "C:\\Users\\nicki\\source\\repos\\nickives\\tabu\\tabu\\data\\darp"};
    //const std::string tabuPath{ "C:\\Users\\nicki\\source\\repos\\nickives\\tabu\\tabu\\data\\tabu" };
    //const std::string tabuPath{ "C:\\Users\\nicki\\source\\repos\\nickives\\tabu\\tabu\\data\\test" };

    vector<DARProblem> problems;
    problems.reserve(500);
    for (const auto& entry : std::filesystem::directory_iterator(tabuPath)) {
        problems.emplace_back(ProblemReader::readTabu(entry.path().string()));
    }
    for (const auto& entry : std::filesystem::directory_iterator(darpPath)) {
        problems.emplace_back(ProblemReader::readDarp(entry.path().string()));
    }
    try {

        const filesystem::path base_path(filesystem::current_path().string() + D_SEP +  "data" + D_SEP + "results");

        for (const auto& p : problems) {
            const auto& result_filename(base_path.string() + D_SEP + p.path.filename().string());
            const double theta = 7.5 * std::log10((p.number_of_requests + 1) * 2);
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