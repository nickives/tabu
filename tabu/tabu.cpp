// tabu.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <chrono>

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

int main() {
 /*   std::vector<Node> nodes = {
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
            {10, 10, 1, 5, 0, 1400, 7.5, 9.1},
            {11, 11, 1, 5, 0, 1400, -5, 4.8},
            {12, 12, 1, 5, 0, 1400, 6, 5.9},
            {13, 13, 1, 5, 0, 1400, -8.4, 5.6},
            {14, 14, 1, 5, 0, 1400, 5, -0.02},
            {15, 15, 1, 5, 0, 1400, 7.5, 5.1},
            {16, 16, 1, 5, 0, 1400, -9.6, 4.8},
            {17, 17, 1, 5, 0, 1400, -1.789, 1.9},
            {18, 18, 1, 5, 0, 1400, 0.4, 9.6},
            {19, 19, 1, 5, 0, 1400, 5, 5},
            {20, 20, 1, 5, 0, 1400, 2.5, 9.1},
            {21, 21, 1, 5, 40, 400, -1.7, -6},
            {22, 22, 1, 5, 20, 350, 2.9, -5.012},
            {23, 23, 1, 5, 40, 300, -1, -7.923},
            {24, 24, 1, 10, 40, 160, -6.5, 8.2345},
            {25, 25, 1, 10, 45, 200, 6, 9.5},
            {26, 26, 1, 5, 40, 100, -1, 6},
            {27, 27, 1, 5, 20, 160, 3, -5.5},
            {28, 28, 1, 5, 40, 600, -8, -7.32},
            {29, 29, 1, 5, 60, 80, 7.5, 9.1},
            {30, 30, 1, 5, 40, 200, -1, -6},
            {31, 1, -1, 5, 40, 400, -1.2345, -6},
            {32, 2, -1, 5, 20, 300, 3, -5.5},
            {33, 3, -1, 5, 40, 300, -1, 7.32},
            {34, 4, -1, 10, 40, 160, -6.5, 2.9},
            {35, 5, -1, 10, 45, 200, -6.5, 2.5},
            {36, 6, -1, 5, 40, 100, -1, 6},
            {37, 7, -1, 5, 20, 160, 3, -5.5},
            {38, 8, -1, 5, 40, 600, -8, -5.32},
            {39, 9, -1, 10, 40, 160, -2.5, -2.9},
            {40, 10, -1, 10, 45, 200, -4.5, -2.5},
            {41, 11, -1, 5, 110, 140, -1, 3.902},
            {42, 12, -1, 5, 67, 160, 3.765, -5.5},
            {43, 13, -1, 5, 40, 300, 8, 7.32},
            {44, 14, -1, 10, 40, 160, 8.5, 2.236},
            {45, 15, -1, 10, 45, 200, 6.5, -2.5},
            {46, 16, -1, 10, 23, 64, -6.3, 3.45},
            {47, 17, -1, 10, 52, 456, 2.5, -3},
            {48, 18, -1, 10, 700, 800, -9.5, -7.45},
            {49, 19, -1, 10, 40, 500, 4.5, -4.45},
            {50, 20, -1, 10, 40, 160, -6.25, -3.45},
            {51, 21, -1, 5, 0, 1400, -5, 4.8},
            {52, 22, -1, 5, 0, 1400, 6, 2.9},
            {53, 23, -1, 5, 0, 1400, -8.4, 5.6},
            {54, 24, -1, 5, 0, 1400, 5.091, -0.02},
            {55, 25, -1, 5, 0, 1400, 4.5, 5.1},
            {56, 26, -1, 5, 0, 1400, -4.987, 4.8},
            {57, 27, -1, 5, 0, 1400, -1.789, 1.9},
            {58, 28, -1, 5, 0, 1400, -0.4, 9.6},
            {59, 29, -1, 5, 0, 1400, -5, 5.7},
            {60, 30, -1, 5, 0, 1400, 2, 9.1},
            {61, 0, 0, 0, 0, 1400, 0, 0}
    };*/

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

    const int max_iterations = 10000;
    const int max_vehicle_load = 6;
    const int planning_horizon = 1000;
    const int max_ride_time = 90;
    const double penalty_lambda = 0.015;
    const int num_vehicles = 4;
    const double tabu_duration_theta = 7.5 * std::log10(num_vehicles * (nodes.size() - 1));

    DARProblem problem{
        nodes, num_vehicles, (uint16_t)((nodes.size() / 2) - 1), max_vehicle_load, max_ride_time,
        planning_horizon
    };

    const std::string filename{ "C:\\Users\\nicki\\source\\repos\\nickives\\tabu\\tabu\\data\\tabu\\pr01" };

    DARProblem from_file = ProblemReader::read(filename);

    //TabuSearch ts(max_vehicle_load, planning_horizon, tabu_duration_theta, penalty_lambda, problem);

    const double from_file_theta = 7.5 * std::log10(from_file.number_of_requests * from_file.number_of_vehicles);

    TabuSearch ts(
        from_file.vehicle_capacity,
        from_file.planning_horizon,
        from_file_theta,
        penalty_lambda,
        from_file
    );
    
    try {
        auto start_time = std::chrono::steady_clock::now();
        auto best_solution = ts.search(max_iterations);
        auto end_time = std::chrono::steady_clock::now();
        auto time_diff = end_time - start_time;
        std::cout << "Best solution cost: " << best_solution.cost << std::endl;
        SolutionPrinter::print_solution(best_solution.solution);
        std::cout << "Running time: " << time_diff << std::endl;
    }
    catch (const std::bad_alloc& e) {
        std::cout << e.what() << std::endl;
    }
    
    return 0;
}