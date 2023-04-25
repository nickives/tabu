// tabu.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

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

int main() {
    std::vector<Node> nodes = {
            {0, 0, 0, 0, 0, 1400, 0, 0},
            {1, 1, 1, 10, 0, 1400, 9, 8},
            {2, 2, 1, 5, 0, 1400, 6, 7},
            {3, 3, 1, 10, 0, 1400, 7, 9},
            {4, 4, 1, 5, 0, 1400, 3, 5},
            {5, 5, 1, 5, 0, 1400, -2, 5},
            {6, 6, 1, 5, 0, 1400, 5, 4.8},
            {7, 7, 1, 5, 0, 1400, 2, 5.9},
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
            {20, 20, 1, 5, 0, 1400, 7.5, 9.1},
            {21, 1, -1, 5, 40, 100, -1, -6},
            {22, 2, -1, 5, 20, 160, 3, -5.5},
            {23, 3, -1, 5, 40, 300, -1, -7.32},
            {24, 4, -1, 10, 40, 160, -9.5, -2.9},
            {25, 5, -1, 10, 45, 200, -6.5, 2.5},
            {26, 6, -1, 5, 40, 100, -1, 6},
            {27, 7, -1, 5, 20, 160, 3, -5.5},
            {28, 8, -1, 5, 40, 300, -8, -7.32},
            {29, 9, -1, 10, 40, 160, -8.5, -2.9},
            {30, 10, -1, 10, 45, 200, -6.5, -2.5},
            {31, 11, -1, 5, 40, 100, -1, 5.902},
            {32, 12, -1, 5, 20, 160, 3.765, -5.5},
            {33, 13, -1, 5, 40, 300, 8, 7.32},
            {34, 14, -1, 10, 40, 160, 8.5, 2.236},
            {35, 15, -1, 10, 45, 200, 6.5, -2.5},
            {36, 16, -1, 10, 100, 160, -6.3, -3.45},
            {37, 17, -1, 10, 200, 500, -2.5, -3},
            {38, 18, -1, 10, 40, 160, -9.5, -7.45},
            {39, 19, -1, 10, 40, 160, -4.5, -4.45},
            {40, 20, -1, 10, 40, 160, -6.25, -3.45},
            {41, 0, 0, 0, 0, 1400, 0, 0}
    };

    const int max_iterations = 500;
    const int max_solution_cost = 1000;
    const int max_vehicle_load = 6;
    const int planning_horizon = 1400;
    const int max_ride_time = 1000;
    const int tabu_list_size = 50000000;
    const uint64_t penalty_lambda = 2;
    const uint64_t tabu_duration = 25;
    const int num_vehicles = 8;

    DARProblem problem{
        nodes, num_vehicles, (uint16_t)((nodes.size() / 2) - 1), max_vehicle_load, max_ride_time,
        planning_horizon
    };

    TabuSearch ts(max_solution_cost, max_vehicle_load, planning_horizon, tabu_list_size, tabu_duration, penalty_lambda, problem);

    try {
        auto best_solution = ts.search(max_iterations);
        std::cout << "Best solution cost: " << best_solution.cost << std::endl;
        SolutionPrinter::print_solution(best_solution.solution);
    }
    catch (const std::bad_alloc& e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}