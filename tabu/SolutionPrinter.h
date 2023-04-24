#pragma once
//
// Created by Nicholas Ives on 23/04/2023.
//

#ifndef TABU_SOLUTIONPRINTER_H
#define TABU_SOLUTIONPRINTER_H

#include <iostream>

#include "tabutypes.h"

class SolutionPrinter {
public:
    static void
        print_solution(const Solution& solution);
};


#endif //TABU_SOLUTIONPRINTER_H
