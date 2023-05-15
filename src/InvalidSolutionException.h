#pragma once
//
// Created by Nicholas Ives on 20/04/2023.
//

#ifndef TABU_INVALIDSOLUTIONEXCEPTION_H
#define TABU_INVALIDSOLUTIONEXCEPTION_H

#include <stdexcept>
#include <utility>

#include "tabutypes.h"

class InvalidSolutionException : std::logic_error {
public:
    InvalidSolutionException(const std::string& string, Solution solution)
        : logic_error(string), solution_(std::move(solution)) {};
    InvalidSolutionException(const char* string, Solution solution)
        : logic_error(string), solution_(std::move(solution)) {};
    InvalidSolutionException(const logic_error& error, Solution solution)
        : logic_error(error), solution_(std::move(solution)) {};

    [[nodiscard]] const Solution getSolution() const;

public:
    [[nodiscard]] const char* what() const noexcept override;

private:
    const Solution solution_;
};


#endif //TABU_INVALIDSOLUTIONEXCEPTION_H
