//
// Created by Nicholas Ives on 20/04/2023.
//

#include "InvalidSolutionException.h"
#include "tabutypes.h"

const Solution InvalidSolutionException::getSolution() const {
    return solution_;
}

const char* InvalidSolutionException::what() const noexcept {
    return logic_error::what();
}
