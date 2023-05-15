#include "SolutionFileWriter.h"

void SolutionFileWriter::addSolution(const SolutionResult& solution_result)
{
    const auto& routes = solution_result.solution.routes;
    for (size_t r = 0; r < routes.size(); ++r) {
        file << "Vehicle " << r + 1 << ":"; 
        for (const auto& node : routes[r].nodes) {
            file << " " << std::to_string(node->id);
        }
        file << " : Duration : " << std::to_string(routes[r].route_excess.duration) << std::endl;
    }
    file << "Total Cost: " << solution_result.cost << endl << endl;
}

void SolutionFileWriter::addRunningTime(const std::chrono::milliseconds& time_in_ms)
{
    file << "Running time (ms): " << time_in_ms.count() << endl;
}



void SolutionFileWriter::close()
{
    file.close();
}

bool SolutionFileWriter::is_open()
{
    return file.is_open();
}
