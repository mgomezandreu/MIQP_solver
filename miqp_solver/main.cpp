#include <MIQPSolver.hpp>   
#include <Utils.hpp>

int main() {
    // Define the MIQP problem
    Eigen::SparseMatrix<double> Q(2, 2);
    Q.insert(0, 0) = 1.0;
    Q.insert(1, 1) = 1.0;
    Eigen::VectorXd c(2);
    c(0) = 1.2;
    c(1) = -0.9;
    Eigen::SparseMatrix<double> A(1, 2);
    A.insert(0, 0) = -1.0;
    A.insert(0, 1) = 1.0;
    Eigen::VectorXd b(1);
    b(0) = -1.0;
    Eigen::VectorXd lb(2);
    lb(0) = -5.0;
    lb(1) = -5.0;
    Eigen::VectorXd ub(2);
    ub(0) = 5.0;
    ub(1) = 5.0;

    std::vector<int> int_list = {0};  // Index of the integer variable
    miqp_solver::MIQPDefinition miqp_definition(Q, c, A, b, lb, ub, int_list);
    miqp_solver::MIQPSolver solver(miqp_definition);
    Eigen::VectorXd solution;
    double objective_value;

    if (solver.solve(solution, objective_value)) {
        std::cout << "Optimal solution found!" << std::endl;
        std::cout << "Solution: " << solution.transpose() << std::endl;
        std::cout << "Objective value: " << objective_value << std::endl;
    } else {
        std::cout << "No optimal solution found." << std::endl;
        return 1;
    }

    // Check if the solution is integral
    for (int i : int_list) {
        if (!miqp_solver::isInteger(solution(i))) {
            std::cout << "Solution is not integral at index " << i << ": " << solution(i) << std::endl;
        } else {
            std::cout << "Solution is integral at index " << i << ": " << solution(i) << std::endl;
        }
    }

    // Check if the solution is within bounds
    for (int i = 0; i < solution.size(); ++i) {
        if (solution(i) < lb(i) || solution(i) > ub(i)) {
            std::cout << "Solution is out of bounds at index " << i << ": " << solution(i) << std::endl;
        } else {
            std::cout << "Solution is within bounds at index " << i << ": " << solution(i) << std::endl;
        }
    }




    return 0;
}
