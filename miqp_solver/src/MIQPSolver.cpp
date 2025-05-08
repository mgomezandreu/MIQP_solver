#include <MIQPDefinition.hpp>
#include <MIQPSolver.hpp>
#include <Utils.hpp>    

namespace miqp_solver {
    MIQPSolver::MIQPSolver(const MIQPDefinition& miqp_definition) : miqp_definition_(miqp_definition) {
        // Initialize the solver with the MIQP definition
        solver_.settings()->setVerbosity(false);
        solver_.data()->setNumberOfVariables(miqp_definition_.getQ().rows());
        solver_.data()->setNumberOfConstraints(miqp_definition_.getQ().rows());
        const Eigen::SparseMatrix<double>& Q = miqp_definition_.getQ();
        solver_.data()->setHessianMatrix(miqp_definition_.getQ());
        Eigen::VectorXd c = miqp_definition_.getc();
        solver_.data()->setGradient(c); 


        // stack the constraints
        Eigen::SparseMatrix<double> A = miqp_definition_.getA();
        Eigen::VectorXd b = miqp_definition_.getb();
        Eigen::VectorXd lb = miqp_definition_.getlb();
        Eigen::VectorXd ub = miqp_definition_.getub();

        int n = Q.rows();

        Eigen::SparseMatrix<double> A_aug(n, n);    
        A_aug.setIdentity(); // Initialize A_aug as an identity matrix of size n x n

        // Assign the original A matrix to the bottom rows of A_aug
        // for (int k = 0; k < A.outerSize(); ++k) {
        //     for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
        //         A_aug.insert(it.row() + n, it.col()) = it.value();
        //     }
        // }

        //convert A_aug to dense
        Eigen::MatrixXd A_dense = Eigen::MatrixXd(A_aug);

        // Eigen::VectorXd ub_combined(b.size() + n);
        // Eigen::VectorXd lb_combined(b.size() + n);

        // ub_combined.head(n) = ub;
        // lb_combined.head(n) = lb;
        // ub_combined.tail(m) = b;
        // lb_combined.tail(m) = Eigen::VectorXd::Constant(m, -std::numeric_limits<double>::infinity());
        

        solver_.data()->setLinearConstraintsMatrix(A_aug);
        solver_.data()->setLowerBound(lb);
        solver_.data()->setUpperBound(ub);

        solver_.initSolver();  
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(miqp_definition_.getQ().rows());
        solver_.setPrimalVariable(x0);  
    }

    std::optional<MIQPNode> MIQPSolver::createNode(Eigen::VectorXd lb, Eigen::VectorXd ub) {
        // Create a new MIQPNode with the given lower and upper bounds
        // Eigen::VectorXd lb_combined(miqp_definition_.getlb().size() + miqp_definition_.getb().size());
        // Eigen::VectorXd ub_combined(miqp_definition_.getub().size() + miqp_definition_.getb().size());

        // lb_combined.head(lb.size()) = lb;
        // ub_combined.head(ub.size()) = ub;
        // lb_combined.tail(miqp_definition_.getb().size()) = Eigen::VectorXd::Constant(miqp_definition_.getb().size(), -10000);
        // ub_combined.tail(miqp_definition_.getb().size()) = miqp_definition_.getb();
        
        // solver_.data()->setLowerBound(lb);
        // solver_.data()->setUpperBound(ub);
        
        //check if the bounds are feasible
        for(int i = 0; i < lb.size(); ++i) {
            if (lb(i) > ub(i)) {
                return std::nullopt; // Node is infeasible
            }
        }

        solver_.updateBounds(lb, ub);
        solver_.solveProblem();

        // std::cout << lb_combined <<std::endl;
        // std::cout << ub_combined << std::endl;


        // Check if the problem is feasible
        auto status = solver_.getStatus();  // returns OsqpEigen::Status
        
        /*
          DualInfeasibleInaccurate = OSQP_DUAL_INFEASIBLE_INACCURATE,
    PrimalInfeasibleInaccurate = OSQP_PRIMAL_INFEASIBLE_INACCURATE,
    SolvedInaccurate = OSQP_SOLVED_INACCURATE,
    Solved = OSQP_SOLVED,
    MaxIterReached = OSQP_MAX_ITER_REACHED,
    PrimalInfeasible = OSQP_PRIMAL_INFEASIBLE,
    DualInfeasible = OSQP_DUAL_INFEASIBLE,
    Sigint = OSQP_SIGINT,
        */

        // switch(status) {
        //     case OsqpEigen::Status::DualInfeasibleInaccurate:
        //          std::cout << "Dual Infeasible Inaccurate" << std::endl;
        //         break;
        //     case OsqpEigen::Status::PrimalInfeasibleInaccurate:
        //         std::cout << "Primal Infeasible Inaccurate" << std::endl;
        //         break;
        //     case OsqpEigen::Status::SolvedInaccurate:
        //         std::cout << "Solved Inaccurate" << std::endl;
        //         break;
        //     case OsqpEigen::Status::Solved:
        //         std::cout << "Solved" << std::endl;
        //         break;
        //     case OsqpEigen::Status::MaxIterReached:
        //         std::cout << "Max Iterations Reached" << std::endl;
        //         break;
        //     case OsqpEigen::Status::Sigint:
        //         std::cout << "SIGINT" << std::endl;
        //         break;
        //     default:
        //         break;
        // }

        if(!(status == OsqpEigen::Status::Solved || status == OsqpEigen::Status::SolvedInaccurate)) {
            return std::nullopt; // Node is infeasible
        }
      

        // Get the solution
        Eigen::VectorXd sol = solver_.getSolution();
        double obj = solver_.getObjValue();

        MIQPNode node(lb, ub, 0);
        node.setSol(sol, obj);

        return node;
    }

    bool MIQPSolver::isIntegral(const Eigen::VectorXd& x) {
        for (int i : miqp_definition_.getIntList()) {
            if (!isInteger(x(i))) {
                return false;
            }
        }
        return true;
    }

    bool MIQPSolver::solve(Eigen::VectorXd& solution, double& objective_value) {
        std::optional<MIQPNode> node = createNode(miqp_definition_.getlb(), miqp_definition_.getub());
        if (!node) {
            std::cerr << "Relaxed total problem infeasible" << std::endl;
            return false;
        }

        Eigen::VectorXd best_solution;
        double best_objective_value = std::numeric_limits<double>::infinity();
        double lower_bound = std::numeric_limits<double>::lowest(); 

        std::vector<MIQPNode> queue;
        queue.push_back(*node);

        int nodes_explored = 0; 
        
        while (!queue.empty()) {
            MIQPNode current_node = queue.back();
            queue.pop_back();

            // Check if the current node is integral
            if (isIntegral(current_node.getSol())) {
                double obj = current_node.getObj();
                if (obj < best_objective_value) {
                    best_objective_value = obj;
                    best_solution = current_node.getSol();
                }
               
                if(isClose(best_objective_value, lower_bound)) {
                    solution = best_solution;
                    objective_value = best_objective_value;
                    return true;
                }

                //prune the queue
                queue.erase(std::remove_if(queue.begin(), queue.end(), [best_objective_value](MIQPNode x) {
                    return x.getObj() > best_objective_value;
                }), queue.end());
            }

            for(int index: miqp_definition_.getIntList()) {
                if(isInteger(current_node.getSol()(index))) 
                    continue;   


                Eigen::VectorXd lb = current_node.getLb();
                Eigen::VectorXd ub = current_node.getUb();

                // Create two new nodes by branching on the integer variable
                double value = current_node.getSol()(index);
                Eigen::VectorXd left_lb = lb;
                Eigen::VectorXd left_ub = ub;
                left_ub(index) = floor(value);

                Eigen::VectorXd right_lb = lb;
                Eigen::VectorXd right_ub = ub;
                right_lb(index) = ceil(value);

            
                std::optional<MIQPNode> left_child = createNode(left_lb, left_ub);
                if (left_child) {
                    queue.push_back(*left_child);
                }
                std::optional<MIQPNode> right_child = createNode(right_lb, right_ub);
                if (right_child) {
                    queue.push_back(*right_child);
                }
                break; // Only branch on one variable at a time
            } 

            if(queue.empty()) {
                // No more nodes to explore
                break;
            }

            // find current lower bound
            std::sort(queue.begin(), queue.end(), [](const MIQPNode& a, const MIQPNode& b) {
                return a.getObj() < b.getObj();
            });
            lower_bound = queue.front().getObj();
        }

        // if we found a solution, return it
        if (best_objective_value < std::numeric_limits<double>::infinity()) {
            solution = best_solution;
            objective_value = best_objective_value;
            return true;
        } else {
            std::cerr << "No feasible solution found" << std::endl;
            return false;
        }


    }


};