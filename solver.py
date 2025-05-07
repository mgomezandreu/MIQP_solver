import numpy as np
import cvxpy as cp

"""
min c^T x
s.t. Ax <= b
lb <= x <= ubx
x_i integer for i in int_set
"""
class MIQPDefinition:
    def __init__(self, name,Q: np.array,  c:np.array, A: np.array, b: np.array, lb: np.array, ub: np.array, int_set: list[int]):
        assert Q.shape[0] == Q.shape[1], "Q must be a square matrix"
        assert Q.shape[0] == c.shape[0], "Q and c dimensions do not match"
        assert c.shape[0] == A.shape[1], "c and A dimensions do not match"
        assert A.shape[0] == b.shape[0], "A and b dimensions do not match"
        assert lb.shape[0] == ub.shape[0], "lb and ub dimensions do not match"
        assert c.shape[0] == lb.shape[0], "c and lb dimensions do not match"
        assert len(int_set) <= c.shape[0], "int_set cannot be larger than the number of variables"
        for i in int_set:
            assert 0 <= i < c.shape[0], f"Index {i} in int_set is out of bounds for c"
        
        self.name = name
        self.Q = Q
        self.c = c
        self.A = A
        self.b = b
        self.lb = lb
        self.ub = ub
        self.int_set = int_set

    def __str__(self):
        return f"MIQPDefinition(name={self.name}, c={self.c}, A={self.A}, b={self.b}, lb={self.lb}, ub={self.ub}, int_set={self.int_set})"

    # add copy method
    def copy(self):
        return MIQPDefinition(
            name=self.name,
            Q=self.Q.copy(),
            c=self.c.copy(),
            A=self.A.copy(),
            b=self.b.copy(),
            lb=self.lb.copy(),
            ub=self.ub.copy(),
            int_set=self.int_set.copy()
        )


class MIQPSolver:

    class MIQPSolverNode: 
        def __init__(self, definition, parent=None):
            self.definition = definition
            self.parent = parent
            self.children = []

        def __str__(self):
            return f"MIQPSolverNode(definition={self.definition}, parent={self.parent})"
        


    def __init__(self, definition, qp_solver=None):
        self.definition = definition

        self.lb = cp.Parameter(definition.c.shape[0], name="lb")
        self.ub = cp.Parameter(definition.c.shape[0], name="ub")


        self.x = cp.Variable(definition.c.shape[0], name="x")
        objective = cp.Minimize(cp.quad_form(self.x, definition.Q) + cp.sum(cp.multiply(definition.c, self.x)))
        constraints = [
            definition.A @ self.x <= definition.b,
            self.x >= self.lb,
            self.x <= self.ub
        ]
        self.problem = cp.Problem(objective, constraints)

        self.lower_bound = -np.inf
        self.upper_bound = np.inf
        self.open_nodes = []

        

    def _is_integral(self, x):
        for i in self.definition.int_set:
            if not np.isclose(x[i], round(x[i])):
                return False
        return True
    
    def _solve_relaxed(self, lb:np.array, ub:np.array):
        # Solve the relaxed problem (ignoring integer constraints)
        self.lb.value = lb
        self.ub.value = ub

        print(f"Solving relaxed problem with bounds: {lb}, {ub}")

        self.problem.solve()
        
        if self.problem.status == cp.OPTIMAL or self.problem.status == cp.OPTIMAL_INACCURATE:
            return True, self.x.value, self.problem.value   
        else:
            return False, None, None
        
    

    def solve(self):
        start_node = self.MIQPSolverNode(self.definition)
        self.open_nodes.append(start_node)

        while self.open_nodes:
            current_node = self.open_nodes.pop(0)
            print(f"Current node: {current_node}")

            success, x, cost = self._solve_relaxed(current_node.definition.lb, current_node.definition.ub)
            if not success:
                continue
        
            # If the solution is infeasible, skip this node, no further branching will make it feasible
            if np.dot(current_node.definition.c, x) > self.upper_bound:
                continue
            
            # Update the lower bound if the solution is feasible, making it tighter if possible
            self.lower_bound = max(self.lower_bound, cost)

            # If a integral solution is found this bounds the problem form above
            if self._is_integral(x):
                self.upper_bound = min(self.upper_bound, cost)
                print(f"Found integral solution: {x}, objective value: {cost}")
    

            # When the bounds are equal, we have found an optimal solution
            print(f"Current bounds: lower = {self.lower_bound}, upper = {self.upper_bound}")
            if np.isclose(self.lower_bound, self.upper_bound):
                print("Optimal solution found")
                return True, np.round(x)


            # Branching on all integral variables
            for i in current_node.definition.int_set:
                # Create two new nodes for the branching
                left_definition = current_node.definition.copy()
                right_definition = current_node.definition.copy()

                # Update bounds for left and right nodes
                left_definition.ub[i] = np.floor(x[i])
                right_definition.lb[i] = np.ceil(x[i])

                print(f"Branching on variable {i}: left = {left_definition.ub[i]}, right = {right_definition.lb[i]}")

                # Create new nodes
                left_node = self.MIQPSolverNode(left_definition, parent=current_node)
                right_node = self.MIQPSolverNode(right_definition, parent=current_node)

                # Add to open nodes
                self.open_nodes.append(left_node)
                self.open_nodes.append(right_node)


        # If we exhaust all nodes and do not find a solution, return False
        print("No solution found")
        return False, None

            

            
