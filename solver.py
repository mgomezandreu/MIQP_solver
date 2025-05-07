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
        def __init__(self, lb,ub, parent=None):
            self.lb = lb
            self.ub = ub
            self.cost = None
            self.sol = None

        def __str__(self):
            return f"MIQPSolverNode(definition={self.definition}, parent={self.parent})"
        
        def set_solution(self, x,cost):
            self.sol = x
            self.cost = cost
        
    def create_node(self, lb, ub):
        # Create a new node with the given bounds
        node = self.MIQPSolverNode(lb, ub)
        self.ub.value = ub
        self.lb.value = lb

        self.problem.solve()
        if self.problem.status == cp.OPTIMAL or self.problem.status == cp.OPTIMAL_INACCURATE:
            node.set_solution(self.x.value, self.problem.value)
            return True, node
        else:
            print(f"Problem not solved optimally: {self.problem.status}")
            return False, None



    def __init__(self, definition):
        self.definition = definition

        self.lb = cp.Parameter(definition.c.shape[0], name="lb")
        self.ub = cp.Parameter(definition.c.shape[0], name="ub")

        self.best_solution = None
        self.best_objective = None

        self.x = cp.Variable(definition.c.shape[0], name="x")
        objective = cp.Minimize(cp.quad_form(self.x, definition.Q) + definition.c @ self.x)
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
            if not np.isclose(x[i], round(x[i]), atol=1e-3):
                return False
        return True
    

    def solve(self):
        success, node = self.create_node(self.definition.lb, self.definition.ub)   
        if not success:
            return False, None, None

        self.open_nodes.append(node)
        checked_nodes_num = 0
        self.lower_bound = node.cost

        while self.open_nodes:
            checked_nodes_num += 1
            print(f"Checked nodes: {checked_nodes_num}, Open nodes: {len(self.open_nodes)}")    

            node = self.open_nodes.pop(0)
            if self._is_integral(node.sol):
                # If the node is integral, check if it is better than the best solution
                if self.best_solution is None or node.cost < self.upper_bound:
                    self.best_solution = node.sol.copy()
                    self.upper_bound = node.cost
                    print(f"Found integral solution: {node.sol}, objective value: {node.cost}")


                    if np.isclose(self.lower_bound, self.upper_bound, atol=1e-6):
                        print("Optimal solution found, stopping search. ==============")
                        return True, self.best_solution, self.upper_bound
                
                    #prune if the cost is greater than the upper bound
                    i = 0
                    while i < len(self.open_nodes):
                        if self.open_nodes[i].cost >= self.upper_bound:
                            print(f"Pruning node with cost {self.open_nodes[i].cost} >= upper bound {self.upper_bound}")
                            self.open_nodes.pop(i)
                        else:
                            i += 1
            
        
            # Branching on all integral variables
            for i in self.definition.int_set:
                if not np.isclose(node.sol[i], round(node.sol[i]), atol=1e-3):
                    print(f"Branching on variable {i}: {node.sol[i]} not integral")
                # Create two new nodes for the branching
                    left_lb = node.lb.copy()    
                    left_ub = node.ub.copy()
                    left_ub[i] = np.floor(node.sol[i])

                    right_lb = node.lb.copy()
                    right_lb[i] = np.ceil(node.sol[i])
                    right_ub = node.ub.copy()
                
                    # print(f"Branching on variable {i}: left = {left_definition.ub[i]}, right = {right_definition.lb[i]}")

                    left_success, left_node = self.create_node(left_lb, left_ub)
                    right_success, right_node = self.create_node(right_lb, right_ub)

                    if left_success:
                        self.open_nodes.append(left_node)
                        print(f"Left node added with bounds {left_node.lb} and {left_node.ub}")
                    if right_success:
                        self.open_nodes.append(right_node)
                        print(f"Right node added with bounds {right_node.lb} and {right_node.ub}")
                    break

            if self.open_nodes == []:
                print("No more nodes to explore, stopping search.")
                break 
            self.lower_bound = min(node.cost for node in self.open_nodes)
        return self.best_solution is not None, self.best_solution, self.upper_bound

            

            
