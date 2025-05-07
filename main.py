from solver import MIQPDefinition, MIQPSolver
import numpy as np


Q = np.array([[1]])
c = np.array([-2.4])
A = np.array([[1],[-1]])
b = np.array([5, 5])
lb = np.array([-5])
ub = np.array([5])
int_set = [0]
milp = MIQPDefinition("test", Q,c, A, b, lb, ub, int_set)
solver = MIQPSolver(milp)
success, x = solver.solve()

print("Success:", success)
print("Solution:", x)


