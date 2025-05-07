import unittest
import numpy as np
import cvxpy as cp
from solver import MIQPDefinition, MIQPSolver  # Adjust import as needed


def generate_random_miqp(n_vars, n_constraints, n_int):
    Q = np.random.randn(n_vars, n_vars)
    Q = Q @ Q.T  # Make it positive semi-definite
    c = np.random.randn(n_vars)* 3
    A = np.random.randn(n_constraints, n_vars)
    b = np.random.randn(n_constraints)
    lb = -4 * np.ones(n_vars)
    ub = 4 * np.ones(n_vars)
    int_set = list(np.random.choice(n_vars, n_int, replace=False))
    return MIQPDefinition("test", Q, c, A, b, lb, ub, int_set)


def solve_with_mosek(defn: MIQPDefinition):
    n = len(defn.c)
    x_vars = []

    for i in range(n):
        is_int = i in defn.int_set
        var = cp.Variable(integer=is_int, name=f"x_{i}")
        x_vars.append(var)

    x = cp.hstack(x_vars)

    objective = cp.Minimize(cp.quad_form(x, defn.Q) + defn.c @ x)
    constraints = [
        defn.A @ x <= defn.b,
        x >= defn.lb,
        x <= defn.ub
    ]

    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.MOSEK)

    if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        x_val = np.array([v.value for v in x_vars])
        return True, x_val, prob.value
    else:
        return False, None, None


class TestMIQPSolverWithMosek(unittest.TestCase):

    def test_random_miqp(self):
        np.random.seed(0)  # For reproducibility
        for _ in range(100):  # Run multiple test cases
            defn = generate_random_miqp(n_vars=10, n_constraints=1, n_int=4)
            miqp_solver = MIQPSolver(defn)
            found, x_miqp, obj_miqp = miqp_solver.solve()

            found_ref, x_ref, obj_ref = solve_with_mosek(defn)

            self.assertTrue(found == found_ref, "Solver found status does not match reference solver {found} {found_ref}")
            if not found:
                continue

            # Compute custom solver's objective
            print("MIQP Sol",x_miqp,"Obj", obj_miqp)
            print("Ref Sol",x_ref,"Obj", obj_ref)


            np.testing.assert_allclose(obj_miqp, obj_ref, rtol=1e-2, atol=1e-2)
            # np.testing.assert_allclose(
            #     x_miqp[defn.int_set],
            #     np.round(x_ref[defn.int_set]),
            #     atol=1e-1,
            #     err_msg="Integer parts of the solution do not match"
            # )

if __name__ == '__main__':
    unittest.main()
