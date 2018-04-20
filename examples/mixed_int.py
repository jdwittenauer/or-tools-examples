# https://developers.google.com/optimization/mip/integer_opt
from ortools.linear_solver import pywraplp


def main():
    # Instantiate a mixed-integer solver, naming it SolveIntegerProblem.
    solver = pywraplp.Solver('Mixed Integer Example', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    # x and y are integer non-negative variables.
    x = solver.IntVar(0.0, solver.infinity(), 'x')
    y = solver.IntVar(0.0, solver.infinity(), 'y')

    # x + 7 * y <= 17.5
    constraint1 = solver.Constraint(-solver.infinity(), 17.5)
    constraint1.SetCoefficient(x, 1)
    constraint1.SetCoefficient(y, 7)

    # x <= 3.5
    constraint2 = solver.Constraint(-solver.infinity(), 3.5)
    constraint2.SetCoefficient(x, 1)
    constraint2.SetCoefficient(y, 0)

    # Maximize x + 10 * y.
    objective = solver.Objective()
    objective.SetCoefficient(x, 1)
    objective.SetCoefficient(y, 10)
    objective.SetMaximization()

    result_status = solver.Solve()

    # The problem has an optimal solution.
    assert result_status == pywraplp.Solver.OPTIMAL

    # The solution looks legit (when using solvers other than
    # GLOP_LINEAR_PROGRAMMING, verifying the solution is highly recommended!).
    assert solver.VerifySolution(1e-7, True)

    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())
    print('Optimal objective value = %d' % solver.Objective().Value())
    print()

    # The value of each variable in the solution.
    variable_list = [x, y]
    for variable in variable_list:
        print('%s = %d' % (variable.name(), variable.solution_value()))


if __name__ == '__main__':
    main()
