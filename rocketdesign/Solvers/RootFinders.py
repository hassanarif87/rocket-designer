from pint import UnitRegistry
import numpy as np
ureg = UnitRegistry()

def newton_raphson(f, df, x0, tol=1e-6, max_iter=100):
    """
    Newton-Raphson method for finding roots of a function.

    Parameters:
    - f: The function for which to find the root.
    - df: The derivative of the function.
    - x0: Initial guess for the root.
    - tol: Tolerance, the iteration stops when abs(f(x)) < tol.
    - max_iter: Maximum number of iterations.

    Returns:
    - Approximate root.
    - Number of iterations performed.
    """

    x = x0
    for iteration in range(max_iter):
        f_value = f(x)
        if abs(f_value) < tol:
            return x, iteration
        df_value = df(x)

        # Avoid division by zero
        if df_value == 0:
            raise ValueError("Derivative is zero. Newton-Raphson method cannot converge.")

        x = x - f_value / df_value

    raise RuntimeError("Newton-Raphson method did not converge within the maximum number of iterations.")

def secant_method(f, x0, x1, tol=1e-6, max_iter=100):
    """
    Secant method for finding roots of a function.

    Parameters:
    - f: The function for which to find the root.
    - x0: Initial guess for the root.
    - x1: Another initial guess for the root (different from x0).
    - tol: Tolerance, the iteration stops when abs(f(x)) < tol.
    - max_iter: Maximum number of iterations.

    Returns:
    - Approximate root.
    - Number of iterations performed.
    """

    for iteration in range(max_iter):
        f_x0 = f(x0)
        f_x1 = f(x1)

        # Avoid division by zero
        if f_x1 - f_x0 == 0:
            raise ValueError("Division by zero. Secant method cannot converge.")

        x_next = x1 - f_x1 * (x1 - x0) / (f_x1 - f_x0)

        if abs(f(x_next)) < tol:
            return x_next, iteration

        x0, x1 = x1, x_next

    raise RuntimeError("Secant method did not converge within the maximum number of iterations.")

# Example usage:
if __name__ == "__main__":

    # Time of flight of a ball with valocity v0 and launch angle theta

    v0 = 10 # m/s 
    g = 9.81 #m/s local gravity
    theta_deg = 60 * ureg.degree
    theta = theta_deg.to(ureg.rad)
    print(theta)
    # Define the function and its derivative
    def f(x):
        return v0 *np.sin(theta) *x - 1/2*g*x**2

    def df(x):
        return v0 *np.sin(theta) - g*x

    # Initial guess
    initial_guess = 1000
    initial_guess2 = 999

    # Apply Newton-Raphson method
    root, iterations = newton_raphson(f, df, initial_guess)

    # Print results
    print(f"newton_raphson \n Approximate root: {root}")
    print(f"Iterations: {iterations}")
    root, iterations = secant_method(f, initial_guess, initial_guess2)

    print(f" secant_method \n Approximate root: {root}")
    print(f"Iterations: {iterations}")