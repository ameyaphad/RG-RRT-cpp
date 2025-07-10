import matplotlib.pyplot as plt

def plot_pendulum_solution(filename, label):
    theta = []
    omega = []

    with open(filename, "r") as file:
        for line in file:
            values = line.strip().split()
            if len(values) >= 2:
                theta.append(float(values[0]))  # First column is theta
                omega.append(float(values[1]))  # Second column is omega

    plt.plot(theta, omega, marker='o', label=label)

# Plot each solution for different torque limits
plot_pendulum_solution("pendulum_solution_path_RRT_torque_3.txt", "Torque 3")
plot_pendulum_solution("pendulum_solution_path_RRT_torque_5.txt", "Torque 5")
plot_pendulum_solution("pendulum_solution_path_RRT_torque_10.txt", "Torque 10")

plt.title("Pendulum Motion: Theta vs Omega for Different Torque Limits")
plt.xlabel("Theta (angle)")
plt.ylabel("Omega (angular velocity)")
plt.legend()
plt.grid(True)
plt.savefig("pendulum_comparison.png")
plt.show()
