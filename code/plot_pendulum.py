import matplotlib
import argparse
from matplotlib import pyplot as plt
import os 
import sys
from matplotlib import patches
import math

def plot_pendulum_solution(filename,name):
    theta = []
    omega = []

    with open(filename, "r") as file:
        for line in file:
            values = line.strip().split()
            if len(values) >= 2:
                theta.append(float(values[0]))  # First column is theta
                omega.append(float(values[1]))  # Second column is omega

    plt.plot(theta, omega, marker='o', label="Planner Result")
    plt.title("Pendulum Motion: Theta vs Omega for Different Torque Limits")
    plt.xlabel("Theta (angle)")
    plt.ylabel("Omega (angular velocity)")
    plt.legend()
    plt.grid(True)
    plt.savefig(name)
    plt.show()



def main():
    
    parser = argparse.ArgumentParser(description='Visualize the path of a robot in an environment with obstacles.')
    parser.add_argument('--path', type=str, default='example_path.txt', help='Name of the path file')
    parser.add_argument('--name', type=str, default='example_path', help='Name to assign to the file') 
    args = parser.parse_args()

    print("***" * 19 + "\n Visualising the environment and path for generated data\n" + "***" * 19)
    

    plot_pendulum_solution(args.path,args.name)

if __name__ == "__main__":
    main()
