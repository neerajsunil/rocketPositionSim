import numpy as np
import matplotlib.pyplot as plt

# Input Rocket Characteristics
rocket_mass = 50.0  # kg
gravity = 9.81  # m/s^2
rocket_drag_coeff = 0.5
rocket_area = 0.1 
constant_thrust = 1500

# Simulated Load Cell Readings (Replace with sensor readings)
def simulate_load_cells():
    # Simulate a rocket launch with increasing thrust and some lateral movement
    Fx = np.random.uniform(-15, 15)
    Fy = np.random.uniform(-15, 15)
    Fz = -rocket_mass * gravity + constant_thrust + np.random.uniform(-1, 1) * 10
    return Fx, Fy, Fz

# Thrust and TVC Calculation with corrections
def calculate_thrust_tvc(Fx, Fy, Fz):
    # Simulate TVC adjustments to counteract wind
    Fx += np.random.randn() * 20 # Random disturbances in x
    Fy += np.random.randn() * 20 # Random disturbances in y
    
    thrust = np.sqrt(Fx**2 + Fy**2 + Fz**2)
    thrust_direction = np.array([Fx, Fy, Fz]) / thrust  # Normalized thrust vector
    return thrust, thrust_direction

# Equations of Motion (Simplified - no rotation, basic drag)
def update_state(x, y, z, vx, vy, vz, thrust, thrust_direction, dt):
    acceleration = thrust * thrust_direction / rocket_mass 
    acceleration[2] -= gravity 
    
    #Simplified drag - always opposing velocity
    velocity = np.array([vx, vy, vz])
    drag_force = -0.5 * rocket_drag_coeff * rocket_area * np.linalg.norm(velocity) * velocity
    acceleration += drag_force / rocket_mass
    
    # Euler integration
    vx += acceleration[0] * dt
    vy += acceleration[1] * dt
    vz += acceleration[2] * dt
    x += vx * dt
    y += vy * dt
    z += vz * dt
    return x, y, z, vx, vy, vz

# Main Simulation Loop
dt = 0.01  # Time step
x, y, z = 0, 0, 0
vx, vy, vz = 0, 0, 0

positions = []
for time_step in range(1000):
    Fx, Fy, Fz = simulate_load_cells()
    thrust, thrust_direction = calculate_thrust_tvc(Fx, Fy, Fz)
    x, y, z, vx, vy, vz = update_state(x, y, z, vx, vy, vz, thrust, thrust_direction, dt)
    positions.append([x,y,z])

# Visualize in 3D
positions = np.array(positions)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:,0], positions[:,1], positions[:,2])
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Rocket Trajectory")
plt.show()