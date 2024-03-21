import numpy as np
import matplotlib.pyplot as plt
import math

# Input Rocket Characteristics
rocket_mass = 50.0  # kg
gravity = 9.81  # m/s^2
rocket_drag_coeff = 0.5
rocket_area = 0.1 
constant_thrust = 1500
TVC_DEG_PER_SECOND_MAX = 300
ROCKET_PITCH_RANGE = 15 # Degrees
ROCKET_YAW_RANGE = 15 # Degrees
TVC_THRESHOLD = 0.1 # m/s

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
    Fx += np.random.randn() * 200 # Random disturbances in x
    Fy += np.random.randn() * 200 # Random disturbances in y
    
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

def calculate_pitch_yaw(vx, vy, vz):
    pitch_direction = math.copysign(1, vx)
    yaw_direction = math.copysign(1, vy)

    pitch = math.atan2(abs(vx), abs(vz)) * 180 / math.pi
    yaw = math.atan2(abs(vy), abs(vz)) * 180 / math.pi

    return pitch * pitch_direction, yaw * yaw_direction

def calculate_tvc_angle(force):
    return force/3.8

def calculate_tvc_force(angle):
    return angle * 3.8

def tvc(x, y, z, vx, vy, vz, thrust, thrust_direction, dt,vanePitch, vaneYaw):
    acceleration = thrust * thrust_direction / rocket_mass 
    acceleration[2] -= gravity 

    vxSign = math.copysign(1, vx)
    vySign = math.copysign(1, vy)

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

    if abs(vx) > TVC_THRESHOLD:
        pitchForceCorrection = rocket_mass * abs(vx) / dt
        maxPitch = vanePitch + TVC_DEG_PER_SECOND_MAX * dt
        maxPitchForce = calculate_tvc_force(maxPitch)
        if pitchForceCorrection > maxPitchForce:
            pitchForceCorrection = maxPitchForce

        vanePitch = calculate_tvc_angle(pitchForceCorrection)
        pitchAcc = pitchForceCorrection / rocket_mass
        vx += pitchAcc * dt * -vxSign
        x += vx * dt * -vxSign

    if abs(vy) > TVC_THRESHOLD:
        yawForceCorrection = rocket_mass * abs(vy) / dt
        maxYaw = vaneYaw + TVC_DEG_PER_SECOND_MAX * dt
        maxYawForce = calculate_tvc_force(maxYaw)
        if yawForceCorrection > maxYawForce:
            yawForceCorrection = maxYawForce

        vaneYaw = calculate_tvc_angle(yawForceCorrection)
        yawAcc = yawForceCorrection / rocket_mass
        vy += yawAcc * dt * -vySign
        y += vy * dt * -vySign
        
    return x, y, z, vx, vy, vz, vanePitch, vaneYaw

# Main Simulation Loop
dt = 0.01  # Time step
x, y, z = 0, 0, 0
vx, vy, vz = 0, 0, 0

xTVC, yTVC, zTVC = 0, 0, 0
vxTVC, vyTVC, vzTVC = 0, 0, 0
vanePitch, vaneYaw = 0, 0


positions = []
positionsTVC = []

for time_step in range(1000):
    Fx, Fy, Fz = simulate_load_cells()
    thrust, thrust_direction = calculate_thrust_tvc(Fx, Fy, Fz)
    x, y, z, vx, vy, vz = update_state(x, y, z, vx, vy, vz, thrust, thrust_direction, dt)
    positions.append([x,y,z])

    xTVC, yTVC, zTVC, vxTVC, vyTVC, vzTVC, vanePitch, vaneYaw = tvc(xTVC, yTVC, zTVC, vxTVC, vyTVC, vzTVC, thrust, thrust_direction, dt, vanePitch, vaneYaw)
    positionsTVC.append([xTVC,yTVC,zTVC])

    print("Time: {:.2f} s, Position: ({:.2f}, {:.2f}, {:.2f}) m, Velocity: ({:.2f}, {:.2f}, {:.2f}) m/s, Thrust: {:.2f} N".format(time_step * dt, x, y, z, vx, vy, vz, thrust))
    
    

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

# Visualize in 3D
positionsTVC = np.array(positionsTVC)
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
ax2.plot(positionsTVC[:,0], positionsTVC[:,1], positionsTVC[:,2])
ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.set_zlabel("Z (m)")
ax2.set_title("Rocket Trajectory TVC")
plt.show()
