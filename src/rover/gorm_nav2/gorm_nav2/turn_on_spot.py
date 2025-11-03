import numpy as np

def turn_on_spot(ang_vel, wheel_radius=0.12, d_lr=0.894, d_fr=0.77, L=0.849, offset=-0.0135):
    """
    Turn the vehicle on the spot (pure rotation).
    Returns steering angles and wheel velocities.
    """
    # Steering angles (wheels at max deflection)
    theta_FL = -np.pi/4
    theta_FR = np.pi/4
    theta_RL = np.pi/4
    theta_RR = -np.pi/4
    steering_angles = np.array([theta_FL, theta_FR, theta_RL, theta_RR]) * -1

    # Wheel velocities for pure spin
    V_FL = -(ang_vel)
    V_FR = -(ang_vel)
    V_ML = -(ang_vel)
    V_MR = -(ang_vel)
    V_RL = -(ang_vel)
    V_RR = -(ang_vel)

    wheel_velocities = np.array([V_FL, V_FR, V_ML, V_MR, V_RL, V_RR]) / (wheel_radius * 2)
    return steering_angles, wheel_velocities
