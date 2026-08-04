import control as ctrl
import numpy as np
import cvxpy as cp
import paho.mqtt.client as mqtt
import json
import threading
import time
from datetime import datetime
import requests

# ------------------ Global Variables and Configuration ------------------

# MQTT broker settings
mqtt_broker =  "192.168.1.20" #"test.mosquitto.org"
mqtt_port = 1883
sensor_topic = "home/temperature/sensors"
control_topic = "home/temperature/control"

# MPC setpoint and initial values (adjust as needed)
set_point = 21              # Desired indoor temperature (°C)
OldContSig = 0              # Previous control signal (initial value)
OldpreTemp = 0              # Previous predicted temperature (initial value)
Ts1 = 1                     # Sampling time for discrete conversion in seconds

# Outdoor temperature (will be updated by a background thread)
OutDoorT = 0

# Outdoor temperature update configuration
outdoor_update_interval = 1800  # seconds (30 minutes)
api_key = 'd5ab2e39077ef93947204e682a35dda8'  # Replace with your actual API key
city_name = 'Montreal'

# Global variable for storing the latest sensor data
latest_sensor_data = None
# ------------------ Outdoor Temperature Update ------------------

def get_outdoor_temperature():
    """Retrieve the current outdoor temperature from the OpenWeather API."""
    global OutDoorT
    try:
        base_url = 'http://api.openweathermap.org/data/2.5/weather'
        params = {'q': city_name, 'appid': api_key, 'units': 'metric'}
        response = requests.get(base_url, params=params)
        if response.status_code == 200:
            data = response.json()
            OutDoorT = data['main']['temp']
            print(f"[{datetime.now()}] Updated Outdoor Temperature: {OutDoorT}°C")
        else:
            print(f"Failed to retrieve outdoor temperature: {response.status_code}")
    except requests.RequestException as e:
        print(f"Request error: {e}")

def outdoor_temperature_thread():
    """Thread function to periodically update the outdoor temperature."""
    while True:
        get_outdoor_temperature()
        time.sleep(outdoor_update_interval)

# ------------------ MPC Helper Functions ------------------
def set_mpc_par():
    """
    Set the MPC controller parameters and weighting factors.
    Returns:
        N: Control horizon.
        Q: Weight for the output error.
        R: Weight for the control input.
        P: Weight for the disturbance (if needed).
        nx: Number of states.
        nu: Number of inputs.
        nv: Number of measured disturbances.
    """
    N = 5           # Control horizon
    Q = 250         # Weight for the error between set-point and output
    R = 0.004       # Weight for the control input effort
    P = 0.01        # Weight for the disturbance (optional)

    Q = np.array([[Q]])
    R = np.array([[R]])
    P = np.array([[P]])
    nx = 1          # Number of states
    nu = 1          # Number of control inputs
    nv = 1          # Number of measured disturbances
    return N, Q, R, P, nx, nu, nv

def mpc_par():
    """
    Build the predictive model of the plant.
    Returns:
        Ad1, Bd1, Cd1, Dd1, Ed1: Discrete state-space matrices.
    """
    # Transfer function coefficients (example values)
    num = [[-0.018, 0.001417], [-0.5437, 0.005812]]
    den = [[1, 0.0229], [1, 0.0152]]

    # Convert the transfer functions to state-space models
    ss1 = ctrl.tf2ss(ctrl.TransferFunction(num[0], den[0]))
    ss2 = ctrl.tf2ss(ctrl.TransferFunction(num[1], den[1]))
# Note: The assignments here follow the given example code.
    A1 = ss1.A
    B1 = ss1.C      # Note: Using ss1.C for B1 per the provided snippet
    C1 = ss1.B
    D1 = ss1.D

    A2 = ss2.A
    B2 = ss2.C
    C2 = ss2.B
    D2 = ss2.D

    # Define continuous-time state-space systems
    Plant1 = ctrl.StateSpace(A1, B1, C1, D1)
    PlantDis = ctrl.StateSpace(A2, B2, C2, D2)

    # Convert to discrete-time using the sampling time Ts1
    Gd1 = ctrl.c2d(Plant1, Ts1)
    GDisd = ctrl.c2d(PlantDis, Ts1)

    # Extract discrete system matrices
    Ad1 = Gd1.A
    Bd1 = Gd1.B
    Cd1 = Gd1.C
    Dd1 = Gd1.D
    Ed1 = GDisd.B
    return Ad1, Bd1, Cd1, Dd1, Ed1
def mpc_control(set_point, Temp_measure, OutDoorT, OldContSig, OldpreTemp):
    """
    Formulate and solve the MPC optimization problem.
    
    Args:
        set_point: The desired temperature setpoint.
        Temp_measure: The current indoor (average) temperature.
        OutDoorT: The current outdoor temperature.
        OldContSig: The previous control signal (for fallback).
        OldpreTemp: The previous predicted temperature (for fallback).
    
    Returns:
        optimized_control_signal: The computed control action.
        predicted_temp: The predicted indoor temperature.
    """
    Ad1, Bd1, Cd1, Dd1, Ed1 = mpc_par()
    N, Q, R, P, nx, nu, nv = set_mpc_par()

    # Initial conditions
    x_init = np.array([[Temp_measure]])
    d_init = np.array([[OutDoorT]])
    r = set_point

    # Decision variables
    u = [cp.Variable((nu, 1)) for _ in range(N)]        # Control inputs over the horizon
    x = [cp.Variable((nx, 1)) for _ in range(N + 1)]      # States over the horizon
    d = [cp.Variable((nv, 1)) for _ in range(N)]          # Disturbances over the horizon

    # Set initial state and disturbance
    x[0] = x_init
    d[0] = d_init
optimized_control_signal = OldContSig
    predicted_temp = OldpreTemp

    # Build the objective function over the prediction horizon
    objective_expr = 0
    for k in range(N):
        objective_expr += cp.quad_form(r - Cd1 @ x[k], Q) + cp.quad_form(u[k], R)
        objective_expr += cp.quad_form(d[k][0], P)
    objective = cp.Minimize(objective_expr)

    # Build the constraints
    constraints = []
    for k in range(N):
        try:
            constraints.append(x[k + 1] == Ad1 @ x[k] + Bd1 @ u[k] + Ed1 @ d[k][0])
            constraints.append(u[k] >= 0)
            constraints.append(u[k] <= 300)
            # Constrain the predicted indoor temperature to lie between 0 and 70°C
            output_expression = Cd1 @ (Ad1 @ x[k] + Bd1 @ u[k] + Ed1 @ d[k][0])
            constraints.append(output_expression >= 0)
            constraints.append(output_expression <= 70)
        except Exception as e:
            print(f"Error setting constraint at iteration {k}: {e}")
            continue
 # Solve the optimization problem
    try:
        problem = cp.Problem(objective, constraints)
        problem.solve(solver=cp.OSQP, verbose=False)
        if problem.status == cp.OPTIMAL:
            optimized_control_signal = u[0].value[0][0]
            predicted_temp_val = Cd1 @ (Ad1 @ x[0] + Bd1 @ u[0].value + Ed1 @ d[0][0])
            predicted_temp = predicted_temp_val.flatten()[0]

            # Enforce bounds on the control signal
            if optimized_control_signal > 100:
                optimized_control_signal = 100
            if optimized_control_signal < 0:
                optimized_control_signal = 0
        else:
            print("Optimization failed, using previous control signal and prediction")
            optimized_control_signal = OldContSig
            predicted_temp = OldpreTemp
    except Exception as e:
        print(f"Optimization error: {e}")
        optimized_control_signal = OldContSig
        predicted_temp = OldpreTemp

    return optimized_control_signal, predicted_temp
# ------------------ MQTT Callbacks ------------------

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker with result code " + str(rc))
    # Subscribe to sensor data topic; control signals will be published separately
    client.subscribe(sensor_topic)

def on_message(client, userdata, msg):
    """
    Update the latest sensor data on receiving a message.
    The control signal will be computed and published every minute in a separate thread.
    """
    global latest_sensor_data
    try:
        data = json.loads(msg.payload.decode())
        latest_sensor_data = data
        print(f"[{datetime.now()}] Updated sensor data: {data}")
    except Exception as e:
        print(f"Error processing sensor message: {e}")
# ------------------ Control Signal Publisher ------------------

def control_signal_publisher(client):
    """
    Computes and publishes the control signal every minute using the latest sensor data.
    """
    global latest_sensor_data, OldContSig, OldpreTemp, OutDoorT
    while True:
        if latest_sensor_data is not None:
            # Expecting keys: 'temp1', 'temp2', 'temp3'
            indoor_temps = [
                latest_sensor_data.get('temp1', 0),
                latest_sensor_data.get('temp2', 0),
                latest_sensor_data.get('temp3', 0)
            ]
            avg_temp = sum(indoor_temps) / len(indoor_temps)
            print(f"[{datetime.now()}] Calculated average indoor temperature: {avg_temp:.2f}°C")
        else:
            print(f"[{datetime.now()}] No sensor data available; using default indoor temperature of 0°C")
            avg_temp = 0

        # Compute the control signal using MPC
        control_signal, predicted_temp = mpc_control(
            set_point, avg_temp, OutDoorT, OldContSig, OldpreTemp
        )
        OldContSig = control_signal
        OldpreTemp = predicted_temp

        # Publish the control signal and predicted temperature
        payload = json.dumps({
            "control_signal": control_signal,
            "predicted_temp": predicted_temp
        })
        client.publish(control_topic, payload)
        print(f"[{datetime.now()}] Published control signal: {control_signal:.2f}, Predicted Temp: {predicted_temp:.2f}")

        # Wait for one minute before publishing the next control signal
        time.sleep(60)
# ------------------ Main Program ------------------

# Set up the MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)

# Start the thread that periodically updates the outdoor temperature
outdoor_thread = threading.Thread(target=outdoor_temperature_thread, daemon=True)
outdoor_thread.start()

# Start the control signal publisher thread that publishes every one minute
control_thread = threading.Thread(target=control_signal_publisher, args=(client,), daemon=True)
control_thread.start()

# Begin the MQTT network loop (this call blocks)
client.loop_forever()
