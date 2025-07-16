from pid_controller import PIDController

pid = PIDController()
u = pid.compute(10, 0, 0.1)
print(f"PID output: {u}")
