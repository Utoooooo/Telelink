import Telelink
import droneControlFunctions as dcf

telemetry = Telelink.Telelink()
ref = 200
dt = 0.04
pid_z = dcf.droneController(dt, kp=0.4, ki=0.5, kd=0.1)
pid_vz = dcf.DistPID(dt, kp=0.7)
uart_port = 'COM16'
link = dcf.drone_control_init(uart_port)
hover_point = 1400
actuation = 0
prev_height = telemetry.tof

print("Press any key to continue")
input(" ")
while True:
    telemetry.receive()
    height = telemetry.tof
    v_z = (height - prev_height) / dt
    v_target = pid_vz.update(ref, float(prev_height))
    actuation = pid_z.update(v_target, v_z) + hover_point
    if actuation > 2000:
        actuation = 2000
    elif actuation < 1000:
        actuation = 1000
    prev_height = height
    dcf.drone_control_update(link, thrust=actuation, roll=1500, pitch=1500, yaw=1500)
