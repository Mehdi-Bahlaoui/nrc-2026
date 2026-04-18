calib_angles = [0, 30, 60, 90, 120, 150, 180]

team_roboticore = [
    [389,  715,  968,  1428, 1827, 2212, 2528],
    [360,  735,  1018, 1493, 1892, 2222, 2550],
    [389,  715,  968,  1478, 1827, 2212, 2555],
    [385,  785,  1078, 1538, 1927, 2322, 2670],
    [360,  740,  1038, 1498, 1892, 2252, 2590],
    [360,  710,  999,  1388, 1732, 2082, 2450],
]

def pwm_to_angle(servo_index, pwm):
    table = team_roboticore[servo_index]
    if pwm <= table[0]:  return calib_angles[0]
    if pwm >= table[-1]: return calib_angles[-1]
    for i in range(len(table) - 1):
        if table[i] <= pwm <= table[i + 1]:
            t = (pwm - table[i]) / (table[i + 1] - table[i])
            return calib_angles[i] + t * (calib_angles[i + 1] - calib_angles[i])
    return None

# (min, default, max) per servo — S5 and S6 have default only
servos = [
    (0, [700,  2100, 1388]),   # S1
    (1, [500,  1493, 2550]),   # S2
    (2, [900,  1538, 2200]),   # S3
    (3, [1498,  500, 2500]),   # S4
    (4, [1408]),               # S5 — default only
    (5, [1396]),               # S6 — default only
]

labels = ["min", "default", "max"]

print(f"{'':4}  {'PWM':>6}  {'Angle':>8}  {'Label'}")
print("-" * 36)
for s_idx, pwm_list in servos:
    for i, pwm in enumerate(pwm_list):
        label = labels[i] if len(pwm_list) > 1 else "default"
        angle = pwm_to_angle(s_idx, pwm)
        print(f"S{s_idx+1}    {pwm:>6}   {angle:>7.1f}°  {label}")
