# Workspace Calculation
# Marcus Rosette
# 29 November 2022


# Parameters (from Reach Robotics Bravo7 manual) -----------------------------
# Joint Limits - J1 = end effector opening, ..., J7 = Base revolute joint
j1 = 118 # [mm] - Jaw opening
j2 = Inf # [deg] - Continuous revolute
j3 = 180 # [deg] - Elbow
j4 = Inf # [deg] - Continuous revolute
j5 = 180 # [deg] - Elbow
j6 = 180 # [deg] - Elbow
j7 = Inf # [deg] - Continuous revolute

# Body Lengths
a = 365 # [mm] - j3 to end effector
b = 160 # [mm] - j5 to j3
c = 292 # [mm] - j6 to j5
d = 159 # [mm] - table top to j6
e = 92  # [mm] - max OD
f = 82  # [mm] - min OD (at tip of wrist)