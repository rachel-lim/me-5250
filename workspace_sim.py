import math
import numpy as np
import matplotlib.pyplot as plt
import random

# link lengths 
L = {
    'AB': 1.968502, 
    'BC': 0.640000, 'BE': 1.00000, 
    'CD': 0.957578, 'ED': 0.651, 'EG': 1.0, 
    'EF': 0.652, 'DF': 0.54, # <-- WARNING: Check these three!            
    'FH': 1.034, 'GH': 0.575, 'GI': 0.837, 'HI': 1.145  
}

# angle limits
theta_1_min = math.radians(-90)
theta_1_max = math.radians(-180)

theta_2_min = math.radians(-90)
theta_2_max = math.radians(0)

theta_3_min = math.radians(-90)
theta_3_max = math.radians(0)

def intersect_circles(p1, r1, p2, r2):
  """find intersection point of two circles with origin p and radius r"""
  x1, y1 = p1
  x2, y2 = p2
  d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

  if d > r1 + r2 or d < abs(r1 - r2) or d == 0: return None

  a = (r1**2 - r2**2 + d**2) / (2 * d)
  h = np.sqrt(abs(r1**2 - a**2))

  x3 = x1 + a * (x2 - x1) / d
  y3 = y1 + a * (y2 - y1) / d

  x4 = x3 + h * (y2 - y1) / d
  y4 = y3 - h * (x2 - x1) / d
  x5 = x3 - h * (y2 - y1) / d
  y5 = y3 + h * (x2 - x1) / d

  return [(x4, y4), (x5, y5)]

def get_angle(p1, p2):
  """get angle between two lines"""
  return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

def normalize_angle(angle):
  """normalize angle to [-180, 180]"""
  while angle > math.pi: 
    angle -= 2 * math.pi
  while angle < -math.pi: 
    angle += 2 * math.pi
  return angle


# valid end effector points
x = []
y = []


for _ in range(num_samples):
  s = random.uniform(stroke_min, stroke_max)

  C_opts = intersect_circles(A, s, B, L['BC'])
  if not C_opts: continue
  C = C_opts[0] 

  # generate theta 1 within limits
  theta_BE = random.uniform(theta_1_min, theta_1_max)
  E = np.array([B[0] + L['BE'] * np.cos(theta_BE), 
                B[1] + L['BE'] * np.sin(theta_BE)])

  D_opts = intersect_circles(C, L['CD'], E, L['ED'])
  if not D_opts: continue
  D = random.choice(D_opts)

  F_opts = intersect_circles(E, L['EF'], D, L['DF'])
  if not F_opts: continue
  F = random.choice(F_opts)

  # generate random EG angle
  theta_EG = random.uniform(0, 2 * math.pi)
  relative_E = normalize_angle(theta_EG - theta_BE)

  # check theta 2 within limits
  if not (theta_2_min <= relative_E <= theta_2_max):
    continue
      
  G = np.array([E[0] + L['EG'] * np.cos(theta_EG), 
                E[1] + L['EG'] * np.sin(theta_EG)])

  H_opts = intersect_circles(G, L['GH'], F, L['FH'])
  if not H_opts: 
    continue
  H = random.choice(H_opts)

  I_opts = intersect_circles(G, L['GI'], H, L['HI'])
  if not I_opts: 
    continue
  I = random.choice(I_opts)

  # generate GI angle
  theta_GI = get_angle(G, I)
  relative_G = normalize_angle(theta_GI - theta_EG)

  # check theta 3 within limits
  if not (theta_3_min <= relative_G <= theta_3_max):
    continue

  x.append(I[0])
  y.append(I[1])

plt.figure(figsize=(10, 10))
plt.scatter(x, y, s=2, alpha=0.6, color='purple', label='Simulated End Effector point (I)')
plt.scatter(B[0], B[1], color='black', marker='X', s=150, label='Base Joint (B)')

plt.title("Gripper workspace")
plt.xlabel("X Coordinate (inches)")
plt.ylabel("Y Coordinate (inches)")
plt.axis('equal')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.show()
