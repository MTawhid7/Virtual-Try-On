import math

def dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def add(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def mul(a, s): return (a[0]*s, a[1]*s, a[2]*s)
def norm(a): return math.sqrt(dot(a,a))
def normalize(a):
    n = norm(a)
    return mul(a, 1.0/n) if n > 0 else a

def compute_dihedral(p_v0, p_v1, p_wa, p_wb):
    edge = sub(p_v1, p_v0)
    e_dir = normalize(edge)
    to_a = sub(p_wa, p_v0)
    to_b = sub(p_wb, p_v0)
    
    perp_a = normalize(sub(to_a, mul(e_dir, dot(to_a, e_dir))))
    perp_b = normalize(sub(to_b, mul(e_dir, dot(to_b, e_dir))))
    
    cos_theta = max(-1.0, min(1.0, dot(perp_a, perp_b)))
    return math.acos(cos_theta)

def rotate_around_axis(p, center, axis, angle):
    v = sub(p, center)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    dt = dot(axis, v)
    rotated = add(add(mul(v, cos_a), mul(cross(axis, v), sin_a)), mul(axis, dt * (1.0 - cos_a)))
    return add(center, rotated)

p_v0 = (0,0,0)
p_v1 = (0,1,0)
axis = normalize(sub(p_v1, p_v0))

# Folded upwards (angle < 180)
w_a = (math.cos(math.pi/4), 0.5, math.sin(math.pi/4))
w_b = (-math.cos(math.pi/4), 0.5, math.sin(math.pi/4))

current = compute_dihedral(p_v0, p_v1, w_a, w_b)

# We want them to project to angle PI (flat). Thus to move OUTWARDS to +/- 1, 0.5, 0.
rest = math.pi
angle_diff = current - rest

# Attempt 1: half_corr = -angle_diff * 0.5
wa1 = rotate_around_axis(w_a, p_v0, axis, -angle_diff * 0.5)
wb1 = rotate_around_axis(w_b, p_v0, axis, angle_diff * 0.5)
print(f"Attempt 1 angle: {compute_dihedral(p_v0, p_v1, wa1, wb1)*180/math.pi}")

# The correct rotation must push wa "back" by -PI/4 and wb by +PI/4?
# What is the relationship between `axis` and the angle?
# In compute_dihedral, cos_theta = dot(perp_a, perp_b).
# It does NOT use a signed angle (atan2). It uses acos, which is always [0, PI].
# Which way is "out"?
# The cross product `cross(perp_a, perp_b)` points parallel or anti-parallel to `axis`.
perp_a = normalize(sub(w_a, mul(axis, dot(w_a, axis))))
perp_b = normalize(sub(w_b, mul(axis, dot(w_b, axis))))
n_cross = cross(perp_a, perp_b)

sign_a = 1.0 if dot(n_cross, axis) > 0 else -1.0
print(f"Sign check: cross={n_cross}, dot_axis={dot(n_cross, axis)}")

# Because we don't know which side w_a and w_b are on relative to the right-hand rule of `axis`,
# we MUST use the cross product to determine the sign of rotation!

