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
    
    perp_a = sub(to_a, mul(e_dir, dot(to_a, e_dir)))
    perp_b = sub(to_b, mul(e_dir, dot(to_b, e_dir)))
    
    perp_a = normalize(perp_a)
    perp_b = normalize(perp_b)
    
    cos_theta = max(-1.0, min(1.0, dot(perp_a, perp_b)))
    return math.acos(cos_theta)

def rotate_around_axis(p, center, axis, angle):
    v = sub(p, center)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    dt = dot(axis, v)
    rotated = add(add(mul(v, cos_a), mul(cross(axis, v), sin_a)), mul(axis, dt * (1.0 - cos_a)))
    return add(center, rotated)

# Start flat (PI)
p_v0 = (0,0,0)
p_v1 = (2,0,0)
p_wa = (1,1,0)
p_wb = (1,-1,0)

print(f"Initial angle: {compute_dihedral(p_v0, p_v1, p_wa, p_wb)}")

# Fold it up to PI/2
# rotate wa around X by -PI/4?
p_wa_folded = (1, math.cos(math.pi/4), math.sin(math.pi/4))
p_wb_folded = (1, -math.cos(math.pi/4), math.sin(math.pi/4))
current_angle = compute_dihedral(p_v0, p_v1, p_wa_folded, p_wb_folded)
print(f"Folded angle: {current_angle} ({current_angle * 180 / math.pi} deg)")

# Now project back to PI
rest_angle = math.pi
angle_diff = current_angle - rest_angle

axis = normalize(sub(p_v1, p_v0))
mid = mul(add(p_v0, p_v1), 0.5)

half_corr = -angle_diff * 0.5

wa_proj = rotate_around_axis(p_wa_folded, mid, axis, half_corr)
wb_proj = rotate_around_axis(p_wb_folded, mid, axis, -half_corr)

proj_angle = compute_dihedral(p_v0, p_v1, wa_proj, wb_proj)
print(f"Projected angle: {proj_angle} ({proj_angle * 180 / math.pi} deg)")

print("wa_proj: ", wa_proj)
print("wb_proj: ", wb_proj)

