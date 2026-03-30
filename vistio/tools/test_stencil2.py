import math

def dot(a, b): return a[0]*b[0] + a[1]*b[1]
def cross(a, b): return a[0]*b[1] - a[1]*b[0]
def sub(a, b): return (a[0]-b[0], a[1]-b[1])
def cot(p0, p1, p2): # angle at p0
    v1 = sub(p1, p0)
    v2 = sub(p2, p0)
    return dot(v1, v2) / abs(cross(v1, v2))

v0 = (0.0, 0.0)
v1 = (2.0, 0.0)
wa = (1.0, 1.0)
wb = (1.5, -2.0)

cot_wa = cot(wa, v0, v1)
cot_wb = cot(wb, v0, v1)

# angles at v1 (opposite to v0 in the two triangles)
cot_v1_a = cot(v1, v0, wa)
cot_v1_b = cot(v1, v0, wb)

# angles at v0 (opposite to v1 in the two triangles)
cot_v0_a = cot(v0, v1, wa)
cot_v0_b = cot(v0, v1, wb)

c_wa = cot_wa
c_wb = cot_wb
c_0 = - (cot_v1_a + cot_v1_b)
c_1 = - (cot_v0_a + cot_v0_b)

print(f"c_0: {c_0}, c_1: {c_1}, c_wa: {c_wa}, c_wb: {c_wb}")
print("sum:", c_0 + c_1 + c_wa + c_wb)

sum_x = c_0*v0[0] + c_1*v1[0] + c_wa*wa[0] + c_wb*wb[0]
sum_y = c_0*v0[1] + c_1*v1[1] + c_wa*wa[1] + c_wb*wb[1]

print(f"Lx: {sum_x}, Ly: {sum_y}")
