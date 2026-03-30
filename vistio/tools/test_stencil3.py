import math

def dot(a, b): return a[0]*b[0] + a[1]*b[1]
def cross(a, b): return a[0]*b[1] - a[1]*b[0]
def sub(a, b): return (a[0]-b[0], a[1]-b[1])
def norm(a): return math.sqrt(a[0]*a[0] + a[1]*a[1])

x0 = (0.0, 0.0)
x1 = (2.0, 0.0)
x2 = (1.0, 1.0) # wa
x3 = (1.5, -2.0) # wb

e = sub(x1, x0)
len_e = norm(e)

A1 = 0.5 * abs(cross(sub(x1, x0), sub(x2, x0)))
A2 = 0.5 * abs(cross(sub(x1, x0), sub(x3, x0)))

c2 = len_e / (2 * A1)
c3 = len_e / (2 * A2)

dot21 = dot(sub(x1, x2), e) / (len_e * len_e)
dot23 = dot(sub(x1, x3), e) / (len_e * len_e)
c0 = - dot21 * c2 - dot23 * c3

dot12 = dot(sub(x2, x0), e) / (len_e * len_e)
dot13 = dot(sub(x3, x0), e) / (len_e * len_e)
c1 = - dot12 * c2 - dot13 * c3

print(f"c0: {c0}, c1: {c1}, c2: {c2}, c3: {c3}")
print(f"sum: {c0 + c1 + c2 + c3}")

sum_x = c0*x0[0] + c1*x1[0] + c2*x2[0] + c3*x3[0]
sum_y = c0*x0[1] + c1*x1[1] + c2*x2[1] + c3*x3[1]
print(f"Lx: {sum_x}, Ly: {sum_y}")
