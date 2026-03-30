import math

def dot(a, b): return a[0]*b[0] + a[1]*b[1]
def cross(a, b): return a[0]*b[1] - a[1]*b[0]
def sub(a, b): return (a[0]-b[0], a[1]-b[1])
def norm(a): return math.sqrt(a[0]*a[0] + a[1]*a[1])

def edge_stencil(p0, p1, pa, pb):
    edge = sub(p1, p0)
    edge_len = norm(edge)
    inv_e2 = 1.0 / (edge_len * edge_len)
    
    A_a = 0.5 * abs(cross(sub(p1, p0), sub(pa, p0)))
    A_b = 0.5 * abs(cross(sub(p1, p0), sub(pb, p0)))
    
    ca = edge_len / (2.0 * A_a)
    cb = edge_len / (2.0 * A_b)
    
    dot0_a = dot(sub(p1, pa), edge) * inv_e2
    dot0_b = dot(sub(p1, pb), edge) * inv_e2
    c0 = -dot0_a * ca - dot0_b * cb
    
    dot1_a = dot(sub(pa, p0), edge) * inv_e2
    dot1_b = dot(sub(pb, p0), edge) * inv_e2
    c1 = -dot1_a * ca - dot1_b * cb
    
    raw = [c0, c1, ca, cb]
    n = math.sqrt(sum(x*x for x in raw))
    return [x/n for x in raw]

# quad grid, horizontal edge
p0 = (0,0)
p1 = (1,0)
pa = (0,1)
pb = (1,-1)
print("horizontal:", edge_stencil(p0, p1, pa, pb))

# quad grid, vertical edge
p0 = (0,0)
p1 = (0,1)
pa = (-1,0)
pb = (1,1)
print("vertical:", edge_stencil(p0, p1, pa, pb))

# quad grid, diagonal edge
p0 = (0,1)
p1 = (1,0)
pa = (1,1)
pb = (0,0)
print("diagonal:", edge_stencil(p0, p1, pa, pb))
