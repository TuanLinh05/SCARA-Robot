"""Cubic Bezier geometry on the PC; bounded chords keep MCU work deterministic."""
import math

def distance_to_line(p,a,b):
    dx=b[0]-a[0]; dy=b[1]-a[1]; length=math.hypot(dx,dy)
    return abs(dx*(a[1]-p[1])-(a[0]-p[0])*dy)/length if length else math.dist(p,a)

def flatten_cubic(control,max_step,max_error,cancelled=lambda:False):
    """de Casteljau subdivision. Control hull bounds deviation from each chord.
    Returns only end vertices; callers retain the exact beginning once.
    """
    output=[]
    def visit(p,depth):
        if cancelled(): raise ValueError("Tác vụ đường cong đã hủy.")
        polygon=sum(math.dist(a,b) for a,b in zip(p,p[1:]))
        flat=max(distance_to_line(v,p[0],p[3]) for v in p[1:3])
        if polygon<=max_step and flat<=max_error:
            output.append(p[3]); return
        if depth>=18 or len(output)>=4096: raise ValueError("Đường cong vượt ngân sách chia đoạn.")
        def midpoint(a,b): return tuple((x+y)/2 for x,y in zip(a,b))
        a,b,c=(midpoint(x,y) for x,y in zip(p,p[1:]))
        d=midpoint(a,b); e=midpoint(b,c); f=midpoint(d,e)
        visit((p[0],a,d,f),depth+1); visit((f,e,c,p[3]),depth+1)
    visit(tuple(control),0)
    return tuple(output)

def ellipse_arc(cx,cy,rx,ry,start,end,step,error):
    # Match end position/tangent; <=90-degree pieces approximate a circular
    # arc within 0.000273 radius. The final DDA is checked against the circle.
    pieces=max(1,math.ceil(abs(end-start)/(math.pi/2)))
    output=[]
    for i in range(pieces):
        a=start+(end-start)*i/pieces; b=start+(end-start)*(i+1)/pieces
        k=(4/3)*math.tan((b-a)/4)
        p0=(cx+rx*math.cos(a),cy+ry*math.sin(a)); p3=(cx+rx*math.cos(b),cy+ry*math.sin(b))
        p1=(p0[0]-k*rx*math.sin(a),p0[1]+k*ry*math.cos(a))
        p2=(p3[0]+k*rx*math.sin(b),p3[1]-k*ry*math.cos(b))
        output.extend(flatten_cubic((p0,p1,p2,p3),step,error))
    return tuple(output)

def ellipse_radial_error(xy,cx,cy,rx,ry):
    dx=xy[0]-cx; dy=xy[1]-cy; radius=math.hypot(dx/rx,dy/ry)
    if not radius: return min(rx,ry)
    # Distance to the radial ellipse point is a conservative bound on the
    # nearest-ellipse distance, suitable for a full-path ink preflight.
    return math.hypot(dx-dx/radius,dy-dy/radius)
