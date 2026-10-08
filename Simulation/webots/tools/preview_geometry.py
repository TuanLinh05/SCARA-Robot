"""Optional geometry preview from actual PROTO/STL; no Webots physics executed.
Requires NumPy/Matplotlib only for this development tool, not the controller.
"""
from pathlib import Path
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from build_project import read_stl
from vrml_check import Node, Parser, load_schema

PROJECT = Path(__file__).resolve().parents[1]


def pose(fields):
    matrix = np.eye(4)
    x,y,z,angle = fields.get('rotation',[0,0,1,0])
    axis = np.array([x,y,z],float)
    axis /= np.linalg.norm(axis)
    x,y,z = axis
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    matrix[:3,:3] = np.eye(3)*math.cos(angle)+(1-math.cos(angle))*np.outer(axis,axis)+math.sin(angle)*skew
    matrix[:3,3] = fields.get('translation',[0,0,0])
    return matrix


def box_faces(size):
    x,y,z = np.array(size)/2
    v = np.array([[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]])
    return [v[f] for f in [[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]]


def cylinder_faces(radius,height):
    angle = np.linspace(0,2*np.pi,25)
    ring = np.c_[radius*np.cos(angle),radius*np.sin(angle)]
    lower,upper = np.c_[ring,np.full(25,-height/2)],np.c_[ring,np.full(25,height/2)]
    return [np.array([lower[i],lower[i+1],upper[i+1],upper[i]]) for i in range(24)]+[lower[:-1],upper[:-1]]


def render():
    parser = Parser(PROJECT/'protos/ScaraBrush.proto',load_schema(PROJECT))
    roots = parser.parse()
    fig = plt.figure(figsize=(10.5,8),facecolor='#f3f6fa')
    ax = fig.add_subplot(projection='3d',facecolor='#f3f6fa')
    def draw(node,matrix):
        if not isinstance(node,Node):
            return
        if node.kind in ('Robot','Solid','TouchSensor','Pose','GPS','Pen'):
            matrix = matrix @ pose(node.fields)
        if node.kind == 'Shape':
            geometry = node.fields['geometry']
            color = node.fields['appearance'].fields.get('baseColor',[.3,.3,.3])
            faces = []
            if geometry.kind == 'Mesh':
                faces = read_stl((PROJECT/'protos'/geometry.fields['url'][0]).resolve())
            elif geometry.kind == 'Box':
                faces = box_faces(geometry.fields['size'])
            elif geometry.kind == 'Cylinder':
                faces = cylinder_faces(geometry.fields['radius'],geometry.fields['height'])
            elif geometry.kind == 'Sphere':
                faces = box_faces([geometry.fields['radius']*2]*3)
            converted = [(np.asarray(face) @ matrix[:3,:3].T+matrix[:3,3])*1000 for face in faces]
            ax.add_collection3d(Poly3DCollection(converted,facecolors=color,linewidth=0,shade=True))
        for child in node.fields.get('children',[]):
            draw(child,matrix)
        if 'endPoint' in node.fields:
            draw(node.fields['endPoint'],matrix)
    for root in roots:
        draw(root,np.eye(4))
    ax.add_collection3d(Poly3DCollection([np.array([[-50,-50,0],[50,-50,0],[50,50,0],[-50,50,0]])],facecolor='white',edgecolor='#8c99a6'))
    angle = np.linspace(0,2*np.pi,700)
    radius = 23+11*np.cos(5*angle)
    ax.plot(radius*np.cos(angle),radius*np.sin(angle),np.full_like(angle,.2),color='#306cc5',linewidth=1.6,label='Desired path',zorder=1000)
    ax.set(xlim=(-100,150),ylim=(-170,65),zlim=(-4,320),xlabel='X (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
    ax.set_box_aspect([250,235,324])
    ax.view_init(elev=28,azim=55)
    ax.set_title('X-SCARA | generated STL assembly + brush\nOffline geometry preview - physics and native ink not executed',fontsize=12,pad=18)
    ax.legend(loc='upper right')
    fig.tight_layout()
    folder = PROJECT/'output/validation'
    folder.mkdir(parents=True,exist_ok=True)
    path = folder/'geometry_preview.png'
    fig.savefig(path,dpi=150)
    plt.close(fig)
    print(path)


if __name__ == '__main__':
    render()

