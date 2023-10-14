import math
import matplotlib.pyplot as plt
import copy

class CarPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def rotate_triangle(triangle, angle_rad):

    # Create a rotation matrix
    cosA = math.cos(angle_rad)
    sinA = math.sin(angle_rad)

    # Initialize the rotation matrix
    rotation_matrix = [
        [cosA, -sinA],
        [sinA, cosA]
    ]

    # Rotate each vertex of the triangle
    rotated_triangle = []
    for vertex in triangle:
        rotated_vertex = CarPoint(0, 0)
        rotated_vertex.x = vertex.x * rotation_matrix[0][0] + vertex.y * rotation_matrix[0][1]
        rotated_vertex.y = vertex.x * rotation_matrix[1][0] + vertex.y * rotation_matrix[1][1]
        rotated_triangle.append(rotated_vertex)

    return rotated_triangle

def translate_triangle(triangle, reference):
    triangle[0] = reference
    triangle[1] = CarPoint(triangle[1].x + reference.x, triangle[1].y + reference.y)
    triangle[2] = CarPoint(triangle[2].x + reference.x, triangle[2].y + reference.y)

    return triangle

def plot_colored_triangle(triangle):
    colors = ['r', 'g', 'b']  # Red, Green, Blue for corners
    x = [vertex.x for vertex in triangle]
    y = [vertex.y for vertex in triangle]

    for i in range(3):
        plt.scatter(x[i], y[i], c=colors[i], s=100, label=f'Point {i+1}')

    plt.xlim(-210, 210)
    plt.ylim(-210, 210)
    plt.plot(x + [x[0]], y + [y[0]], 'k-', linewidth=2)
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Colored Triangle')
    plt.grid(True)
    

def main():
    ClosePoint = [0,-200]
    #State = [-50, -135, 60]
    State = [0, 0, 0]
    angle = math.atan2(ClosePoint[0] - State[0],ClosePoint[1] - State[1]) - math.radians(State[2])
    angle = math.pi/2

    # Define the initial triangle vertices [a, b, c]
    initial_triangle = [CarPoint(0, 0), CarPoint(81, 71), CarPoint(81, -71)]

    
    rotated_triangle = rotate_triangle(initial_triangle, math.radians(State[2]))# Rotate the initial triangle
    copied_rotated_triangle = copy.deepcopy(rotated_triangle)  # Create a deep copy to avoid modification
    trans_triangle = translate_triangle(copied_rotated_triangle, CarPoint(State[0], State[1]))

    final_triangle = rotate_triangle(trans_triangle, angle)

    C = CarPoint(0,0)
    B = CarPoint(trans_triangle[0].x,trans_triangle[0].y)
    #Phase 2
    if(angle>0):
        A = CarPoint(trans_triangle[1].x,trans_triangle[1].y)
    else:
        A = CarPoint(trans_triangle[2].x,trans_triangle[2].y)
    
    C.x=A.x+(B.x-A.x)*math.cos(-angle) - (B.y-A.y)*math.sin(-angle)
    C.y=A.y+(B.x-A.x)*math.sin(-angle) + (B.y-A.y)*math.cos(-angle)

    
    print("rotate angle: ",angle*180/math.pi)
    print("around: ",A.x,",",A.y)
    print("from: ",B.x,",",B.y)
    print("result: ",C.x,",",C.y)
    # Plot the rotated triangle with different colored corners
    #plot_colored_triangle(final_triangle)
    plot_colored_triangle(initial_triangle)
    plot_colored_triangle(rotated_triangle)
    plot_colored_triangle(trans_triangle)
    plt.plot(C.x, C.y, 'X')

    plt.show()

if __name__ == "__main__":
    main()
