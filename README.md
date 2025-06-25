# Computational-Geometry-Program
Program were you can create a polygon by mouse clicks, than find its convex hull and compute its surface

# C++/Qt GUI Application

This project provides an interactive GUI for drawing polygons and performing computational geometry tasks, such as:
- Constructing a polygon from clicked points
- Drawing its **Convex Hull**
- **Triangulating** the polygon
- Computing **Surface Area**
- Finding the **Smallest Enclosing Circle**

# Main Features & Algorithms
Convex Hull: Implemented using a QuickHull-like recursive algorithm

Triangulation: Implementing ear clipping algorithm

Surface Area: Computed using as a sume of surface areas of triangualted triangles

Circle: Smallest enclosing circle computed from the convex hull using brute force naive approach

# Future Improvements
Handle collinear edge cases better in convex hull
Fix triangulation


# Technologies
Built using **C++** and **Qt Framework**.

---

## 📦 Installation

### Requirements
- Qt 5 or Qt 6 (recommended: Qt Creator for easy project setup)
- CMake (or use `.pro` file directly in Qt Creator)
- C++17-compatible compiler

### Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/Alharf42/Computational-Geometry-Program.git
   cd Computational-Geometry-Program
2. Open the .pro file in Qt Creator and click "Run", OR use terminal:
   mkdir build && cd build
   cmake ..
   make
   ./ComputtationalGeometeryApp
