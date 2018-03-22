// OpenSCAD for a "porch" extension for a Turtlebot3 burger
//           It includes a lampholder.

// This is the basic perimeter
module base() {
    linear_extrude(height=3,center=true,twist=0,convexity=10,scale=1.0) {
        polygon(points=[[-45,0],[45,0],[30,30],[-30,30],[-45,0]]);
    }
}

module lampholder() {
    difference(
        cube(30,30,3),
        cylinder(16)
    );
}

base();
lampholder();