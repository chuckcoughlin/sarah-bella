// Porch tabs. These use plastic rivets to brace the porch.
// 2 are required.
length = 24;
thickness  = 3;
width  = 12;
radius = 2.5;  // Hole radius
// Returns a plate centered on the origin
module tab(x,y,z) {
    round_radius=2;  // Radius of the corner arcs
    $fn=100;
    linear_extrude(height=z) {
        hull() {
            // place 4 circles in the corners, with the given radius
            translate([(-x/2)+(round_radius/2), (-y/2)+(round_radius/2), 0])
            circle(r=round_radius);

            translate([(x/2)-(round_radius/2), (-y/2)+(round_radius/2), 0])
            circle(r=round_radius);

            translate([(-x/2)+(round_radius/2), (y/2)-(round_radius/2), 0])
            circle(r=round_radius);

            translate([(x/2)-(round_radius/2), (y/2)-(round_radius/2), 0])
            circle(r=round_radius);
        }
    }
}
module holes(x,y,z,hole_radius) {
    pad = 0.5;
    for(i=[[-x/2+hole_radius+pad,-y/2+hole_radius+pad],
           [x/2-hole_radius-pad,-y/2+hole_radius+pad],
           [-x/2+hole_radius+pad,y/2-hole_radius-pad],
           [x/2-hole_radius-pad,y/2-hole_radius-pad]]) {
        translate(i)
        cylinder(r=hole_radius,h=z);
    }
}

difference() {
    tab(width,length,thickness);
    holes(width,length,thickness,radius);
}