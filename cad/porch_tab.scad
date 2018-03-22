// Porch tabs. These use plastic rivets to brace the poro are required.

// Returns a plate centered on the origin
module tab() {
    radius=2;  // Radius of the corner arcs
    x = 12;
    y = 24;
    $fn=100;
    linear_extrude(height=3) {
        hull() {
            // place 4 circles in the corners, with the given radius
            translate([(-x/2)+(radius/2), (-y/2)+(radius/2), 0])
            circle(r=radius);

            translate([(x/2)-(radius/2), (-y/2)+(radius/2), 0])
            circle(r=radius);

            translate([(-x/2)+(radius/2), (y/2)-(radius/2), 0])
            circle(r=radius);

            translate([(x/2)-(radius/2), (y/2)-(radius/2), 0])
            circle(r=radius);
        }
    }
}
module holes() {
    
    for(i=[[-3,-9],[3,-9],[-3,9],[3,9]]) {
        translate(i)
        cylinder(r=2.5,h=4);
    }
}

difference() {
    tab();
    holes();
}