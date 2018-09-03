// Right-side ear and mounting assembly.
thickness  = 3;
mic_radius=5;
fillet = 1;
radius = 2.5;  // Hole radius
bottom_length = 36;
bottom_width = 18;
board_depth = 6;
board_width=16;  // Center of microphone is 6 from edge
ear_hole_height = 20;
vertical_height = 60;
// Returns a plate centered on the origin
module bottom() {
    round_radius=2;  // Radius of the corner arcs
    x = bottom_length;
    y = bottom_width;
    $fn=100;
    linear_extrude(height=thickness) {
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

// Include fillet support
module vertical() {
    difference() {
        union() {
            translate([-bottom_length/2,-bottom_width/2,0]) {
                cube([bottom_length,thickness,vertical_height]);
            }
            translate([-bottom_length/2,-bottom_width/2+thickness,thickness]) {
                cube([bottom_length,fillet,fillet]);
            }
            translate([-board_width/2-1.5,-bottom_width/2+thickness,
                        ear_hole_height-mic_radius-3*thickness/2]) {
                cube([board_width,board_depth+thickness+fillet,thickness]);
            }
            translate([-board_width/2-1.5,-bottom_width/2+2*thickness+board_depth,
                        ear_hole_height-mic_radius-thickness+fillet]) {
                cube([board_width,fillet,fillet]);
            }
        }
        translate([0,-bottom_width/2,ear_hole_height]) {
            rotate([-90,0,0]) cylinder(3*thickness,mic_radius,mic_radius,true);
        };
        // 1mm holes. Only way I can think of to fasten board
        translate([mic_radius,-bottom_width/2,ear_hole_height+24]) {
            rotate([-90,0,0]) cylinder(3*thickness,1,1,true);
        }
        // 1mm holes. Only way I can think of to fasten board
        translate([mic_radius-13,-bottom_width/2,ear_hole_height+24]) {
            rotate([-90,0,0]) cylinder(3*thickness,1,1,true);
        }
        // Score a line on the "ear" side for a paper-clip wire
        translate([mic_radius-13,-bottom_width/2,ear_hole_height+24-fillet/2]) {
            cube([13,fillet,fillet]);
        }
    }
}
module holes() {
    x = 10;  // From center
    y = 10 - radius;  // From edge
    for(i=[[-x+radius,-bottom_width/2+y],
           [x-radius,-bottom_width/2+y]] ) {
        translate(i)
        cylinder(r=radius,h=thickness);
    }
}

// For development only. For final we will print separately
// and weld together.
module ear() {
        translate([0,2,0]) {
        rotate([90,-0,0]) import("Ear_right.stl");
    }
}

union() {
    translate([0,-10,20]) {
        //ear();
    }
    vertical();
    difference() {
        bottom();
        holes();
    }
}