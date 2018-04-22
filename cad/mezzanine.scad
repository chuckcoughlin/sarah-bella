// OpenSCAD for a "mezzanine" extension for a Turtlebot3 burger
//           It includes a well for a breadboard and
//           rivet holes for a battery.
thickness = 4;
wide_y  = 36;    // The length of the wide rectangular portion
fillet_side= 16;    // End of flare to narrow
full_y  = 86;    // Greatest extent of the plate
wide_x  = 110;   // 1/2 width of the "wings"
narrow_x= 45;    // 1/2 width of the battery holder
breadboard_x = 52;  // Chopped
breadboard_y = 26;  // Chopped
breadboard_z = 10;
pillar_x     = 24;    // Center-line to center of pillar
pillar_y     = 30;    // Baseline to center of pillar
rim          = 3;     // Width of rim
rivet1_x     = 54;
rivet2_x     = 60;
rivet_y      = 8;
rivet_radius = 2.5;   // Hole radius for rivet attachment
round_radius = 2;     // Radius of the corner arcs

// This is the holder for the chopped off breadboard.
// rwidth is the rim width
module tub(z,rwidth) {
    linear_extrude(height=z,center=false,convexity=10) {
        difference() {
            square([breadboard_x+2*rwidth,breadboard_y+2*rwidth],true);
            square([breadboard_x,breadboard_y],true);
        }
    }
}
// Subtract this from the base platform
// These are the posts
module post_holes(z) {
    for(i=[[pillar_x,-pillar_y],[-pillar_x,-pillar_y]]) {
        translate(i)
        cylinder(r=5,h=z,$fn=6);
    }
}

// These are the various holes for rivets
// z - thickness
module rivet_holes(z) {
    for(i=[[rivet1_x,-rivet_y],[rivet2_x,-rivet_y],
           [-rivet1_x,-rivet_y],[-rivet2_x,-rivet_y]]) {
        translate(i)
        cylinder(r=rivet_radius,h=z);
    }
}
// 0,0 is upper left corner
module fillet(radius,angle) {
    rotate(angle,[0,0,1])
    difference() {
        square(radius,false);
        translate([radius,radius,0])
        circle(r=radius);
    }
    
}
// This is the basic plate. The baseline is y=0 along the GPIO pins.
// This object cannot be translated once created.
// y is negative towards us. Couldn't make a single hull work.
module base(z) {
    $fn=100;
    linear_extrude(height=z,center=false,convexity=10) {
        union() {
        hull() {
            // place 4 circles in the corners, with the rounding radius
            translate([-wide_x+round_radius/2, round_radius/2, 0])
            circle(r=round_radius);
            
            translate([wide_x-round_radius/2, round_radius/2, 0])
            circle(r=round_radius);
                
            translate([wide_x-round_radius/2, -wide_y+round_radius/2, 0])
            circle(r=round_radius);
                
            translate([-wide_x+round_radius/2, -wide_y+round_radius/2, 0])
            circle(r=round_radius);
            }
        };
        hull() {
            translate([narrow_x-round_radius/2, -wide_y+round_radius/2, 0])
            circle(r=round_radius);
                
            translate([narrow_x-round_radius/2, -full_y+round_radius/2, 0])
            circle(r=round_radius);
                
            translate([-narrow_x+round_radius/2, -full_y+round_radius/2, 0])
            circle(r=round_radius);
                
            translate([-narrow_x+round_radius/2, -wide_y+round_radius/2, 0])
            circle(r=round_radius);
        }
        translate([narrow_x,-wide_y,0]) 
        fillet(fillet_side,270); 
        translate([-narrow_x,-wide_y,0]) 
        fillet(fillet_side,180);
    }
}

// This is the basic plate less a rim width all around. This is meant to
// be subtracted from the standard plate.
// There should be no rim where the hulls join
module base_cutout(z,rwidth) {
    $fn=100;
    linear_extrude(height=z,center=false,convexity=10) {
        union() {
        hull() {
            translate([-wide_x+round_radius/2+rwidth, round_radius/2-rwidth, 0])
            circle(r=round_radius);
            
            translate([wide_x-round_radius/2-rwidth, round_radius/2-rwidth, 0])
            circle(r=round_radius);
                
            translate([wide_x-round_radius/2-rwidth, -wide_y+round_radius/2+rwidth, 0])
            circle(r=round_radius);
                
            translate([-wide_x+round_radius/2+rwidth, -wide_y+round_radius/2+rwidth, 0])
            circle(r=round_radius);
            }
        };
        hull() {
            translate([narrow_x-round_radius/2-rwidth, -wide_y+round_radius/2, 0])
            circle(r=round_radius);
                
            translate([narrow_x-round_radius/2-rwidth, -full_y+round_radius/2+rwidth, 0])
            circle(r=round_radius);
                
            translate([-narrow_x+round_radius/2+rwidth,-full_y+round_radius/2+rwidth, 0])
            circle(r=round_radius);
                
            translate([-narrow_x+round_radius/2+rwidth, -wide_y+round_radius/2, 0])
            circle(r=round_radius);
        } 
        translate([narrow_x-rwidth,-wide_y+rwidth,0]) 
        fillet(fillet_side,270); 
        translate([-narrow_x+rwidth,-wide_y+rwidth,0]) 
        fillet(fillet_side,180);  
    }
}

// --------------------- Final Assembly ----------------------
// Baseplate
translate([0,0,-thickness/2])
difference() {
    base(thickness);
    rivet_holes(thickness);
    post_holes(thickness);
}
translate([0,0,thickness/2])
difference() {
        base(thickness);
        base_cutout(thickness,rim);
 }
 translate([0,-breadboard_y/2,thickness/2])
 tub(thickness,rim);
     
