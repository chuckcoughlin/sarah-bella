// OpenSCAD for a "mezzanine" extension for a Turtlebot3 burger
//           It includes a well for a breadboard and
//           rivet holes for a battery.
thickness = 4;
wide_y  = 26;    // The length of the wide rectangular portion
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
            // place 8 circles in the corners, with the rounding radius
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
module base_cutout(z,rwidth) {
    $fn=100;
    linear_extrude(height=z,center=false,convexity=10) {
        union() {
        hull() {
            // place 8 circles in the corners, with the rounding radius
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
    }
}

// --------------------- Final Assembly ----------------------
// Baseplate
translate([0,0,-thickness/2])
base(thickness);
translate([0,0,thickness/2])
difference() {
        base(thickness);
        base_cutout(thickness,rim);
  }