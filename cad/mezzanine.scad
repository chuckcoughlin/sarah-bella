// OpenSCAD for a "mezzanine" extension for a Turtlebot3 burger
//           It includes supports for a breadboard and
//           rivet holes for a battery. The breadboard is
//           centered on the origin. y is positive toward the RPi.
thickness = 4;

wide_x  = 110;    // 1/2 width of the "wings"
start_cutout_x  = 40;   // 1/2 width of cutout around the helper board
end_cutout_x    = 65;   // 1/2 width of cutout around the helper board
narrow_x= 45;    // 1/2 width of the battery holder

min_y = 18;      // beginning of "wing
cutout_y = 0;    // close edge of the cutout
wide_y  = -26;    // The near edge the wide rectangular portion
fillet_side= 16;  // End of flare to narrow
full_y  = -86;    // Greatest extent of the plate

breadboard_width = 164;     // 
breadboard_height = 26;     // Chopped
breadboard_thickness = 10;
breadboard_z = 10;          // Base to bottom of breadboard

pillar_x     = 24;    // Center-line to center of pillar
pillar_y     = -20;   // Baseline to center of pillar
rim          = 3;     // Width of rim

battery_rivet_x     = 28;  // On centerline
battery_rivet_y     = 16;  // On centerline, 16mm from rim.
rivet2_x     = 60;
rivet_y      = 8;
rivet_radius = 2.5;   // Hole radius for rivet attachment
round_radius = 2;     // Radius of the corner arcs

// Make 4 holders for the breadboard corners plus 1 at midpoint on battery edge
module tub(z,rwidth) {
    //linear_extrude(height=z,center=false,convexity=10) {
     //   difference() {
      //      square([breadboard_x+2*rwidth,breadboard_y+2*rwidth],true);
      //      square([breadboard_x,breadboard_y],true);
       // }
    //}
}
// Subtract this from the base platform
// These are the posts
module post_holes(z) {
    for(i=[[pillar_x,pillar_y],[-pillar_x,pillar_y]]) {
        translate(i)
        rotate(22.5,0,0)
        cylinder(r=4,h=z,$fn=6);
    }
}

// These are the various holes for battery rivets.
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
// This is the basic plate. The baseline is y=0 along the edge of the helper
// board next to the RPi GPIO header. This object cannot be translated once created.
// y is positive towards the RPi. Couldn't make a single hull work.
module base(z) {
    $fn=100;
    linear_extrude(height=z,center=false,convexity=10) {
        // place circles in the corners, account for the rounding radius
        union() {
         // Breadboard rectangular area.
        hull() {
            translate([-wide_x+round_radius/2, wide_y+round_radius/2, 0])
            circle(r=round_radius);
            
            translate([wide_x-round_radius/2, wide_y+round_radius/2, 0])
            circle(r=round_radius);
            
            translate([wide_x-round_radius/2, cutout_y-round_radius/2, 0])
            circle(r=round_radius);
                
            translate([-wide_x+round_radius/2, cutout_y-round_radius/2, 0])
            circle(r=round_radius);
            }
        };
        // Battery area
        hull() {
            translate([narrow_x-round_radius/2, wide_y-round_radius/2, 0])
            circle(r=round_radius);
                
            translate([narrow_x-round_radius/2, full_y+round_radius/2, 0])
            circle(r=round_radius);
                
            translate([-narrow_x+round_radius/2, full_y+round_radius/2, 0])
            circle(r=round_radius);
                
            translate([-narrow_x+round_radius/2, wide_y-round_radius/2, 0])
            circle(r=round_radius);
        }
        // Wingtip left
        hull() {
            translate([-wide_x+round_radius/2, cutout_y-round_radius/2, 0])
            circle(r=round_radius);
            
            translate([-wide_x+round_radius/2, min_y+round_radius/2, 0])
            circle(r=round_radius);
            
            translate([-end_cutout_x+round_radius/2, min_y+round_radius/2, 0])
            circle(r=round_radius);
            
            translate([-start_cutout_x+round_radius/2, cutout_y+round_radius/2, 0])
            circle(r=round_radius);
        }
        // Wingtip right
        hull() {
            translate([wide_x-round_radius/2, cutout_y-round_radius/2, 0])
            circle(r=round_radius);
            
            translate([wide_x-round_radius/2, min_y+round_radius/2, 0])
            circle(r=round_radius);
            
            translate([end_cutout_x-round_radius/2, min_y+round_radius/2, 0])
            circle(r=round_radius);
            
            translate([start_cutout_x-round_radius/2, cutout_y+round_radius/2, 0])
            circle(r=round_radius);
        }
        translate([narrow_x,wide_y,0]) 
        fillet(fillet_side,270); 
        translate([-narrow_x,wide_y,0]) 
        fillet(fillet_side,180);
    }
}

// This is the basic plate less a rim width all around. This is meant to
// be subtracted from the standard plate. There should be no rim where hulls join.
module base_cutout(z,rwidth) {
    $fn=100;
    linear_extrude(height=z,center=false,convexity=10) {
        // place circles in the corners, account for the rounding radius and rim
        // There is no rim on joint.
        union() {
            // Breadboard rectangular area.
            hull() {
            translate([-wide_x+round_radius/2+rwidth, wide_y+round_radius/2+rwidth, 0])
            circle(r=round_radius);
            
            translate([wide_x-round_radius/2-rwidth, wide_y+round_radius/2+rwidth, 0])
            circle(r=round_radius);
            
            translate([wide_x-round_radius/2-rwidth, cutout_y-round_radius/2, 0])
            circle(r=round_radius);
                
            translate([-wide_x+round_radius/2+rwidth, cutout_y-round_radius/2, 0])
            circle(r=round_radius);
            }
        };
        // Battery area
        hull() {
            translate([narrow_x-round_radius/2-rwidth, wide_y+round_radius, 0])
            circle(r=round_radius);
                
            translate([narrow_x-round_radius/2-rwidth, full_y+round_radius/2+rwidth, 0])
            circle(r=round_radius);
                
            translate([-narrow_x+round_radius/2+rwidth, full_y+round_radius/2+rwidth, 0])
            circle(r=round_radius);
                
            translate([-narrow_x+round_radius/2+rwidth, wide_y+round_radius, 0])
            circle(r=round_radius);
        }
        // Wingtip left
        hull() {
            translate([-wide_x+round_radius/2+rwidth, cutout_y-round_radius/2, 0])
            circle(r=round_radius);
            
            translate([-wide_x+round_radius/2+rwidth, min_y+round_radius/2-rwidth, 0])
            circle(r=round_radius);
            
            translate([-end_cutout_x+round_radius/2-rwidth, min_y+round_radius/2-rwidth, 0])
            circle(r=round_radius);
            
            translate([-start_cutout_x+round_radius/2, cutout_y-round_radius/2-rwidth, 0])
            circle(r=round_radius);
        }
        // Wingtip right
        hull() {
            translate([wide_x-round_radius/2-rwidth, cutout_y-round_radius/2, 0])
            circle(r=round_radius);
            
            translate([wide_x-round_radius/2-rwidth, min_y+round_radius/2-rwidth, 0])
            circle(r=round_radius);
            
            translate([end_cutout_x-round_radius/2+rwidth, min_y+round_radius/2-rwidth, 0])
            circle(r=round_radius);
            
            translate([start_cutout_x-round_radius/2, cutout_y-round_radius/2-rwidth, 0])
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
     color("red")
     base(thickness);
     base_cutout(thickness,rim);
}
 //translate([0,-breadboard_y/2,thickness/2])
 //tub(thickness,rim);
     
