// OpenSCAD for a "mezzanine" extension for a Turtlebot3 burger
//           It includes supports for a breadboard and
//           rivet holes for a battery. The breadboard is
//           centered on the origin. y is positive toward the RPi.
thickness = 4;

wide_x  = 110;    // 1/2 width of the "wings"
start_cutout_x  = 40;   // 1/2 width of cutout around the helper board
end_cutout_x    = 65;   // 1/2 width of cutout around the helper board
narrow_x= 80;    // 1/2 width of the battery holder

min_y    = 18;    // beginning of "wing
cutout_y = 0;     // close edge of the cutout
wide_y  = -26;    // The near edge the wide rectangular portion
fillet_side= 16;  // End of flare to narrow
full_y  = -86;    // Greatest extent of the plate

// NOTE: Added 1mm of "slop" to breadboard dimensions
breadboard_width = 165;     // 
breadboard_height = 27;     // Chopped
breadboard_thickness = 10;
breadboard_z = 10;          // Base to bottom of breadboard
post_width   = 8;          // Support post for breadboard

pillar_x     = 24;    // Center-line to center of pillar
pillar_y     = -20;   // Baseline to center of pillar
rim          = 3;     // Width of rim

arduino_width       = 54; // y direction
battery_rivet_x     = 28; // On centerline
battery_rivet_y     = 8;  // On centerline, 16mm from rim.
main_rivet_x        = 48; // Connect to main board
main_rivet_y        = 0;

rivet_radius = 2.5;   // Hole radius for rivet attachment
round_radius = 2;     // Radius of the corner arcs

// Make 4 holders for the breadboard corners
module breadboard_posts() {
     z = breadboard_z+thickness;
    
     for(i=[[breadboard_width/2,breadboard_height/2,z/2],
            [-breadboard_width/2,breadboard_height/2,z/2], 
            [-breadboard_width/2,-breadboard_height/2,z/2],
            [breadboard_width/2,-breadboard_height/2,z/2]]) {
                translate(i)
                cube(size=[post_width,post_width,z],center=true);
       }
}
// Offset holders for the breadboard corners to make notches at the top
module breadboard_notches() {
     z = 2*breadboard_z+thickness;
     w = post_width;
     for(i=[[breadboard_width/2-w/2,breadboard_height/2-w/2,z/2],
            [-breadboard_width/2+w/2,breadboard_height/2-w/2,z/2], 
            [-breadboard_width/2+w/2,-breadboard_height/2+w/2,z/2],
            [breadboard_width/2-w/2,-breadboard_height/2+w/2,z/2]]) {
                translate(i)
                cube(size=[w,w,breadboard_thickness],center=true);
       }
}
// Subtract this from the base platform
// These are the posts.
// Cylinder arguments: height,r1,r2,center
module post_holes(z) {
    for(i=[[pillar_x,pillar_y],[-pillar_x,pillar_y]]) {
        translate(i)
        rotate(22.5,0,0)
        cylinder(z,4,4,false,$fn=6);
    }
}

// These are for connecting the main board
// z - thickness
module main_holes(z) {
    for(i=[[main_rivet_x,main_rivet_y],
           [-main_rivet_x,main_rivet_y]] ) {
        translate(i)
        cylinder(r=rivet_radius,h=z);
    }
}
// These are the various holes for battery rivets.
// We've offset by 35mm
module rivet_holes(z) {
    for(i=[[battery_rivet_x-35,0],[-battery_rivet_x-35,0],
           [-35,battery_rivet_y],[-35,-battery_rivet_y]]) {
        translate(i)
        cylinder(r=rivet_radius,h=z);
    }
}
// These are the various holes for arduino rivets.
// These are on the plus side
module arduino_holes(z) {
    for(i=[[11,17],[11,45],
           [62,0],[64,48]]) {
        translate(i)
        cylinder(r=rivet_radius,h=z);
    }
}
// These are for connecting the "ears" near the
// end of the wings.
module ear_holes(z) {
    for(i=[[wide_x-14,cutout_y-2+10],
           [wide_x-14,cutout_y-2-10],
           [-wide_x+14,cutout_y-2+10],
           [-wide_x+14,cutout_y-2-10]]) {
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
    post_holes(thickness);
    main_holes(thickness);
    ear_holes(thickness);
    translate([0,full_y+32,0]) {
        rivet_holes(thickness);
    }
    translate([0,full_y+thickness,0]) {
        arduino_holes(thickness);
    }
}
translate([0,0,thickness/2])
difference() {
     color("red")
     base(thickness);
     base_cutout(thickness,rim);
}
 translate([0,0,thickness/2])
difference() {
    breadboard_posts();
    breadboard_notches();
}