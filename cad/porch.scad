// OpenSCAD for a "porch" extension for a Turtlebot3 burger
//           It includes a lampholder.
thickness = 4;
bulb_width = 25;
porch_center_y = 30;  // The length of the rectangular portion
porch_x  = 90;
radius = 2.5;         // Hole radius
tab_x = 66;           // center-to-center
tab_y = 9;            // edge to center of holes
rim = 3;              // Width of rim
hex_offset = 13;      // Center-to-center

// This is the basic plate. Y argument is the protrusion (2/3 of the
// porch extent). This object cannot be translated once created.
module base(x,y,z) {
    round_radius=2;  // Radius of the corner arcs
    $fn=100;
    linear_extrude(height=z,center=false,convexity=10) {
        hull() {
        // place 6 circles in the corners, with the rounding radius
        translate([(-x/2)+(round_radius/2), -y+(round_radius/2), 0])
        circle(r=round_radius);
        
        translate([(x/2)-(round_radius/2), -y+(round_radius/2), 0])
        circle(r=round_radius);

        translate([(x/2)-(round_radius/2), 0-(round_radius/2), 0])
        circle(r=round_radius);
        
        translate([x/2-y+(round_radius/2), y/2-(round_radius/2), 0])
        circle(r=round_radius);

        translate([(-x/2)+y-(round_radius/2),y/2-(round_radius/2), 0])
        circle(r=round_radius);
     
        translate([(-x/2)+(round_radius/2), 0+(round_radius/2), 0])
        circle(r=round_radius);
        }
    }
}

// This is the basic plate less a rim width all around. This is meant to
// be subtracted from the standard plate.
module base_cutout(x,y,z,rim) {
    round_radius=2;  // Radius of the corner arcs
    $fn=100;
    linear_extrude(height=z,center=false,convexity=10) {
        hull() {
        // place 6 circles in the corners, with the rounding radius
        translate([(-x/2)+(round_radius/2)+rim, -y+(round_radius/2)+rim, 0])
        circle(r=round_radius);
        
        translate([(x/2)-(round_radius/2)-rim, -y+(round_radius/2)+rim, 0])
        circle(r=round_radius);

        translate([(x/2)-(round_radius/2)-rim, 0-(round_radius/2)-rim/2, 0])
        circle(r=round_radius);
        
        translate([x/2-y+(round_radius/2), y/2-(round_radius/2)-rim, 0])
        circle(r=round_radius);

        translate([(-x/2)+y-(round_radius/2),y/2-(round_radius/2)-rim, 0])
        circle(r=round_radius);
     
        translate([(-x/2)+(round_radius/2)+rim, 0+(round_radius/2)-rim/2, 0])
        circle(r=round_radius);
        }
    }
}

// x - center to between holes
// y - baseline to center of hole
// z - thickness
module holes(x,y,z) {
    hole_radius = 5;
    pad = 0.5;
    for(i=[[x+hole_radius+pad,y],[x-hole_radius-pad,y],
           [-x+hole_radius+pad,y],[-x-hole_radius-pad,y]]) {
        translate(i)
        cylinder(r=hole_radius,h=z);
    }
}

// This allows screws to main platform
module connector(z) {
    difference() {
        cylinder(h=z,r1=5,r2=5,$fn=6,center=true);
        cylinder(h=z,r1=2,r2=2,center=true);
        translate([0,0,z/2])
          cylinder(height=z/2,r1=2.5,r2=2.5,$fn=6);
    }
}
// Subtract this from the base platform
module connector_housing(z) {
    linear_extrude(height=z,center=true) {
        circle(r=5,$fn=6);
    }
}
module lampholder(width,z) {
    bulb_tail_radius = 8;
    difference() {
        union() {
            translate([0,-width/4,0])
                cube([width,width/2,z],center=true);
            translate([0,-width/2,-z/2])
                cylinder(r=width/2,h=z);
           }
           translate([0,-width/2,-z/2])
            cylinder(r=bulb_tail_radius,h=z);
    }
}

// Support between the lampholders.
module support(x,y,z) {
    height=6;
    cube([x,y,height],center=true);
}
  

translate([0,0,thickness/2])
difference() {
    base(porch_x,porch_center_y,thickness);
    translate([0,-porch_center_y,0])
        holes(tab_x/2,tab_y,thickness);
    translate([-hex_offset,-porch_center_y,thickness/2])
        connector_housing(2*thickness);
    translate([hex_offset,-porch_center_y,thickness/2])
        connector_housing(2*thickness);
}
translate([0,0,-thickness/2]) {
    difference() {
        base(porch_x,porch_center_y,thickness);
        base_cutout(porch_x,porch_center_y,thickness,rim);
        translate([hex_offset,-porch_center_y,thickness/2])
            connector_housing(2*thickness);
        translate([-hex_offset,-porch_center_y,thickness/2])
            connector_housing(2*thickness);
    }
}
translate([0,0,thickness/2])
    rotate(90,[1,0,0]) 
        lampholder(bulb_width,thickness);
translate([0,porch_center_y/2-0.5,thickness/2])
    rotate(90,[1,0,0]) 
      lampholder(bulb_width,thickness);
    
translate([bulb_width/2-rim/2,porch_center_y/4,0])
    support(rim,porch_center_y/2,6);
translate([-bulb_width/2+rim/2,porch_center_y/4,0])
    support(rim,porch_center_y/2,6);
translate([hex_offset,-porch_center_y,thickness])
    rotate(180,[0,1,0])
        connector(thickness);
translate([-hex_offset,-porch_center_y,0])
    connector(thickness);