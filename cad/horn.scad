// Cup-shaped horn to be glued in front of level-detector microphone.
// Dimensions in mm
// cylinder(height, bottom radius,top radius, centered?)
// cube([width,depth,height],centered?)
// sphere(radius)

mic = 10;       // Microphone diameter
diameter=20;    // Outer horn diameter
thickness = 2;
$fn=150;

// Hole for the microphone (plus padding)
module hole() {
    cylinder(diameter/2,mic/2,mic/2,false);
}
module plate() {
   cube([mic+2*thickness,mic+2*thickness,thickness],true);
}
    

module cup() {
  difference() {
    union() {
        difference() {
            sphere(diameter/2);
            translate([0, 0, diameter/2]) {
                cube(diameter, true);
            }
        }
    }
    sphere(diameter/2 - thickness);
  }
}


difference() {
    union() {
        translate([0,0,-diameter/2+thickness]) {
            plate();
        }
        cup();
    }
    translate([0,0,-diameter/2-thickness]) {
        hole();
    };
    translate([-mic/2-thickness,-thickness/2,-diameter/2+thickness/2]) {
       cube([mic+2*thickness,thickness,thickness]);
    }
}