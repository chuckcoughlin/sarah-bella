// Cup-shaped horn to be glued in front of level-detector microphone.
// Dimensions in mm

mic = 10;       // Microphone diameter
diameter=25;
thickness = 4;
$fn=150;

// Hole for the microphone (plus padding)
module hole() {
    cylinder(diameter,mic,mic,false);
}
module plate() {
    cube([diameter,diameter,thickness],true);
}
    

module cup() {
  difference() {
    union() {
        difference() {
            sphere(diameter);
            translate([0, 0, diameter]) {
                cube(2*diameter, true);
            }
        }
    }
    sphere(diameter - thickness);
  }
}


difference() {
    union() {
        translate([0,0,-diameter+thickness]) {
            plate();
        }
        cup();
    }
    translate([0,0,-diameter-thickness]) {
        hole();
    }
}