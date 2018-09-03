// PCB holder for front-back sound level detector
// Two identical holders clip together to form a box that holds the
// printed circuit boards. External horns at either end are glued
// in front of the microphones
mic = 10;       // Microphone diameter
thickness = 2;
length = 80;
width = 20;
height = 22;
horn_inset = 14;   // Square, round mic hole in center
groove = 1;        // Rail and groove fitting
$fn=150;

module bottom() {
    difference() {
        cube([length,width,thickness],true);
        translate([-length/2+thickness/4,-width/2,-thickness/2]) {
            cube([groove,width,groove]);
        }
        translate([-length/2,-thickness/4,-thickness/2]) {
            cube([length,thickness/2,thickness/2]);
        }
        // Holes
        translate([-21,-6,0]) {
            cylinder(thickness,thickness,thickness,true);
        }
        translate([-21,6,0]) {
            cylinder(thickness,thickness,thickness,true);
        }
        translate([21,-6,0]) {
            cylinder(thickness,2,2,true);
        }
        translate([21,6,0]) {
            cylinder(thickness,2,2,true);
        }
    }
}
module end() {
    union() {
        difference() {
           cube([thickness,width,height],true);
           translate([-thickness/2,-width/2,-height/2]) {
               cube([thickness,width,thickness]);
            };
            translate([-thickness/2,-horn_inset/2,-horn_inset/2]) {
               cube([thickness,horn_inset,horn_inset]); 
            };
        }
        translate([-thickness/4,-width/2,-height/2+thickness/2]) {
            cube([groove,width,groove]);
        }
    }
}
module side() {
    difference() {
        cube([length,thickness,height],true); 
        translate([-length/2,-thickness/2,-height/2]) {
            cube([thickness,thickness,height]);
        };
        translate([length/2-thickness,-thickness/2,-height/2]) {
            cube([thickness,thickness,thickness]);
        }
        translate([0,thickness/2,0]) {
            rotate([90,0,0]) cylinder(thickness,thickness,thickness);
        }
    }
}

union() {
    translate([0,0,height/2-thickness/2]) {
        bottom();
    }
    translate([0,width/2-thickness/2,0]) {
        side();
    }
    translate([length/2-thickness/2,0,0]) {
        end();
    }
}