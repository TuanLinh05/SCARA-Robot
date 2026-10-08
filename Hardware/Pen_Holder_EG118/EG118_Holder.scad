// SCARA pen holder, revision 1. Units: mm.
// +Y = outward from fork; +Z = up. Pen held by a 90 degree V-saddle.
// Verified barrel range: 8.5 to 9.5 mm; diameter supplied by user: ~9 mm.
// part: "base", "cap_print", "test_base", "test_cap_print", "assembly"
part = "assembly";
pen_diameter = 9.0;
height = 36;
fork_radius = 12.4;
fork_rear_flat = 11.4;
mount_centers_z = [5.5, 30.5];
mount_slot_d = 3.6;
mount_slot_stroke = 2.4;
clamp_hole_d = 3.4;
nut_af = 5.8;
nut_depth = 2.6;
$fn = 96;

pen_y = 14 + pen_diameter / sqrt(2);
pressure_y = pen_y + pen_diameter / 2;

assert(pen_diameter >= 8.5 && pen_diameter <= 9.5,
       "Revision 1: use pen_diameter between 8.5 and 9.5 mm.");

module round_rect(x0,y0,x1,y1,r=1.5) {
    translate([x0+r,y0+r]) offset(r=r,$fn=32)
        square([x1-x0-2*r,y1-y0-2*r]);
}
module tongue_profile() {
    intersection() {
        union() {
            intersection() {
                circle(r=fork_radius);
                translate([-20,-20]) square([40,20]);
            }
            translate([-fork_radius,0]) square([2*fork_radius,13.5]);
        }
        translate([-20,-fork_rear_flat]) square([40,40]);
    }
}
module x_hole(z) {
    translate([-25,0,z]) rotate([0,90,0]) cylinder(h=50,d=mount_slot_d);
}
module y_hole(x,z) {
    translate([x,-2,z]) rotate([-90,0,0]) cylinder(h=44,d=clamp_hole_d);
}
module nut_socket(x,z) {
    translate([x,11.99,z]) rotate([-90,0,0])
        cylinder(h=nut_depth+0.01,r=nut_af/sqrt(3),$fn=6);
}
module base(h=height, coupon=false) {
    difference() {
        linear_extrude(h) union() {
            tongue_profile();
            round_rect(-17,12,17,23.2);
        }
        translate([0,0,-1]) linear_extrude(h+2)
            polygon([[0,14],[22,36],[-22,36]]);
        translate([0,0,-1]) linear_extrude(h+2)
            round_rect(-7,-6,7,6,2);
        for (z=coupon ? [4] : mount_centers_z)
            hull() {
                x_hole(z-mount_slot_stroke/2);
                x_hole(z+mount_slot_stroke/2);
            }
        for (x=[-13,13],z=coupon ? [h/2] : [12,24]) {
            y_hole(x,z);
            nut_socket(x,z);
        }
    }
}
module cap(h=height,coupon=false) {
    difference() {
        linear_extrude(h) round_rect(-17,pressure_y,17,pressure_y+4,1.2);
        for (x=[-13,13],z=coupon ? [h/2] : [12,24]) y_hole(x,z);
    }
}
module cap_on_bed(h=height,coupon=false) {
    translate([17,h,pressure_y+4]) rotate([90,0,0]) cap(h,coupon);
}
if (part=="base") translate([17,fork_rear_flat,0]) base();
else if (part=="cap_print") cap_on_bed();
else if (part=="test_base") translate([17,fork_rear_flat,0]) base(8,true);
else if (part=="test_cap_print") cap_on_bed(8,true);
else if (part=="assembly") {
    color([1,.55,.12]) base();
    color([.1,.65,.6]) cap();
    // Reference barrel only. Do not export this scene for printing.
    %translate([0,pen_y,-17]) cylinder(h=87,d=pen_diameter);
} else assert(false,"Unknown part selector");
