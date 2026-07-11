/* ============================================================================
   Rigol Foot Pedal - printable enclosure (ESP32-C3 Super Mini)
   ============================================================================
   Parametric OpenSCAD design. Free viewer/editor: https://openscad.org

   I don't have your actual footswitch / battery holder / board in hand to
   measure, so treat this as a FIRST-PASS STARTING POINT, not a verified
   final design:
     1. Open this file in OpenSCAD and check it renders cleanly (F5 preview,
        then F6 for a full render) before printing anything.
     2. Work through the "VERIFY AGAINST YOUR PARTS" block below and update
        every number to match what you actually have.
     3. The actuator boss (under the pedal) and the hinge fit are called out
        below as "trim to fit" - no amount of code can perfectly predict
        your specific switch's travel/preload, so expect to sand/trim a
        millimeter or two after a test fit. This is normal for this kind
        of press-fit mechanism, not a sign anything is wrong.

   THREE PARTS - printed separately. Set `part` near the bottom of this file
   to "top", "bottom", "pedal", or "all" (all three laid out side by side
   for a quick look - do NOT print "all" as one piece, export each part on
   its own).

     top    - outer shell: footswitch-mounting bridge, 2x LED holes,
              2x hinge posts, 4x corner posts
     bottom - flat base plate: closes the box underneath, holds the board +
              battery holder, 4x corner holes
     pedal  - the hinged lever you actually step on

   Hardware you'll need (off the shelf, not modeled here):
     - 2x M3 screws (~12-16mm) + 2x M3 heat-set threaded inserts -> hinge
     - 4x M3 screws (~16-20mm) + 4x M3 heat-set threaded inserts -> join
       top+bottom; the same 4 screws also pass through standard screw-on
       rubber feet against the underside of the bottom plate
     - Your footswitch, ESP32-C3 Super Mini, 3x AA battery holder, 2x 3mm LEDs

   Print notes:
     - "top" has an internal bridge (the footswitch mount) - it'll likely
       want supports under it, or print top upside-down (open face down)
       so the bridge is naturally supported from below by the print bed
       side. Try both orientations in your slicer's preview.
     - Everything else is a straightforward flat orientation.
   ============================================================================ */

$fn = 48;

// ---------------- VERIFY AGAINST YOUR PARTS ----------------

// Footswitch: panel-mount momentary pushbutton, spring return
sw_hole_d = 12;          // panel-mount hole diameter - 12mm is common, check yours
sw_body_drop = 20;       // roughly how far the switch body hangs below the bridge

// ESP32-C3 Super Mini board (rough footprint - most Super Minis have no
// mounting holes, so it just sits on the floor, held with foam tape/hot glue)
board_l = 23;
board_w = 18;

// USB-C cutout, cut through the FRONT wall (opposite the hinge) so you can
// still plug in without opening the case
usbc_x = 8;              // distance from the left edge of the front wall
usbc_z = 8;              // height above the bottom plate
usbc_w = 10;
usbc_h = 4;

// 3x AA battery holder (inline, wired) - MEASURE YOUR ACTUAL HOLDER
batt_l = 60;
batt_w = 18;
batt_h = 16;

// 2x 3mm LEDs, mounted in the fixed ledge behind the hinge (stays visible
// no matter what position the pedal is in)
led_d = 3.4;             // hole diameter (3mm LED + a little clearance)
led_spacing = 12;

// M3 heat-set threaded inserts - check your insert's own datasheet, these
// are typical numbers for the common "single knurl" M3 inserts
insert_d = 4.2;           // pilot hole diameter
insert_depth = 6;         // pilot hole depth
screw_clear_d = 3.4;      // clearance hole for an M3 screw shaft (no threads)

// ---------------- GENERAL BUILD PARAMETERS ----------------

wall = 2.4;               // side wall thickness
floor = 2.6;               // bottom plate thickness
wall_low_h = 6;            // height of the low perimeter rim (non-hinge sides)
post_d = 10;                // diameter of the corner/hinge posts
corner_inset = post_d/2 + 3;
ledge_depth = 16;          // fixed strip behind the hinge that holds the LEDs
front_depth = 18;          // zone at the front for the switch bridge + pedal tip
bay_depth = batt_l + 6;    // main compartment depth, sized to the battery holder
hinge_drop_from_top = 8;   // how far down from the very top the hinge pivot sits
pedal_t = 4;               // pedal plate thickness
boss_d = 8;                // actuator boss diameter (under the pedal)
boss_drop = 10;             // TRIM TO FIT - see note at the top of the file
front_gap = 5;              // clearance gap between the pedal's tip and the front wall

// ---------------- DERIVED DIMENSIONS ----------------

internal_w = board_w + batt_w + 14;               // board + battery side by side
internal_l = ledge_depth + bay_depth + front_depth;
internal_h = batt_h + 10;

outer_w = internal_w + 2*wall;
outer_l = internal_l + 2*wall;
outer_h = internal_h + floor;

hinge_axis_z = outer_h - hinge_drop_from_top;

sw_y = wall + front_depth/2;                       // switch bridge center, box Y
ledge_y0 = outer_l - ledge_depth - wall;            // where the back ledge block starts

echo(str("Outer footprint: ", outer_w, " x ", outer_l, " x ", outer_h, " mm"));

// ================================ TOP ================================
module chassis_top() {
    difference() {
        union() {
            // Low perimeter rim (picture frame), all four sides
            difference() {
                cube([outer_w, outer_l, wall_low_h]);
                translate([wall, wall, -1])
                    cube([internal_w, internal_l, wall_low_h + 2]);
            }

            // Tall back ledge - fixed, does not move with the pedal.
            // Holds the LED holes and the hinge posts.
            translate([0, ledge_y0, 0])
                cube([outer_w, outer_l - ledge_y0, outer_h]);

            // Footswitch mounting bridge, spans the internal width near
            // the front. The switch's body hangs down below it into the
            // open interior; the pedal hovers just above it.
            translate([wall, sw_y - 5, outer_h - 8])
                cube([internal_w, 10, 6]);

            // 4 corner posts - join to chassis_bottom, also carry the
            // rubber feet (same screws, from underneath)
            for (x = [corner_inset, outer_w - corner_inset])
                for (y = [corner_inset, outer_l - corner_inset])
                    translate([x, y, 0])
                        cylinder(d = post_d, h = wall_low_h);

            // 2 hinge posts, back corners, full height (embedded in the ledge)
            for (x = [corner_inset, outer_w - corner_inset])
                translate([x, outer_l - corner_inset, 0])
                    cylinder(d = post_d, h = outer_h);
        }

        // Footswitch hole through the bridge
        translate([outer_w/2, sw_y, outer_h - 10])
            cylinder(d = sw_hole_d, h = 12);

        // 2 LED holes, drilled down through the ledge's top face
        led_y = outer_l - (ledge_depth + wall)/2;
        for (x = [outer_w/2 - led_spacing/2, outer_w/2 + led_spacing/2])
            translate([x, led_y, outer_h - 10])
                cylinder(d = led_d, h = 12);

        // USB-C cutout, front wall
        translate([usbc_x, -0.1, usbc_z])
            cube([usbc_w, wall + 2, usbc_h]);

        // 4 corner insert pilot holes, drilled UP from the bottom face
        // (accessible once mated with chassis_bottom)
        for (x = [corner_inset, outer_w - corner_inset])
            for (y = [corner_inset, outer_l - corner_inset])
                translate([x, y, -0.1])
                    cylinder(d = insert_d, h = insert_depth + 0.1);

        // 2 hinge insert pilot holes, drilled horizontally through each
        // post from its outer face
        for (x = [corner_inset, outer_w - corner_inset])
            translate([x - post_d/2 - 1, outer_l - corner_inset, hinge_axis_z])
                rotate([0, 90, 0])
                    cylinder(d = insert_d, h = post_d + 2);
    }
}

// ============================== BOTTOM ==============================
module chassis_bottom() {
    difference() {
        cube([outer_w, outer_l, floor]);
        // 4 corner clearance holes - screw passes through from underneath;
        // a rubber foot's own screw shares the same hole
        for (x = [corner_inset, outer_w - corner_inset])
            for (y = [corner_inset, outer_l - corner_inset])
                translate([x, y, -1])
                    cylinder(d = screw_clear_d, h = floor + 2);
    }
}

// =============================== PEDAL ===============================
module pedal() {
    pedal_w = outer_w - 0.6;                 // hair narrower than the box for clearance
    pedal_l = outer_l - ledge_depth - wall - front_gap;
    pedal_start_boxY = outer_l - ledge_depth - wall - pedal_l; // box-Y of the pedal's front tip
    boss_y = sw_y - pedal_start_boxY;         // switch position, translated into pedal-local Y
    tab_x = [corner_inset, pedal_w - corner_inset];

    union() {
        // Main plate - what you step on
        translate([0.3, 0, 0])
            cube([pedal_w, pedal_l, pedal_t]);

        // Actuator boss, underside, above the switch.
        // TRIM TO FIT: this height sets how much the switch gets pressed;
        // print it a touch long and sand it down until the switch just
        // clicks when the pedal is pressed fully down.
        translate([0.3 + pedal_w/2, boss_y, -boss_drop])
            cylinder(d = boss_d, h = boss_drop + 0.1);

        // 2 hinge tabs, back edge - line up with chassis_top's hinge
        // posts (same corner_inset, same hinge_drop_from_top)
        for (x = tab_x)
            translate([0.3 + x - post_d/2 - 1, pedal_l - post_d/2, -hinge_drop_from_top])
                rotate([0, 90, 0])
                    cylinder(d = screw_clear_d, h = post_d + 2);
    }
}

// ================================ RENDER ================================
part = "all"; // "top" | "bottom" | "pedal" | "all" (all = layout preview only)

if (part == "top") chassis_top();
else if (part == "bottom") chassis_bottom();
else if (part == "pedal") pedal();
else {
    chassis_top();
    translate([outer_w + 15, 0, 0]) chassis_bottom();
    translate([0, outer_l + 15, 0]) pedal();
}
