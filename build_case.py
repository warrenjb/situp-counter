"""
Chest-strap case for the sit-up counter.
Holds: Seeed XIAO nRF52840 Sense (21 x 17.5 mm) + 402030 LiPo (4 x 20 x 30 mm) side by side,
with room for the JST PH 2.0 connector between them.

Outputs (all in mm, oriented for printing with no supports):
  situp_case_base.stl   tray with strap slots on both ends (strap up to 40 mm wide)
  situp_case_lid.stl    lid with press-fit lip, hold-down ribs, thin LED window
  gopro_adapter.stl     optional 2-prong GoPro-style mount to glue on the back
"""
from manifold3d import Manifold, CrossSection, JoinType
import numpy as np
import trimesh

# ---------------- parameters ----------------
WALL = 2.0          # side wall thickness
FLOOR = 2.0         # base floor thickness
IN_L, IN_W, IN_H = 62.0, 22.0, 6.6   # internal cavity (length, width, height above floor)
R_OUT = 3.0         # outer corner radius

LID_T = 1.6         # lid plate thickness
LIP_H, LIP_T, FIT = 2.0, 1.2, 0.15   # lid lip height, thickness, clearance per side

STRAP_W = 40.0      # widest strap that fits
SLOT_GAP = 4.0      # slot opening (strap thickness + slack)
BAR = 3.0           # outer bar width
TAB_T = 2.5         # tab thickness

BOARD_L, BOARD_W = 21.0, 17.5
BOARD_X0 = 0.5                      # board starts 0.5 mm from USB-end wall (cavity coords)
RAIL_H = 1.5                        # board raised so solder joints underneath clear the floor
PCB_T = 1.2
USB_W, USB_Z_CENTER_ABOVE_PCB = 12.0, 1.6
BATT_X0, BATT_L = 30.5, 31.0

OUT_L, OUT_W = IN_L + 2 * WALL, IN_W + 2 * WALL
BASE_H = FLOOR + IN_H


def rrect(l, w, r):
    """Rounded rectangle centered at origin (2D)."""
    r = min(r, l / 2 - 0.01, w / 2 - 0.01)
    if r <= 0:
        return CrossSection.square([l, w], center=True)
    return CrossSection.square([l - 2 * r, w - 2 * r], center=True).offset(r, JoinType.Round, circular_segments=48)


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


# Coordinates: case centered in x/y. Cavity spans x in [-IN_L/2, IN_L/2], y in [-IN_W/2, IN_W/2].
CX0 = -IN_L / 2          # cavity x origin (USB end)
CY0 = -IN_W / 2


def cav(x0, x1, y0, y1, z0, z1):
    """Box in cavity coordinates (x from USB-end wall, y from one side wall, z above floor)."""
    return box(CX0 + x0, CX0 + x1, CY0 + y0, CY0 + y1, FLOOR + z0, FLOOR + z1)


# ---------------- base ----------------
def build_base():
    outer = rrect(OUT_L, OUT_W, R_OUT).extrude(BASE_H)

    # Strap tabs on both ends, coplanar with the back so the case lies flat on the strap.
    tab_y = STRAP_W + 2 * BAR
    tab_x_out = SLOT_GAP + BAR            # how far the tab extends past the case end
    tabs = Manifold()
    for side in (-1, 1):
        end = side * OUT_L / 2
        overlap = 8.0                      # tab region fused under the case for strength
        x_in, x_out = end - side * overlap, end + side * tab_x_out
        xa, xb = min(x_in, x_out), max(x_in, x_out)
        plate = rrect(xb - xa, tab_y, 2.5).extrude(TAB_T).translate([(xa + xb) / 2, 0, 0])
        sx0 = end if side > 0 else end - SLOT_GAP
        slot = rrect(SLOT_GAP, STRAP_W, 1.5).extrude(TAB_T + 2).translate([sx0 + SLOT_GAP / 2, 0, -1])
        tabs = tabs + (plate - slot)

    body = outer + tabs

    # Cavity
    cavity = rrect(IN_L, IN_W, max(R_OUT - WALL, 0.5)).extrude(IN_H + 1).translate([0, 0, FLOOR])
    body = body - cavity

    # USB-C notch through the USB-end wall, open at the top (lid has a matching notch)
    usb_zc = FLOOR + RAIL_H + PCB_T + USB_Z_CENTER_ABOVE_PCB
    notch_z0 = usb_zc - 3.6
    body = body - box(-OUT_L / 2 - 1, CX0 + 0.01, -USB_W / 2, USB_W / 2, notch_z0, BASE_H + 1)

    # Board rails (under the long edges of the board)
    by0 = (IN_W - BOARD_W) / 2
    by1 = by0 + BOARD_W
    rails = (cav(BOARD_X0 + 0.5, BOARD_X0 + BOARD_L - 0.5, by0, by0 + 1.5, 0, RAIL_H)
             + cav(BOARD_X0 + 0.5, BOARD_X0 + BOARD_L - 0.5, by1 - 1.5, by1, 0, RAIL_H))
    # Side guides keep the board centered (0.15 mm clearance each side)
    guides = (cav(BOARD_X0 + 2, BOARD_X0 + BOARD_L - 2, by0 - 0.15 - 0.8, by0 - 0.15, 0, RAIL_H + PCB_T + 0.8)
              + cav(BOARD_X0 + 2, BOARD_X0 + BOARD_L - 2, by1 + 0.15, by1 + 0.15 + 0.8, 0, RAIL_H + PCB_T + 0.8))
    # End stop behind the board (gaps at the sides let the battery wires through)
    stop = cav(BOARD_X0 + BOARD_L + 0.3, BOARD_X0 + BOARD_L + 1.5, 6.0, IN_W - 6.0, 0, RAIL_H + PCB_T + 0.8)
    # Low fence at the battery end of the connector bay keeps the battery from sliding
    # (the JST connector lies crosswise in the bay between the end stop and this fence)
    fence = cav(BATT_X0 - 0.9, BATT_X0 - 0.1, 4.0, IN_W - 4.0, 0, 2.0)

    return body + rails + guides + stop + fence


# ---------------- lid ----------------
def build_lid():
    """Built in assembled position (plate sits on top of the base), then flipped for printing."""
    z_top = BASE_H
    plate = rrect(OUT_L, OUT_W, R_OUT).extrude(LID_T).translate([0, 0, z_top])

    lip_outer = rrect(IN_L - 2 * FIT, IN_W - 2 * FIT, max(R_OUT - WALL - FIT, 0.5))
    lip_inner = rrect(IN_L - 2 * FIT - 2 * LIP_T, IN_W - 2 * FIT - 2 * LIP_T, 0.5)
    lip = (lip_outer - lip_inner).extrude(LIP_H).translate([0, 0, z_top - LIP_H])

    lid = plate + lip

    # Hold-down ribs over the board's long edges
    by0 = (IN_W - BOARD_W) / 2
    by1 = by0 + BOARD_W
    board_top = RAIL_H + PCB_T
    for y0 in (by0 + 0.3, by1 - 1.3):
        lid = lid + cav(BOARD_X0 + 4, BOARD_X0 + BOARD_L - 3, y0, y0 + 1.0, board_top + 0.1, IN_H)
    # Hold-down ribs across the battery (battery top at ~4.0 mm)
    for xc in (BATT_X0 + 7, BATT_X0 + BATT_L - 7):
        lid = lid + cav(xc - 0.6, xc + 0.6, 4.0, IN_W - 4.0, 4.3, IN_H)

    # Thin LED window over the USB end of the board (print lid in white/natural so LEDs glow through)
    win = cav(BOARD_X0 + 1, BOARD_X0 + 12, by0 + 2.5, by1 - 2.5, IN_H - LIP_H - 1, IN_H + 1.0)
    lid = lid - win

    # USB notch continues through the lid edge and lip
    usb_zc = FLOOR + RAIL_H + PCB_T + USB_Z_CENTER_ABOVE_PCB
    lid = lid - box(-OUT_L / 2 - 1, CX0 + 2.0, -USB_W / 2, USB_W / 2, usb_zc - 3.6, z_top + LID_T + 1)

    # Flip for printing: outer face down on the bed
    lid = lid.rotate([180, 0, 0])
    b = lid.bounding_box()
    return lid.translate([0, 0, -b[2]])


# ---------------- GoPro-style adapter ----------------
def build_gopro():
    base = rrect(28, 22, 3).extrude(2.5)
    prong_t, gap = 3.0, 3.3
    r, hole_d, hc = 7.5, 5.4, 2.5 + 7.5
    prong_profile = (CrossSection.square([2 * r, hc - 2.5]).translate([-r, 2.5])
                     + CrossSection.circle(r, 64).translate([0, hc]))
    hole = CrossSection.circle(hole_d / 2, 48).translate([0, hc])
    prong2d = prong_profile - hole
    adapter = base
    for y_off in (gap / 2, -gap / 2 - prong_t):
        p = prong2d.extrude(prong_t)            # in XY, extruded along +Z
        p = p.rotate([90, 0, 0])                # profile -> XZ plane, thickness along -Y
        p = p.translate([0, y_off + prong_t, 0])
        adapter = adapter + p
    return adapter


def save(m, path):
    mesh = m.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts))
    tm.export(path)
    bb = m.bounding_box()
    dims = [bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]]
    print(f"{path}: {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm, watertight={tm.is_watertight}, "
          f"volume={tm.volume / 1000:.1f} cm3")


if __name__ == "__main__":
    import os
    out = os.path.dirname(os.path.abspath(__file__))
    base, lid, gp = build_base(), build_lid(), build_gopro()
    save(base, f"{out}/situp_case_base.stl")
    save(lid, f"{out}/situp_case_lid.stl")
    save(gp, f"{out}/gopro_adapter.stl")
