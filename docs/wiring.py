"""Generates docs/wiring.svg (doorbell wiring). Run: python3 docs/wiring.py"""
import os
W, H = 1100, 990
o = []
def a(x): o.append(x)
def text(x, y, s, size=14, anchor="start", weight=400, fill="#1f2328", italic=False):
    st = ' font-style="italic"' if italic else ''
    a(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" font-weight="{weight}" fill="{fill}"{st}>{s}</text>')
def line(x1, y1, x2, y2, c, w=3, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ''
    a(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{c}" stroke-width="{w}" stroke-linecap="round"{d}/>')
def poly(pts, c, w=3):
    a(f'<polyline points="{" ".join(f"{x},{y}" for x,y in pts)}" fill="none" stroke="{c}" stroke-width="{w}" stroke-linejoin="round" stroke-linecap="round"/>')
def box(x, y, w, h, c, fill, rx=6, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ''
    a(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{c}" stroke-width="1.5"{d}/>')
def dot(x, y, c="#1f2328", r=4): a(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{c}"/>')

RED, BLK, BLU, PUR, GRN, ORG, GRY = "#cf222e", "#1f2328", "#0969da", "#8250df", "#1a7f37", "#bc4c00", "#8c959f"
FILL = {RED:"#ffebe9", BLK:"#f6f8fa", BLU:"#ddf4ff", PUR:"#fbefff", GRN:"#dafbe1", ORG:"#fff1e5", GRY:"#f6f8fa"}

a(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="-apple-system, Segoe UI, Helvetica, Arial, sans-serif">')
a(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
text(40, 44, "Doorbell wiring – production board", 22, weight=600)
text(40, 68, "Seeed XIAO ESP32C6 + MikroE NFC 4 Click (ST25R3916, reworked to I2C) + 1S LiPo. Pins as in relay-doorbell firmware, 2026-10-08.", 13, fill="#57606a")

# XIAO body
bx, by, bw, bh = 450, 150, 200, 470
a(f'<rect x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="14" fill="#2d333b" stroke="#1f2328" stroke-width="2"/>')
a(f'<rect x="{bx+bw/2-30}" y="{by-22}" width="60" height="30" rx="5" fill="#afb8c1" stroke="#57606a" stroke-width="1.5"/>')
text(bx+bw/2, by-2, "USB-C", 11, "middle", 600, "#24292f")
text(bx+bw/2, by+34, "XIAO ESP32C6", 16, "middle", 600, "#ffffff")
text(bx+bw/2, by+54, "top view", 12, "middle", 400, "#adbac7", italic=True)

left = [("D0","A0 / GPIO0"),("D1","GPIO1"),("D2","GPIO2"),("D3","GPIO21"),("D4","GPIO22"),("D5","GPIO23"),("D6","GPIO16")]
right = [("5V",""),("GND",""),("3V3",""),("D10","GPIO18"),("D9","GPIO20"),("D8","GPIO19"),("D7","GPIO17")]
ys = [by+95+58*i for i in range(7)]
for (n,g),y in zip(left,ys):
    dot(bx, y, "#d4a72c", 7)
    text(bx+14, y+5, n, 14, weight=600, fill="#ffffff")
    if g: text(bx+46, y+5, g, 10, fill="#adbac7")
for (n,g),y in zip(right,ys):
    dot(bx+bw, y, "#d4a72c", 7)
    text(bx+bw-14, y+5, n, 14, "end", 600, "#ffffff")
    if g: text(bx+bw-48, y+5, g, 10, "end", fill="#adbac7")

def lbox(y, c, title, sub):
    x0, w0 = 60, 300
    line(x0+w0, y, bx-8, y, c)
    box(x0, y-24, w0, 48, c, FILL[c])
    text(x0+12, y-4, title, 14, weight=600, fill=c if c!=BLK else BLK)
    text(x0+12, y+15, sub, 12, fill="#424a53")
def rbox(y, c, title, sub):
    x0, w0 = 730, 330
    line(bx+bw+8, y, x0, y, c)
    box(x0, y-24, w0, 48, c, FILL[c])
    text(x0+12, y-4, title, 14, weight=600, fill=c if c!=BLK else BLK)
    text(x0+12, y+15, sub, 12, fill="#424a53")
def unused(x, y, anchor):
    text(x, y+5, "not used", 12, anchor, fill=GRY, italic=True)

lbox(ys[0], ORG, "Battery sense", "junction of R1, R2, C1 (divider below)")
lbox(ys[1], PUR, "NFC 4 Click: IRQ", "wakes the doorbell when a phone arrives")
lbox(ys[2], GRN, "Doorbell button", "other leg to GND; diagonal pins on a 4-pin switch")
unused(bx-20, ys[3], "end")
lbox(ys[4], BLU, "NFC 4 Click: SCL", "I2C clock")
lbox(ys[5], BLU, "NFC 4 Click: SDA", "I2C data")
unused(bx-20, ys[6], "end")
unused(bx+bw+20, ys[0], "start")
rbox(ys[1], BLK, "GND (shared)", "Click GND, button, R2 + C1: splice here")
rbox(ys[2], RED, "NFC 4 Click: 3V3", "the Click runs from the XIAO's 3.3 V")
for y in ys[3:]: unused(bx+bw+20, y, "start")

# underside pads
py = by + bh + 22
box(bx+28, py, 64, 28, RED, "#ffebe9", 4, "5 3")
text(bx+60, py+19, "BAT+", 13, "middle", 600, RED)
box(bx+108, py, 64, 28, BLK, "#f6f8fa", 4, "5 3")
text(bx+140, py+19, "BAT−", 13, "middle", 600, BLK)
text(bx+16, py+19, "underside pads", 12, "end", fill="#57606a", italic=True)
# battery
cx, cy = 730, py-8
box(cx, cy, 330, 76, "#57606a", "#f6f8fa", 8)
text(cx+14, cy+24, "LiPo cell, 1S (e.g. 503450)", 14, weight=600)
text(cx+14, cy+44, "JST-PH pigtail: red to BAT+, black to BAT−", 12, fill="#424a53")
text(cx+14, cy+62, "check the plug's polarity before connecting", 12, fill=RED)
poly([(bx+60, py+28),(bx+60, py+70),(cx-24, py+70),(cx-24, cy+56),(cx, cy+56)], RED)
poly([(bx+140, py+28),(bx+140, py+52),(cx-40, py+52),(cx-40, cy+26),(cx, cy+26)], BLK)

# divider inset
ix, iy = 40, 750
box(ix, iy, 480, 210, "#d0d7de", "#ffffff", 10)
text(ix+16, iy+26, "Battery divider (on D0)", 15, weight=600)
text(ix+16, iy+46, "full 4.2 V reads 2.1 V at D0; draws ~2 µA; no part has a polarity", 12, fill="#57606a")
nx, top, node, bot = ix+150, iy+74, iy+134, iy+190
text(ix+30, top+5, "BAT+", 13, weight=600, fill=RED)
line(ix+70, top, nx, top, RED, 2.5)
line(nx, top, nx, top+10, RED, 2.5)
box(nx-9, top+10, 18, 36, "#57606a", "#eaeef2", 2)
line(nx, top+46, nx, bot-46, ORG, 2.5)
text(nx+16, top+33, "R1  1 MΩ", 12)
dot(nx, node)
line(nx, node, ix+300, node, ORG, 2.5)
text(ix+306, node+5, "to D0", 13, weight=600, fill=ORG)
box(nx-9, node+10, 18, 36, "#57606a", "#eaeef2", 2)
line(nx, node, nx, node+10, ORG, 2.5)
text(nx+16, node+33, "R2  1 MΩ", 12)
c2 = nx+170
dot(c2-60, node)
line(c2-60, node, c2-60, node+22, ORG, 2.5)
line(c2-74, node+22, c2-46, node+22, BLK, 2.5)
line(c2-74, node+30, c2-46, node+30, BLK, 2.5)
text(c2-38, node+31, "C1  100 nF", 12)
line(nx, node+46, nx, bot, BLK, 2.5)
line(c2-60, node+30, c2-60, bot, BLK, 2.5)
line(nx, bot, c2-60, bot, BLK, 2.5)
text(nx+40, bot+16, "GND", 13, weight=600)

# stack-up inset
sx, sy = 560, 750
box(sx, sy, 500, 210, "#d0d7de", "#ffffff", 10)
text(sx+16, sy+26, "Stack-up in the doorbell, outside to inside", 15, weight=600)
layers = [("doorbell's plastic cover", "#eaeef2"), ("NFC 4 Click – printed spiral side facing out", "#dafbe1"),
          ("ferrite sheet – antenna area only, against the Click", "#d0d7de"), ("XIAO + battery behind the chip half", "#ddf4ff")]
for i,(t,f) in enumerate(layers):
    yy = sy+42+i*32
    box(sx+16, yy, 468, 26, "#8c959f", f, 4)
    text(sx+28, yy+18, t, 12)
text(sx+16, sy+196, "keep the XIAO's small ceramic antenna clear of the battery, ferrite and metal", 11, fill="#57606a", italic=True)
a('</svg>')
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "wiring.svg"), "w", encoding="utf-8").write('<?xml version="1.0" encoding="UTF-8"?>\n' + "\n".join(o) + "\n")

