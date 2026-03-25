#!/usr/bin/env python3
"""Generate diagram images for the manifest slides."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT_DIR = "docs/presentations/img"
os.makedirs(OUT_DIR, exist_ok=True)

# Colors
BG = (255, 255, 255, 0)  # transparent
WHITE = (255, 255, 255)
DARK = (38, 38, 38)
GRAY = (120, 120, 120)
BLUE = (26, 115, 232)
GREEN = (52, 168, 83)
RED = (234, 67, 53)
ORANGE = (251, 188, 4)
LIGHT_BLUE = (210, 227, 252)
LIGHT_GREEN = (206, 234, 214)
LIGHT_ORANGE = (254, 239, 206)
LIGHT_GRAY = (240, 240, 240)
CODE_BG = (30, 30, 46)
CODE_FG = (205, 214, 244)

W, H = 1600, 700


def get_font(size=16, bold=False):
    """Try to load a good font, fall back to default."""
    paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf" if bold
        else "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    ]
    for p in paths:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def get_mono_font(size=14):
    paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    ]
    for p in paths:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def draw_box(draw, x, y, w, h, fill, outline, text, font, text_color=DARK):
    """Draw a rounded rectangle with centered text."""
    draw.rounded_rectangle([x, y, x + w, y + h], radius=8, fill=fill, outline=outline, width=2)
    bbox = draw.textbbox((0, 0), text, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text((x + (w - tw) // 2, y + (h - th) // 2), text, font=font, fill=text_color)


def draw_arrow(draw, x1, y1, x2, y2, color=GRAY, width=2):
    """Draw line with arrowhead."""
    draw.line([x1, y1, x2, y2], fill=color, width=width)
    # Arrowhead
    import math
    angle = math.atan2(y2 - y1, x2 - x1)
    size = 10
    draw.polygon([
        (x2, y2),
        (x2 - size * math.cos(angle - 0.4), y2 - size * math.sin(angle - 0.4)),
        (x2 - size * math.cos(angle + 0.4), y2 - size * math.sin(angle + 0.4)),
    ], fill=color)


def gen_rcl_interception():
    """RCL Interception Architecture diagram."""
    img = Image.new("RGB", (W, H), WHITE)
    draw = ImageDraw.Draw(img)
    font = get_font(16)
    font_b = get_font(16, bold=True)
    font_s = get_font(13)
    mono = get_mono_font(13)

    # ROS 2 Node box
    draw_box(draw, 50, 80, 280, 160, LIGHT_BLUE, BLUE, "", font)
    draw.text((110, 90), "ROS 2 Node", font=font_b, fill=BLUE)
    draw.text((80, 120), "rcl_publish()", font=mono, fill=DARK)
    draw.text((80, 145), "rcl_take()", font=mono, fill=DARK)
    draw.text((80, 175), "normal ROS code", font=font_s, fill=GRAY)

    # LD_PRELOAD arrow
    draw.text((370, 100), "LD_PRELOAD", font=font_b, fill=RED)
    draw_arrow(draw, 330, 160, 490, 160, RED, 3)

    # Interception .so box
    draw_box(draw, 490, 80, 320, 160, LIGHT_ORANGE, ORANGE, "", font)
    draw.text((510, 90), "Interception .so", font=font_b, fill=DARK)
    draw.text((510, 120), "hooks rcl_publish/take", font=font_s, fill=DARK)
    draw.text((510, 145), "extracts header.stamp", font=font_s, fill=DARK)
    draw.text((510, 170), "writes InterceptionEvent", font=font_s, fill=DARK)

    # SPSC arrow
    draw.text((855, 100), "zero-copy", font=font_s, fill=GRAY)
    draw.text((855, 118), "SPSC ring", font=font_s, fill=GRAY)
    draw_arrow(draw, 810, 160, 960, 160, GREEN, 3)

    # play_launch consumer box
    draw_box(draw, 960, 80, 320, 160, LIGHT_GREEN, GREEN, "", font)
    draw.text((990, 90), "play_launch", font=font_b, fill=DARK)
    draw.text((990, 118), "consumer task", font=font_s, fill=DARK)
    draw.text((990, 150), "FrontierPlugin", font=mono, fill=DARK)
    draw.text((990, 172), "  per-topic timestamp frontier", font=font_s, fill=GRAY)
    draw.text((990, 195), "StatsPlugin", font=mono, fill=DARK)
    draw.text((990, 217), "  message counts & rates", font=font_s, fill=GRAY)

    # Bottom: what gets captured
    y = 310
    draw.text((50, y), "Captured per message:", font=font_b, fill=DARK)
    items = [
        ("mono_time", "wall-clock when pub/take happened"),
        ("header.stamp", "data timestamp from message"),
        ("topic_name", "which topic"),
        ("node_name", "which node"),
    ]
    for i, (key, desc) in enumerate(items):
        draw.text((80, y + 30 + i * 28), key, font=mono, fill=BLUE)
        draw.text((280, y + 30 + i * 28), desc, font=font_s, fill=GRAY)

    # Bottom right: output files
    draw.text((700, y), "Output:", font=font_b, fill=DARK)
    outputs = [
        "frontier_summary.json",
        "stats_summary.json",
        "manifest_audit.json",
    ]
    for i, name in enumerate(outputs):
        draw.text((730, y + 30 + i * 28), name, font=mono, fill=GREEN)

    # Contract check arrow
    draw.text((700, y + 130), "→ contract violation checks", font=font_b, fill=RED)

    img.save(f"{OUT_DIR}/rcl-interception.png")
    print(f"  {OUT_DIR}/rcl-interception.png")


def gen_scope_hierarchy():
    """Scope hierarchy / manifest composition diagram."""
    img = Image.new("RGB", (W, 600), WHITE)
    draw = ImageDraw.Draw(img)
    font = get_font(15)
    font_b = get_font(15, bold=True)
    font_s = get_font(12)
    mono = get_mono_font(12)

    # Top-level scope
    draw.rounded_rectangle([30, 20, W - 30, 580], radius=10, outline=GRAY, width=2)
    draw.text((50, 30), "planning_simulator", font=font_b, fill=DARK)

    # Sensing scope
    draw.rounded_rectangle([50, 70, 380, 220], radius=8, fill=LIGHT_BLUE, outline=BLUE, width=2)
    draw.text((70, 80), "sensing", font=font_b, fill=BLUE)
    draw_box(draw, 70, 115, 130, 40, WHITE, BLUE, "lidar_driver", font_s)
    draw_box(draw, 220, 115, 140, 40, WHITE, BLUE, "camera_driver", font_s)
    draw.text((70, 170), "exports: pointcloud, image", font=font_s, fill=GRAY)
    draw.text((70, 190), "io: source, 10 Hz", font=mono, fill=GRAY)

    # Perception scope
    draw.rounded_rectangle([410, 70, 1050, 370], radius=8, fill=LIGHT_ORANGE, outline=ORANGE, width=2)
    draw.text((430, 80), "perception", font=font_b, fill=DARK)

    # Lidar sub-scope
    draw.rounded_rectangle([430, 110, 720, 230], radius=6, fill=WHITE, outline=ORANGE, width=1)
    draw.text((445, 115), "lidar", font=font_b, fill=ORANGE)
    draw_box(draw, 445, 142, 110, 30, LIGHT_GRAY, GRAY, "cropbox", font_s)
    draw_box(draw, 445, 178, 110, 30, LIGHT_GRAY, GRAY, "centerpoint", font_s)
    draw.text((570, 150), "50 ms", font=mono, fill=RED)

    # Camera sub-scope
    draw.rounded_rectangle([740, 110, 1030, 230], radius=6, fill=WHITE, outline=ORANGE, width=1)
    draw.text((755, 115), "camera", font=font_b, fill=ORANGE)
    draw_box(draw, 755, 142, 100, 30, LIGHT_GRAY, GRAY, "rectifier", font_s)
    draw_box(draw, 755, 178, 100, 30, LIGHT_GRAY, GRAY, "yolo", font_s)
    draw.text((870, 150), "30 ms", font=mono, fill=RED)

    # Fusion + tracker
    draw_box(draw, 430, 250, 150, 40, LIGHT_GRAY, GRAY, "fusion_node", font_s)
    draw.text((590, 258), "all_ready, 20 ms", font=mono, fill=RED)
    draw_box(draw, 430, 310, 150, 40, LIGHT_GRAY, GRAY, "tracker", font_s)
    draw.text((590, 318), "periodic: 10 Hz", font=mono, fill=RED)

    # Perception exports
    draw.text((750, 310), "exports: tracked_objects", font=font_s, fill=GRAY)
    draw.text((750, 330), "io: latency_ms: 85", font=mono, fill=GRAY)

    # Planning scope
    draw.rounded_rectangle([1080, 70, 1380, 220], radius=8, fill=LIGHT_GREEN, outline=GREEN, width=2)
    draw.text((1100, 80), "planning", font=font_b, fill=DARK)
    draw_box(draw, 1100, 115, 120, 40, WHITE, GREEN, "prediction", font_s)
    draw_box(draw, 1100, 165, 130, 40, WHITE, GREEN, "motion_planner", font_s)
    draw.text((1250, 125), "100 ms", font=mono, fill=RED)

    # Arrows between scopes
    draw_arrow(draw, 380, 140, 410, 140, BLUE, 2)  # sensing → perception
    draw_arrow(draw, 1050, 140, 1080, 140, ORANGE, 2)  # perception → planning

    # Critical path at bottom
    draw.line([50, 430, W - 50, 430], fill=LIGHT_GRAY, width=2)
    draw.text((50, 445), "Critical path:", font=font_b, fill=DARK)
    draw.text((200, 445), "max(50, 30) + 20 + 100 =", font=font, fill=DARK)
    draw.text((510, 445), "170 ms", font=font_b, fill=RED)

    # Topic wiring at bottom
    draw.text((50, 490), "Topics wire scopes:", font=font_b, fill=DARK)
    wires = [
        "pointcloud:    sensing/pointcloud → perception/input",
        "tracked_obj:   perception/output → planning/input",
    ]
    for i, w in enumerate(wires):
        draw.text((80, 520 + i * 25), w, font=mono, fill=GRAY)

    img.save(f"{OUT_DIR}/scope-hierarchy.png")
    print(f"  {OUT_DIR}/scope-hierarchy.png")


def gen_contract_4way():
    """4-way contract diagnosis diagram."""
    img = Image.new("RGB", (800, 400), WHITE)
    draw = ImageDraw.Draw(img)
    font = get_font(16)
    font_b = get_font(18, bold=True)
    font_s = get_font(14)

    # Header
    draw.text((50, 20), "4-Way Contract Diagnosis", font=font_b, fill=DARK)

    # Grid
    cells = [
        (0, 0, "Assumption", GREEN, "OK"),
        (1, 0, "Guarantee", GREEN, "OK"),
        (2, 0, "", None, "Nominal"),
        (0, 1, "Assumption", GREEN, "OK"),
        (1, 1, "Guarantee", RED, "FAIL"),
        (2, 1, "", None, "Node bug"),
        (0, 2, "Assumption", RED, "FAIL"),
        (1, 2, "Guarantee", GREEN, "OK"),
        (2, 2, "", None, "Node robust"),
        (0, 3, "Assumption", RED, "FAIL"),
        (1, 3, "Guarantee", RED, "FAIL"),
        (2, 3, "", None, "Upstream problem"),
    ]

    x0, y0 = 50, 70
    cw, ch = 200, 60
    colors = {
        "Nominal": GREEN,
        "Node bug": RED,
        "Node robust": BLUE,
        "Upstream problem": ORANGE,
    }

    for col, row, label, color, value in cells:
        x = x0 + col * (cw + 20)
        y = y0 + row * (ch + 10)

        if col < 2:
            fill = LIGHT_GREEN if color == GREEN else (255, 220, 220)
            outline = color
            draw.rounded_rectangle([x, y, x + cw, y + ch], radius=6, fill=fill, outline=outline, width=2)
            draw.text((x + 10, y + 8), label, font=font_s, fill=GRAY)
            draw.text((x + 10, y + 30), value, font=font_b, fill=color)
        else:
            # Result
            draw.text((x + 30, y + 5), "→", font=font_b, fill=GRAY)
            result_color = colors.get(value, DARK)
            draw.text((x + 60, y + 5), value, font=font_b, fill=result_color)

    img.save(f"{OUT_DIR}/contract-4way.png")
    print(f"  {OUT_DIR}/contract-4way.png")


if __name__ == "__main__":
    print("Generating diagrams:")
    gen_rcl_interception()
    gen_scope_hierarchy()
    gen_contract_4way()
    print("Done.")
