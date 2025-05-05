import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import matplotlib.patches as mpatches

def draw_box(ax, text, xy, width=1.8, height=0.8):
    """Draws a labeled box."""
    box = FancyBboxPatch(xy, width, height,
                         boxstyle="round,pad=0.1",
                         edgecolor='black',
                         facecolor='lightgray',
                         linewidth=1.5)
    ax.add_patch(box)
    ax.text(xy[0]+width/2, xy[1]+height/2, text,
            ha='center', va='center', fontsize=9)
    return xy[0]+width, xy[1]+height/2  # Return output point

def draw_arrow(ax, start, end):
    """Draws an arrow from start to end."""
    ax.annotate('', xy=end, xytext=start,
                arrowprops=dict(arrowstyle="->", linewidth=1.2))

def generate_qam_block_diagram():
    fig, ax = plt.subplots(figsize=(14, 6))
    ax.set_xlim(0, 18)
    ax.set_ylim(0, 7)
    ax.axis('off')

    # Tx Side
    x, y = 0.5, 3.5
    x1, y1 = draw_box(ax, "Binary Bits", (x, y))
    x2, y2 = draw_box(ax, "4-bit Grouping", (x1 + 0.5, y - 0.2))

    # Split to I and Q
    xi, yi = x2 + 1.0, y2 + 0.8
    xq, yq = x2 + 1.0, y2 - 1.0
    draw_box(ax, "I: bits 1-2", (xi, yi))
    draw_box(ax, "Q: bits 3-4", (xq, yq))
    draw_arrow(ax, (x2, y2), (xi, yi + 0.4))
    draw_arrow(ax, (x2, y2), (xq, yq + 0.4))

    # 4-PAM Mapping
    xi += 2.0
    xq += 2.0
    draw_box(ax, "4-PAM Mapper", (xi, yi))
    draw_box(ax, "4-PAM Mapper", (xq, yq))

    # Pulse Shaping
    xi += 2.0
    xq += 2.0
    draw_box(ax, "Pulse Shaping\n(p or pₛ)", (xi, yi))
    draw_box(ax, "Pulse Shaping\n(p or pₛ)", (xq, yq))

    # Carrier modulation
    xi += 2.0
    xq += 2.0
    draw_box(ax, "× cos(2πfₜt)", (xi, yi))
    draw_box(ax, "× sin(2πfₜt)", (xq, yq))

    # Combine I & Q to form Tx signal
    draw_box(ax, "Tx Signal\nIcos - Qsin", (xi + 2.0, yi - 1.0))

    # Receiver Side
    rx_base_x = xi + 5.0
    draw_box(ax, "× cos(2πfₜt)", (rx_base_x, yi))
    draw_box(ax, "× sin(2πfₜt)", (rx_base_x, yq))

    draw_box(ax, "Matched Filter", (rx_base_x + 2, yi))
    draw_box(ax, "Matched Filter", (rx_base_x + 2, yq))

    draw_box(ax, "Sampler +\nThreshold", (rx_base_x + 4, yi))
    draw_box(ax, "Sampler +\nThreshold", (rx_base_x + 4, yq))

    draw_box(ax, "4-PAM to Bits", (rx_base_x + 6, yi))
    draw_box(ax, "4-PAM to Bits", (rx_base_x + 6, yq))

    # Final Recovered Bits
    ax.text(rx_base_x + 8, yi + 0.3, "Bits Out", fontsize=11)

    # Arrows between everything
    for i in range(5):
        draw_arrow(ax, (1.4 + i*2.0, 4), (1.6 + i*2.0, 4))
        draw_arrow(ax, (1.4 + i*2.0, 2.2), (1.6 + i*2.0, 2.2))
    draw_arrow(ax, (11.4, 4), (12.6, 4))
    draw_arrow(ax, (11.4, 2.2), (12.6, 2.2))
    draw_arrow(ax, (13.4, 4), (14.6, 4))
    draw_arrow(ax, (13.4, 2.2), (14.6, 2.2))
    draw_arrow(ax, (15.4, 4), (16.6, 4))
    draw_arrow(ax, (15.4, 2.2), (16.6, 2.2))
    draw_arrow(ax, (17.4, 4.4), (18, 4.4))

    plt.tight_layout()
    plt.savefig("qam_block_diagram.png", dpi=300)
    plt.show()

generate_qam_block_diagram()
