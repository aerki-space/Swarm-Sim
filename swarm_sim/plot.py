"""Matplotlib figures of a finished run."""
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COLORS = {'PA': '#1d4ed8', 'EO': '#d97706', 'AQ': '#0f766e', 'CP': '#7c3aed'}
LABELS = {'PA': 'Radar (PA)', 'EO': 'Thermal (EO)', 'AQ': 'Air sampler (AQ)', 'CP': 'Relay (CP)'}


def draw(ax, sim, title=None):
    w = sim.world
    ax.imshow(w.canopy, origin='lower', extent=(0, w.width, 0, w.height), cmap='Greens', vmin=0, vmax=1.4,
              interpolation='bilinear')
    shown = set()
    for d in sim.drones:
        tr = np.array(sim.tracks[d.id])
        ax.plot(tr[:, 0], tr[:, 1], color=COLORS[d.kind], lw=0.8, alpha=0.85,
                label=LABELS[d.kind] if d.kind not in shown else None)
        shown.add(d.kind)
    for c in sim.candidates:
        if c.hotspot is None:
            color = '#475569' if c.rejected else ('#a21caf' if c.confirmed else '#dc2626')
            ax.plot(c.x, c.y, marker='x', color=color, ms=6, mew=1.5, ls='none')
    for h in w.hotspots:
        if h.confirmed_at is not None:
            ax.plot(h.x, h.y, marker='*', color='#dc2626', ms=16, mec='white', mew=0.8, ls='none')
        elif h.found_at is not None:
            ax.plot(h.x, h.y, marker='o', color='#f59e0b', ms=10, mec='white', ls='none')
        else:
            ax.plot(h.x, h.y, marker='o', mfc='none', mec='black', ms=10, mew=1.5, ls='none')
    r = sim.result()
    ax.set_title(title or f'{sim.fleet_name}: found {r.found}/{r.hotspots}, confirmed {r.confirmed}, '
                          f'unresolved false alarms {r.false_unresolved}', fontsize=10)
    ax.set_xlim(0, w.width)
    ax.set_ylim(0, w.height)
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.set_aspect('equal')


def legend_handles():
    from matplotlib.lines import Line2D
    return [Line2D([], [], color=c, lw=2, label=LABELS[k]) for k, c in COLORS.items()] + [
        Line2D([], [], marker='*', color='#dc2626', ms=12, ls='none', label='hotspot found + confirmed'),
        Line2D([], [], marker='o', color='#f59e0b', ms=8, ls='none', label='hotspot found, not confirmed'),
        Line2D([], [], marker='o', mfc='none', mec='black', ms=8, ls='none', label='hotspot missed'),
        Line2D([], [], marker='x', color='#475569', ms=7, ls='none', label='false alarm, rejected by sampler'),
        Line2D([], [], marker='x', color='#a21caf', ms=7, ls='none', label='false alarm, wrongly confirmed'),
        Line2D([], [], marker='x', color='#dc2626', ms=7, ls='none', label='false alarm, unresolved'),
    ]


def save_single(sim, path):
    fig, ax = plt.subplots(figsize=(7, 7))
    draw(ax, sim)
    ax.legend(handles=legend_handles(), loc='upper right', fontsize=7, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def save_comparison(sims, path, suptitle=None):
    fig, axes = plt.subplots(1, len(sims), figsize=(6.2 * len(sims), 6.6))
    for ax, sim in zip(np.atleast_1d(axes), sims):
        draw(ax, sim)
    fig.legend(handles=legend_handles(), loc='lower center', ncol=5, fontsize=8, frameon=False)
    if suptitle:
        fig.suptitle(suptitle, fontsize=12)
    fig.tight_layout(rect=(0, 0.1, 1, 0.97))
    fig.savefig(path, dpi=120)
    plt.close(fig)
