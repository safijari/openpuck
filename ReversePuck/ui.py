"""ui.py -- fullscreen touchscreen UI for the Steam Deck forwarder.

Big tappable tiles, one per bonded puck. A tile is tappable when its puck is LIVE (the RF link is up).
Tapping a live tile detaches the Deck controls and forwards to that puck; tapping anywhere while
forwarding stops and returns control to the Deck. Sized for the Deck's 1280x800 touchscreen but scales.
"""
import time
import pygame

BG = (16, 18, 24)
FG = (230, 232, 238)
DIM = (120, 126, 140)
ACCENT = (90, 170, 255)
LIVE = (70, 200, 120)
WARN = (235, 180, 70)
STOP = (235, 90, 90)
TILE = (32, 36, 46)
TILE_LIVE = (28, 54, 40)
TILE_SEL = (40, 70, 110)


def _font(sz, bold=False):
    f = pygame.font.SysFont("noto sans,dejavusans,sans", sz, bold=bold)
    return f


def run(app):
    pygame.init()
    pygame.display.set_caption("OpenController · Steam Deck")
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    W, H = screen.get_size()
    clock = pygame.time.Clock()
    f_title = _font(int(H * 0.05), bold=True)
    f_big = _font(int(H * 0.045), bold=True)
    f_med = _font(int(H * 0.03))
    f_small = _font(int(H * 0.022))

    tile_rects = []  # (rect, slot, tappable)
    del_rects = []   # (rect, slot) -- the per-tile "remove bond" ✕ target

    def draw():
        nonlocal tile_rects, del_rects
        screen.fill(BG)
        s = app.status
        connected = app.dongle_connected
        # compact status line (no big title banner): dongle presence first, then RF link
        if not connected:
            badge, col = "⨯ no dongle", WARN
        elif s["link_up"]:
            badge, col = "● linked  ch%d" % s["sess_ch"], LIVE
        else:
            badge, col = "○ searching…", DIM
        t = f_med.render(badge, True, col)
        screen.blit(t, (W - t.get_width() - 40, 24))

        # tiles -- live pucks float to the top, then by slot index
        tile_rects = []
        del_rects = []
        bonds = sorted(s["bonds"], key=lambda b: (not b["alive"], b["slot"]))
        top = int(H * 0.16)
        gap = int(H * 0.025)
        th = int((H * 0.42 - gap * max(0, len(bonds) - 1)) / max(1, len(bonds)))
        th = min(th, int(H * 0.20))
        if not connected:
            msg = f_big.render("Plug in the OpenController dongle", True, WARN)
            screen.blit(msg, (W // 2 - msg.get_width() // 2, H // 2 - 40))
            sub = f_small.render("Waiting for the nRF (Valve 28DE:1302) on USB… it'll appear here automatically.",
                                 True, DIM)
            screen.blit(sub, (W // 2 - sub.get_width() // 2, H // 2 + 20))
        elif not bonds:
            msg = f_big.render("No paired pucks", True, DIM)
            screen.blit(msg, (W // 2 - msg.get_width() // 2, H // 2 - 40))
            sub = f_small.render("Pair on a computer with Steam or PairTUI (scpair.py), then plug the puck into your host.",
                                 True, DIM)
            screen.blit(sub, (W // 2 - sub.get_width() // 2, H // 2 + 20))
        for n, b in enumerate(bonds):
            slot = b["slot"]
            y = top + n * (th + gap)
            rect = pygame.Rect(40, y, W - 80, th)
            sel = app.forwarding and app.fwd_slot == slot
            tappable = b["alive"] and not app.forwarding
            color = TILE_SEL if sel else (TILE_LIVE if b["alive"] else TILE)
            pygame.draw.rect(screen, color, rect, border_radius=18)
            pygame.draw.rect(screen, (ACCENT if (tappable or sel) else DIM), rect, width=2, border_radius=18)
            # glyph + serial
            glyph = "●" if b["alive"] else "○"
            gcol = LIVE if b["alive"] else DIM
            screen.blit(f_big.render(glyph, True, gcol), (rect.x + 30, rect.y + th // 2 - 22))
            screen.blit(f_big.render(b["serial"] or "puck %d" % slot, True, FG),
                        (rect.x + 90, rect.y + 20))
            state = ("FORWARDING — tap to stop" if sel
                     else "available — tap to forward" if tappable
                     else "live (forwarding elsewhere)" if b["alive"]
                     else "paired · offline")
            screen.blit(f_small.render("slot %d · %s" % (slot, state), True, DIM),
                        (rect.x + 90, rect.y + th - 38))
            tile_rects.append((rect, slot, tappable or sel))
            # remove-bond ✕ on the far right (only when idle -- can't un-bond while forwarding)
            if not app.forwarding:
                dsz = min(th - 24, 64)
                drect = pygame.Rect(rect.right - dsz - 18, rect.y + (th - dsz) // 2, dsz, dsz)
                pygame.draw.rect(screen, TILE, drect, border_radius=12)
                pygame.draw.rect(screen, STOP, drect, width=2, border_radius=12)
                x = f_big.render("✕", True, STOP)
                screen.blit(x, (drect.centerx - x.get_width() // 2,
                                drect.centery - x.get_height() // 2))
                del_rects.append((drect, slot))

        # firmware log panel (the in-app serial monitor) — fills the area below the tiles
        log_top = int(H * 0.60)
        log_bot = H - int(H * (0.12 if app.forwarding else 0.02))
        lh = f_small.get_height() + 2
        nlines = max(1, (log_bot - log_top - 24) // lh)
        screen.blit(f_small.render("firmware log", True, DIM), (40, log_top))
        lines = app.log[-nlines:]
        for i, ln in enumerate(lines):
            # colorize the lines that matter for connecting
            c = DIM
            if "SESSION live" in ln or "CONNECTED" in ln:
                c = LIVE
            elif "adopted" in ln or "E1 heard" in ln:
                c = ACCENT
            elif "lost" in ln or "no bond" in ln or "fail" in ln:
                c = WARN
            screen.blit(f_small.render(ln[:120], True, c), (40, log_top + 24 + i * lh))

        # forwarding banner (drawn last so it sits on top)
        if app.forwarding:
            bar = pygame.Rect(0, H - int(H * 0.1), W, int(H * 0.1))
            pygame.draw.rect(screen, STOP, bar)
            msg = f_big.render("FORWARDING — Deck controls detached · tap anywhere to stop", True, (255, 255, 255))
            screen.blit(msg, (W // 2 - msg.get_width() // 2, bar.y + bar.height // 2 - 22))
        pygame.display.flip()

    def handle_tap(pos):
        if app.forwarding:
            app.stop_forwarding()
            return
        # the remove-bond ✕ takes priority over the tile body it sits on
        for drect, slot in del_rects:
            if drect.collidepoint(pos):
                app.remove_bond(slot)
                return
        for rect, slot, tappable in tile_rects:
            if rect.collidepoint(pos) and tappable:
                app.toggle(slot)
                return

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                handle_tap(ev.pos)
            elif ev.type == pygame.FINGERDOWN:
                handle_tap((int(ev.x * W), int(ev.y * H)))

        app.pump_serial()
        app.pump_input()
        app.auto_release_on_drop()
        draw()
        # high tick so forwarding input stays smooth; UI redraw is cheap
        clock.tick(120)

    pygame.quit()
