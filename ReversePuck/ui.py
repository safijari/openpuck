"""ui.py -- fullscreen touchscreen UI for the Steam Deck forwarder.

Big tappable tiles, one per bonded puck. A tile is tappable when its puck is LIVE (the RF link is up).
Tapping a live tile detaches the Deck controls and forwards to that puck (a single tap -- connecting is
cheap and reversible). The two DESTRUCTIVE actions -- stopping/disconnecting a forward, and removing a
bond (unpair) -- instead require a HOLD (HOLD_SECS): an errant screen tap mid-game used to drop the forward
(which on the puck side tears down a USB device and could reboot the MCU), so a brief touch must no longer
trigger it. A progress fill shows the hold; lifting before it completes cancels. Sized for the Deck's
1280x800 touchscreen but scales.

Screen timeout, while forwarding with the puck link up: once per forward, when the link first comes up, the
screen counts down SCREEN_OFF_SECS and goes black; a tap during the countdown keeps it on instead. After
that a tap just toggles the screen off/on. A hold works whether the screen is on or black (it lights the
screen for the progress fill and stops forwarding); a hold let go early leaves the screen as it was.

Status icons (dots, the remove ✕) are drawn with primitives, not font glyphs -- the Deck's available fonts
don't carry ● ○ ✕ etc., so rendering them as text just showed blank/boxes.
"""
import time
import pygame

# Destructive actions (stop forwarding, remove bond) require holding this long; a tap no longer triggers them.
HOLD_SECS = 2.0
# When forwarding to a live puck starts, the screen goes fully black after this countdown (OLED pixels off;
# an LCD Deck keeps its backlight at the user's brightness).
SCREEN_OFF_SECS = 5.0
# a press released sooner than this is a TAP (toggles the screen); a longer one that's let go before
# HOLD_SECS is a cancelled hold and changes nothing
TAP_SECS = 0.4
# lines of firmware log shown at the bottom of the screen
LOG_LINES = 4

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


def _draw_dot(screen, center, r, color, filled=True):
    """Status dot drawn with primitives -- font glyphs (● ○) don't render on the Deck's fallback font."""
    if filled:
        pygame.draw.circle(screen, color, center, r)
    else:
        pygame.draw.circle(screen, color, center, r, max(2, r // 3))


def _draw_cross(screen, rect, color, width):
    """Remove-bond ✕ drawn as two diagonals (the font glyph doesn't render)."""
    pad = max(6, rect.width // 4)
    pygame.draw.line(screen, color, (rect.x + pad, rect.y + pad),
                     (rect.right - pad, rect.bottom - pad), width)
    pygame.draw.line(screen, color, (rect.right - pad, rect.y + pad),
                     (rect.x + pad, rect.bottom - pad), width)


def _tile_rects(n, area, gap=16):
    """Split area among n tiles: one fills it; otherwise two columns, the left holding n // 2 tiles and the
    right the rest (3 -> 1 left, 2 right; 4 -> 2 and 2), each column's tiles stacked to its full height."""
    if n <= 1:
        return [area] if n else []
    cw = (area.width - gap) // 2
    rects = []
    for col, count in enumerate((n // 2, n - n // 2)):
        x = area.x + col * (cw + gap)
        th = (area.height - gap * (count - 1)) // count
        rects += [pygame.Rect(x, area.y + i * (th + gap), cw, th) for i in range(count)]
    return rects


def _wrap(text, font, max_w):
    """Lines for text in font within max_w: its " - " parts packed onto as few lines as fit, a part that's
    too wide on its own wrapped at spaces."""
    pieces = []
    for part in text.split(" - "):
        line = ""
        for word in part.split(" "):
            if line and font.size(line + " " + word)[0] > max_w:
                pieces.append(line)
                line = word
            else:
                line = (line + " " + word).strip()
        pieces.append(line)
    lines = []
    for p in pieces:
        if lines and font.size(lines[-1] + " - " + p)[0] <= max_w:
            lines[-1] += " - " + p
        else:
            lines.append(p)
    return lines


def _fit_text(text, fonts, color, max_w, max_h, max_lines=4):
    """Render text in the largest of fonts whose wrapped lines fit max_w x max_h in at most max_lines
    (beyond that a huge font reads as a word list; better one size down). Else the smallest font."""
    for font in fonts:
        lines = _wrap(text, font, max_w)
        if (len(lines) <= max_lines and all(font.size(t)[0] <= max_w for t in lines)
                and len(lines) * font.get_linesize() <= max_h):
            break
    return [font.render(t, True, color) for t in lines]


def run(app):
    pygame.init()
    pygame.display.set_caption("ReversePuck · Steam Deck")
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    W, H = screen.get_size()
    clock = pygame.time.Clock()
    f_huge = _font(int(H * 0.09), bold=True)
    f_title = _font(int(H * 0.05), bold=True)
    f_big = _font(int(H * 0.045), bold=True)
    f_med = _font(int(H * 0.03))
    f_small = _font(int(H * 0.022))
    # tile headline sizes, largest first (all bold); each tile uses the largest that fits
    f_heads = (f_huge, _font(int(H * 0.06), bold=True), f_big, _font(int(H * 0.035), bold=True),
               _font(int(H * 0.028), bold=True), _font(int(H * 0.022), bold=True))

    tile_rects = []  # (rect, slot, tappable)
    del_rects = []   # (rect, slot) -- the per-tile "remove bond" ✕ target
    # Active hold-to-confirm gesture, or None. dict(kind="stop"|"unpair", slot, start). Read by draw() for
    # the progress fill; set in handle_down()/cleared in handle_up() and on completion.
    hold = None
    # Screen timeout (see the module docstring). countdown: None = not yet started this forward, a time =
    # running since then, False = over (timed out or tapped away). press: (start time, screen was black)
    # of the touch in progress, so its release can tell a tap from a cancelled hold.
    blank = False
    countdown = None
    was_forwarding = False
    press = None

    def timeout_active():
        """The screen timeout only runs while forwarding to a puck whose link is up."""
        return app.forwarding and app.status.get("link_up")

    def screen_off_in():
        """Seconds until the screen goes black, or None when no countdown is running."""
        if not timeout_active() or not countdown:
            return None
        return max(0.0, SCREEN_OFF_SECS - (time.monotonic() - countdown))

    def hold_progress():
        """0.0..1.0 of the current hold, or None if no hold is active."""
        if hold is None:
            return None
        return max(0.0, min(1.0, (time.monotonic() - hold["start"]) / HOLD_SECS))

    def draw():
        nonlocal tile_rects, del_rects
        screen.fill(BG)
        s = app.status
        connected = app.dongle_connected
        # compact status line (no big title banner): dongle presence first, then RF link. The leading status
        # dot is drawn (font glyphs don't render); filled = linked, ring = no-dongle / searching.
        if not connected:
            badge, col = "no dongle", WARN
        elif s["link_up"]:
            badge, col = "linked  ch%d" % s["sess_ch"], LIVE
        else:
            badge, col = "searching...", DIM
        t = f_med.render(badge, True, col)
        bx = W - t.get_width() - 40
        screen.blit(t, (bx, 24))
        _draw_dot(screen, (bx - 18, 24 + t.get_height() // 2), 8, col,
                  filled=connected and s["link_up"])
        # legend (top-left): how the gestures work
        screen.blit(f_small.render("Tap to connect, hold to disconnect or remove", True, DIM),
                    (40, 30))

        # tiles -- live pucks float to the top, then by slot index
        tile_rects = []
        del_rects = []
        bonds = sorted(s["bonds"], key=lambda b: (not b["alive"], b["slot"]))
        # Layout, top to bottom: header, tiles, LOG_LINES of firmware log, the forwarding banner's space
        # (kept free even when idle so the tiles don't resize when a forward starts). The tiles fill all the
        # space between header and log: one tile alone, else two columns, the left taking the smaller half.
        lh = f_small.get_height() + 2
        log_bot = H - int(H * 0.12)
        log_top = log_bot - 24 - LOG_LINES * lh
        top = int(H * 0.16)
        tiles_bottom = log_top - 16
        if not connected:
            msg = f_big.render("Plug in the ReversePuck dongle", True, WARN)
            screen.blit(msg, (W // 2 - msg.get_width() // 2, H // 2 - 40))
            sub = f_small.render("Waiting for the nRF (Valve 28DE:1302) on USB... it'll appear here automatically.",
                                 True, DIM)
            screen.blit(sub, (W // 2 - sub.get_width() // 2, H // 2 + 20))
        elif not bonds:
            msg = f_big.render("No paired pucks", True, DIM)
            screen.blit(msg, (W // 2 - msg.get_width() // 2, H // 2 - 40))
            sub = f_small.render("Pair on a computer with Steam or PairTUI (scpair.py), then plug the puck into your host.",
                                 True, DIM)
            screen.blit(sub, (W // 2 - sub.get_width() // 2, H // 2 + 20))
        for b, rect in zip(bonds, _tile_rects(len(bonds), pygame.Rect(40, top, W - 80, tiles_bottom - top))):
            slot = b["slot"]
            th = rect.height
            sel = app.forwarding and app.fwd_slot == slot
            tappable = b["alive"] and not app.forwarding
            color = TILE_SEL if sel else (TILE_LIVE if b["alive"] else TILE)
            pygame.draw.rect(screen, color, rect, border_radius=18)
            pygame.draw.rect(screen, (ACCENT if (tappable or sel) else DIM), rect, width=2, border_radius=18)
            # status dot (drawn -- font ● ○ don't render); the state is the headline (it says what tapping
            # does), the puck's serial the small print under it; the block sits centred in the tile
            gcol = LIVE if b["alive"] else DIM
            _draw_dot(screen, (rect.x + 46, rect.centery), min(13, th // 4), gcol, filled=b["alive"])
            state = ("CONNECTED - hold anywhere to disconnect" if sel
                     else "available - tap to connect" if tappable
                     else "live (connected to another puck)" if b["alive"]
                     else "paired - offline")
            dsz = min(64, max(20, th - 16)) if not app.forwarding else 0
            serial = f_small.render(b["serial"] or "puck %d" % slot, True, DIM)
            heads = _fit_text("slot %d - %s" % (slot, state), f_heads, FG,
                              rect.width - 90 - (dsz + 36 if dsz else 24),
                              th - serial.get_height() - 16)
            y = rect.centery - (sum(h.get_height() for h in heads) + serial.get_height() + 4) // 2
            for h in heads:
                screen.blit(h, (rect.x + 90, y))
                y += h.get_height()
            screen.blit(serial, (rect.x + 90, y + 4))
            tile_rects.append((rect, slot, tappable or sel))
            # remove-bond ✕ on the far right (only when idle -- can't un-bond while forwarding)
            if not app.forwarding:
                drect = pygame.Rect(rect.right - dsz - 18, rect.centery - dsz // 2, dsz, dsz)
                pygame.draw.rect(screen, TILE, drect, border_radius=12)
                # while this ✕ is being held, fill it from the bottom up as a hold-to-unpair progress meter
                held = (hold is not None and hold["kind"] == "unpair"
                        and hold["slot"] == slot)
                hp = hold_progress()
                if held and hp:
                    fh = int(dsz * hp)
                    fill = pygame.Rect(drect.x, drect.bottom - fh, dsz, fh)
                    pygame.draw.rect(screen, STOP, fill, border_radius=12)
                pygame.draw.rect(screen, STOP, drect, width=2, border_radius=12)
                _draw_cross(screen, drect, FG if held else STOP, 4)
                del_rects.append((drect, slot))

        # screen timeout: the countdown big and centred right under the tiles -- over the log for its few
        # seconds when the tiles leave no room; afterwards just a hint, centred between header and tiles
        show_log = True
        if timeout_active() and hold is None:
            left = screen_off_in()
            if left is not None:
                cd = [f_huge.render("Screen off in %d" % (left + 0.99), True, WARN),
                      f_med.render("Tap to keep the screen on", True, DIM)]
                cd_h = sum(c.get_height() + 4 for c in cd)
                y = (tiles_bottom if bonds else int(H * 0.40)) + 8
                if y + cd_h > log_top:
                    y, show_log = log_top, False
                for c in cd:
                    screen.blit(c, (W // 2 - c.get_width() // 2, y))
                    y += c.get_height() + 4
            else:
                surf = f_med.render("Tap to turn the screen off", True, DIM)
                screen.blit(surf, (W // 2 - surf.get_width() // 2, (top + 60) // 2 - surf.get_height() // 2))

        # firmware log panel (the in-app serial monitor): the last LOG_LINES lines, above the banner space
        lines = app.log[-LOG_LINES:] if show_log else []
        if show_log:
            screen.blit(f_small.render("firmware log", True, DIM), (40, log_top))
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
            hp = hold_progress()
            if hold is not None and hold["kind"] == "stop" and hp:
                # fill the banner left->right as the stop hold progresses; lighten so the fill reads as a meter
                pygame.draw.rect(screen, (255, 150, 150),
                                 pygame.Rect(bar.x, bar.y, int(bar.width * hp), bar.height))
                left = max(0.0, HOLD_SECS * (1.0 - hp))
                msg = f_big.render("KEEP HOLDING TO DISCONNECT - %.0fs" % (left + 0.99),
                                   True, (255, 255, 255))
            else:
                msg = f_big.render("CONNECTED - Deck controls go to the puck - hold to disconnect",
                                   True, (255, 255, 255))
            screen.blit(msg, (W // 2 - msg.get_width() // 2, bar.y + bar.height // 2 - 22))
        pygame.display.flip()

    def handle_down(pos):
        # Press handler. CONNECT (toggle a live tile) fires immediately -- it's cheap and reversible. The
        # destructive actions (STOP a forward, REMOVE a bond) instead ARM a hold; they only fire from
        # check_hold() once HOLD_SECS elapses, so an errant tap can't trigger them. Actions still only
        # ENQUEUE intents so they never race the IO thread's serial/libusb access.
        nonlocal hold, blank, press
        if press is None:
            press = (time.monotonic(), blank)
        # light the screen while touched (a hold shows its progress fill); the release decides what stays
        blank = False
        if hold is not None:
            return  # a hold is already in progress; ignore re-entrant/duplicate down events
        if app.forwarding:
            # hold anywhere on the screen to stop forwarding
            hold = {"kind": "stop", "slot": None, "start": time.monotonic()}
            return
        # the remove-bond ✕ takes priority over the tile body it sits on -- hold it to unpair
        for drect, slot in del_rects:
            if drect.collidepoint(pos):
                hold = {"kind": "unpair", "slot": slot, "start": time.monotonic()}
                return
        # live tile: connecting/forwarding stays a single tap
        for rect, slot, tappable in tile_rects:
            if rect.collidepoint(pos) and tappable:
                app.request_toggle(slot)
                return

    def handle_up():
        # Lifting (or a stray up) before HOLD_SECS cancels the pending destructive action. While the screen
        # timeout runs, a quick release is a TAP: it ends the countdown (screen stays on) or toggles the
        # screen. A slower release is a cancelled hold and puts the screen back as it was.
        nonlocal hold, blank, countdown, press
        pending_stop = hold is not None and hold["kind"] == "stop"
        hold = None
        if press is None:
            return
        start, was_blank = press
        press = None
        if not pending_stop or not timeout_active():
            return  # not forwarding, or the hold fired and stopped it
        if time.monotonic() - start >= TAP_SECS:
            blank = was_blank
        elif screen_off_in() is not None:
            countdown = False  # tapped during the countdown: keep the screen on
        else:
            blank = not was_blank

    def check_screen_timeout():
        # Run the countdown once per forward, from when the link to the puck first comes up; go black when
        # it runs out (never while a finger is down). The screen comes back on if the link drops.
        nonlocal was_forwarding, countdown, blank
        if app.forwarding and not was_forwarding:
            countdown = None
        was_forwarding = app.forwarding
        if not timeout_active():
            blank = False
            if countdown:
                countdown = False  # link dropped mid-countdown: don't restart it
            return
        if countdown is None:
            countdown = time.monotonic()
        if press is None and screen_off_in() == 0.0:
            countdown = False
            blank = True

    def check_hold():
        # Fire the held action once it has been sustained long enough, then disarm. Also cancel a hold whose
        # target went away (the forward auto-released, or forwarding started so a bond can't be removed).
        nonlocal hold
        if hold is None:
            return
        if hold["kind"] == "stop" and not app.forwarding:
            hold = None
            return
        if hold["kind"] == "unpair" and app.forwarding:
            hold = None
            return
        if time.monotonic() - hold["start"] >= HOLD_SECS:
            if hold["kind"] == "stop":
                app.request_stop()
            elif hold["kind"] == "unpair":
                app.request_remove(hold["slot"])
            hold = None

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False
            # SDL also reports every touch as a synthetic mouse click (ev.touch); take the finger event
            # only, or each tap would count twice
            elif ev.type == pygame.MOUSEBUTTONDOWN and not getattr(ev, "touch", False):
                handle_down(ev.pos)
            elif ev.type == pygame.FINGERDOWN:
                handle_down((int(ev.x * W), int(ev.y * H)))
            elif (ev.type == pygame.MOUSEBUTTONUP and not getattr(ev, "touch", False)
                  or ev.type == pygame.FINGERUP):
                handle_up()

        check_hold()  # promote a sustained hold into the actual stop/unpair intent
        check_screen_timeout()

        if blank:
            # all black (OLED pixels off); redraw rarely -- 10 Hz still notices a touch promptly
            screen.fill((0, 0, 0))
            pygame.display.flip()
            clock.tick(10)
            continue

        # Render only -- serial I/O + input forwarding run on app._io_loop (a separate thread) so the
        # vsync'd flip() below can't throttle forwarding to 60Hz. A bad draw must never kill the app.
        try:
            draw()
        except Exception as ex:
            app._flog("UI draw error: %r" % ex)
            app.note = "recovered from error: %s" % ex
        clock.tick(60)  # UI render rate; forwarding is decoupled on the IO thread now

    pygame.quit()
